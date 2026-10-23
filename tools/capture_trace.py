#!/usr/bin/env python3
"""Capture a frozen RAM trace without flashing/resetting; motors must already be stopped.
If VS Code/OpenOCD is already debugging, use trace-save in that session instead.
"""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def quote_gdb(path):
    return '"' + str(path).replace('\\', '\\\\').replace('"', '\\"') + '"'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', choices=['lqr', 'imu'], default='lqr')
    parser.add_argument('--elf', type=Path, default=ROOT/'build/Debug/balance_car.elf')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--timeout', type=float, default=90, help='GDB capture timeout in seconds')
    args = parser.parse_args()
    elf = args.elf.resolve()
    if not elf.is_file():
        parser.error('ELF not found: ' + str(elf))
    for executable in ('openocd', 'gdb-multiarch'):
        if not shutil.which(executable):
            parser.error(executable + ' is not installed')
    for comm in Path('/proc').glob('[0-9]*/comm'):
        try:
            active = comm.read_text().strip() == 'openocd'
        except OSError:
            continue
        if active:
            parser.error('OpenOCD already running: use trace-save in the existing VS Code debug console; do not attach a second debugger.')
    output = (args.output_dir or ROOT/'docs/experiments/traces'/(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_'+args.source)).resolve()
    output.mkdir(parents=True, exist_ok=False)
    # Reserve a free loopback port number; OpenOCD failure is checked before GDB runs.
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    command = ['openocd', '-c', 'bindto 127.0.0.1', '-c', f'gdb_port {port}',
               '-c', 'tcl_port disabled', '-c', 'telnet_port disabled',
               '-f', str(ROOT/'.vscode/openocd.cfg'), '-c', 'init']
    log_path = output/'openocd.log'
    process = None
    gdb_started = False
    detached = False
    try:
        with log_path.open('x') as log:
            process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+15
            # Do not probe the GDB port: a probe itself would be a debugger connection.
            while f'Listening on port {port}' not in log_path.read_text(errors='replace'):
                if process.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('OpenOCD failed to become ready; see ' + str(log_path))
                time.sleep(.1)
            # GDB attachment may halt the CPU. Never run this while motor control is active.
            script = f'''set pagination off
set confirm off
target remote 127.0.0.1:{port}
source {quote_gdb(ROOT/'tools/trace_gdb.py')}
python
try:
    # Read-only flash comparison prevents interpreting RAM through a different build's symbols.
    comparison = gdb.execute('compare-sections .text', to_string=True)
    gdb.write(comparison)
    if 'MIS-MATCHED' in comparison or 'matched.' not in comparison:
        raise RuntimeError('ELF does not match target .text; use the exact ELF that was flashed.')
    save_trace({args.source!r}, {str(output)!r}, make_report=False)
except Exception:
    gdb.execute('monitor resume')
    gdb.execute('detach')
    raise
end
monitor resume
detach
'''
            with tempfile.TemporaryDirectory(prefix='balance-trace-') as temporary:
                commands = Path(temporary)/'capture.gdb'
                commands.write_text(script)
                gdb_started = True
                result = subprocess.run(['gdb-multiarch', '--batch', str(elf), '-x', str(commands)],
                                        cwd=ROOT, text=True, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, timeout=args.timeout)
            (output/'gdb.log').write_text(result.stdout)
            print(result.stdout)
            detached = result.returncode == 0
            if result.returncode or not (output/(args.source+'_trace.json')).is_file():
                raise RuntimeError('Capture failed; see ' + str(output/'gdb.log'))
    finally:
        # Only stop the server started by this invocation, never an existing debug server.
        if process is not None and process.poll() is None:
            if gdb_started and not detached:
                # The timed-out GDB process is already dead. Release only our own server's target.
                try:
                    cleanup = subprocess.run(['gdb-multiarch', '--batch', str(elf),
                        '-ex', f'target remote 127.0.0.1:{port}', '-ex', 'monitor resume', '-ex', 'detach'],
                        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
                    (output/'cleanup.log').write_text(cleanup.stdout)
                    if cleanup.returncode:
                        print('Could not confirm target resumed; see cleanup.log.')
                except (OSError, subprocess.TimeoutExpired):
                    print('Could not confirm target resumed after capture failure.')
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    from trace_report import report
    path = output/(args.source+'_trace.json')
    for artifact in report(path):
        print(artifact)
    print('Trace bundle:', output)


if __name__ == '__main__':
    main()
