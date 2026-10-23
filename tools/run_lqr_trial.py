"""One explicitly invoked two-second firmware trial via the project's paired HC-05."""
import socket,time,json,argparse
from pathlib import Path
from datetime import datetime
parser=argparse.ArgumentParser()
parser.add_argument("--soft", action="store_true", help="Two-second 70% pitch-rate feedback comparison")
args=parser.parse_args()
log=[]
s=socket.socket(socket.AF_BLUETOOTH,socket.SOCK_STREAM,socket.BTPROTO_RFCOMM)
s.settimeout(5)
s.connect(('00:26:01:02:E7:53',1));s.settimeout(.15)
def send(cmd):
    log.append({'t':time.monotonic(),'tx':cmd});s.sendall((cmd+'\r\n').encode())
def read(seconds):
    end=time.monotonic()+seconds;out=''
    while time.monotonic()<end:
        try:
            b=s.recv(4096)
            if not b: raise RuntimeError('Bluetooth disconnected')
            text=b.decode(errors='replace');out+=text
            log.append({'t':time.monotonic(),'rx':text});print(text,end='',flush=True)
        except socket.timeout: pass
    return out
try:
    send('@stream:0');send('@lqrcheck');check=read(1)
    if 'valid=1 faults=0x0' not in check or 'motor_mode=0 frame=1 wheels=1,1 seq_match=1 pose_valid=1 pose_gap=0' not in check:
        raise RuntimeError('Read-only precheck failed; no trial requested')
    send('@lqrsoft' if args.soft else '@lqrtrial');read(3)
    send('@lqrstatus');read(1)
finally:
    try:
        send('@mstop');read(.5)
    finally:
        s.close()
        path=Path(__file__).resolve().parents[1]/'docs/experiments'/('lqr_uart_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
        path.write_text(json.dumps(log,ensure_ascii=False,indent=2)+'\n');print('\nSaved:',path)
