#!/usr/bin/env python3
"""Convert schema-3 RAM trace JSON to CSV, summary JSON and a PNG (offline)."""
import argparse
import csv
import json
import math
from pathlib import Path

EVENTS = {0: 'sample', 1: 'stop', 2: 'fault', 3: 'invalid', 4: 'manual'}
LQR_ARRAYS = {
    'error': ['x_error_m', 'yaw_error_rad', 'pitch_rad', 'v_error_mps', 'yaw_rate_radps', 'pitch_rate_radps'],
    'torque_nm': ['torque_left_nm', 'torque_right_nm'],
    'duty': ['duty_left', 'duty_right'],
    'relative_rpm': ['rpm_left', 'rpm_right'],
}
IMU_ARRAYS = {'accel_g': ['accel_x_g', 'accel_y_g', 'accel_z_g'],
              'gyro_dps': ['gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps']}


def flatten(document):
    if document.get('schema_version') != 3:
        raise ValueError('Expected schema_version=3; historical trial files use a different format.')
    source = document['status']['source_id']
    if source not in (1, 2):
        raise ValueError('Unsupported source_id')
    arrays = LQR_ARRAYS if source == 1 else IMU_ARRAYS
    scalar = ['input_timestamp_us', 'input_sequence', 'dt_us', 'age_us', 'pitch_rate_scale', 'yaw_rate_scale'] if source == 1 else []
    fields = ['time_s', 'timestamp_us', 'sequence', 'business_mode', 'faults', 'event', 'event_name', 'valid'] + scalar
    fields += [name for names in arrays.values() for name in names]
    rows, elapsed, previous = [], 0, None
    for frame in document['frames']:
        stamp = int(frame['timestamp_us'])
        if previous is not None:
            delta = (stamp - previous) & 0xffffffff
            if delta >= 0x80000000:
                raise ValueError('Timestamps are out of order or separated by more than half a uint32 cycle.')
            elapsed += delta
        previous = stamp
        row = {key: frame[key] for key in fields if key in frame}
        row.update(time_s=elapsed / 1e6, event_name=EVENTS.get(frame['event'], 'unknown'))
        if frame['valid'] and frame['event'] == 0:
            for key in scalar:
                if key == 'yaw_rate_scale' and document['status'].get('format_id') == 2:
                    row[key] = frame.get(key, 1.0)  # Format 2 predates yaw-rate scaling.
                else:
                    row[key] = frame[key]
            for key, names in arrays.items():
                values = frame[key]
                if len(values) != len(names) or not all(math.isfinite(v) for v in values):
                    raise ValueError('Invalid measurement array: ' + key)
                row.update(zip(names, values))
        else:
            # Events never reuse the previous measurement, even if supplied by an input file.
            for key in scalar + [name for names in arrays.values() for name in names]:
                row.pop(key, None)
        rows.append(row)
    return source, fields, rows


def render(source, rows, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    valid = [r for r in rows if r['valid'] and r['event'] == 0]
    if source == 1:
        panels = [(['pitch_rad'], 'Pitch (deg)', 180 / math.pi),
                  (['pitch_rate_radps'], 'Pitch rate (deg/s)', 180 / math.pi),
                  (['x_error_m', 'v_error_mps'], 'Position (m), speed (m/s)', 1),
                  (['duty_left', 'duty_right'], 'Duty (%)', 100),
                  (['torque_left_nm', 'torque_right_nm'], 'Requested torque (Nm)', 1),
                  (['rpm_left', 'rpm_right'], 'Wheel speed (rpm)', 1),
                  (['yaw_error_rad', 'yaw_rate_radps'], 'Yaw (rad), rate (rad/s)', 1),
                  (['dt_us', 'age_us'], 'Input dt / age (ms)', .001)]
    else:
        panels = [(['accel_x_g', 'accel_y_g', 'accel_z_g'], 'Acceleration (g)', 1),
                  (['gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps'], 'Angular rate (deg/s)', 1)]
    labels = {'pitch_rad': 'Pitch', 'pitch_rate_radps': 'Pitch rate',
              'dt_us': 'Input interval', 'age_us': 'Input age',
              'duty_left': 'Left', 'duty_right': 'Right'}
    fig, axes = plt.subplots(len(panels), 1, figsize=(12, 2.1 * len(panels)), sharex=True)
    for ax, (keys, label, scale) in zip(axes, panels):
        for key in keys:
            ax.plot([r['time_s'] for r in valid], [r[key] * scale for r in valid], '.-',
                    markersize=3, linewidth=1, label=labels.get(key, key))
        for row in rows:
            if row['event'] != 0:
                ax.axvline(row['time_s'], color='crimson', linestyle=':', alpha=.7)
        ax.set_ylabel(label)
        ax.grid(alpha=.25)
        ax.legend(loc='upper left', fontsize=8, ncol=3)
    axes[-1].set_xlabel('Time since oldest retained record (s)')
    events = ', '.join(r['event_name'] for r in rows if r['event'] != 0) or 'none'
    fig.suptitle(f'{"LQR" if source == 1 else "IMU"} RAM trace | {len(valid)} valid samples | events: {events}\n'
                 'Retained ring-buffer window; not the complete trial')
    fig.tight_layout(rect=(0, 0, 1, .96))
    try:
        with output.open('xb') as stream:
            fig.savefig(stream, format='png', dpi=140)
    finally:
        plt.close(fig)


def report(path, output_dir=None, plot=True):
    path = Path(path)
    document = json.loads(path.read_text())
    source, fields, rows = flatten(document)
    directory = Path(output_dir) if output_dir else path.parent
    directory.mkdir(parents=True, exist_ok=True)
    csv_path, summary_path, png_path = [directory / (path.stem + suffix) for suffix in ('.csv', '_summary.json', '.png')]
    for target in [csv_path, summary_path] + ([png_path] if plot else []):
        if target.exists():
            raise FileExistsError('Refusing to overwrite ' + str(target))
    with csv_path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    valid = [r for r in rows if r['valid'] and r['event'] == 0]
    summary = dict(source_id=source, status=document['status'], retained_records=len(rows),
                   valid_samples=len(valid), window_s=rows[-1]['time_s'] if rows else 0,
                   events=[{k: r[k] for k in ('time_s', 'event_name', 'faults')} for r in rows if r['event'] != 0])
    if valid:
        summary['sample_interval_us'] = [(int(b['timestamp_us']) - int(a['timestamp_us'])) & 0xffffffff
                                         for a, b in zip(valid, valid[1:])]
    if source == 1 and valid:
        summary['tuning'] = {key: sorted({r[key] for r in valid}) for key in ('pitch_rate_scale', 'yaw_rate_scale')}
        summary.update(pitch_deg_range=[min(r['pitch_rad'] for r in valid)*180/math.pi,
                                        max(r['pitch_rad'] for r in valid)*180/math.pi],
                       max_input_age_us=max(r['age_us'] for r in valid),
                       peak_abs_duty=max(abs(r[k]) for r in valid for k in ('duty_left', 'duty_right')))
    with summary_path.open('x') as stream:
        json.dump(summary, stream, indent=2)
    if plot:
        render(source, rows, png_path)
    return [csv_path, summary_path] + ([png_path] if plot else [])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('json_file', type=Path)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--no-plot', action='store_true', help='CSV and summary only; no matplotlib dependency')
    args = parser.parse_args()
    for output in report(args.json_file, args.output_dir, not args.no_plot):
        print(output.resolve())


if __name__ == '__main__':
    main()
