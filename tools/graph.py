import sys
import re
import time
import subprocess
import threading
import collections
import argparse
import os

import matplotlib.pyplot as plt
import matplotlib.animation as animation

def main():
    parser = argparse.ArgumentParser(description='Live angle plot from decode_ws22')
    parser.add_argument('port', nargs='?', default='/dev/ttyACM0')
    parser.add_argument('--window', type=int, default=1000)
    args = parser.parse_args()

    decoder = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'decode_ws22')
    if not os.path.exists(decoder):
        print(f'error: decoder not found at {decoder}', file=sys.stderr)
        sys.exit(1)

    angles = collections.deque(maxlen=args.window)
    temps = collections.deque(maxlen=1000)
    lock = threading.Lock()
    angle_pat = re.compile(r'\s(\d{1,3}\.\d{2})\s+deg')
    temp_pat = re.compile(r'temp=([\d.\-]+)°C')

    def reader():
        try:
            proc = subprocess.Popen(
                [decoder, args.port],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
        except OSError as e:
            print(f'failed to start decoder: {e}', file=sys.stderr)
            return
        for line in proc.stdout:
            ma = angle_pat.search(line)
            mt = temp_pat.search(line)
            if ma:
                now = time.monotonic()
                with lock:
                    angles.append((float(ma.group(1)), now))
            if mt:
                with lock:
                    temps.append(float(mt.group(1)))

    threading.Thread(target=reader, daemon=True).start()

    fig, ax = plt.subplots(figsize=(10, 4))
    (line,) = ax.plot([], [], lw=0.7, color='steelblue')

    ax.set_ylim(-10, 370)
    ax.set_yticks([0, 90, 180, 270, 360])
    ax.set_ylabel('angle (deg)')
    ax.set_xlabel('samples')
    ax.set_title(f'Motor Position  —  {args.port}')
    ax.grid(axis='y', linestyle='--', alpha=0.4)

    rpm_text = ax.text(0.98, 0.95, '', transform=ax.transAxes,
                       ha='right', va='top', fontsize=12,
                       fontweight='bold', color='darkred',
                       bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.85))
    temp_text = ax.text(0.98, 0.82, '', transform=ax.transAxes,
                        ha='right', va='top', fontsize=11,
                        color='darkgreen',
                        bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.85))

    def compute_rpm(data):
        if len(data) < 2:
            return 0.0
        span = min(200, len(data))
        a0, t0 = data[-span]
        a1, t1 = data[-1]
        dt = t1 - t0
        if dt < 1e-6:
            return 0.0
        da = a1 - a0
        while da > 180:
            da -= 360
        while da < -180:
            da += 360
        deg_per_sec = da / dt
        return deg_per_sec / 6.0

    def update(_):
        with lock:
            data = list(angles)
            t_list = list(temps)
        n = len(data)
        if n:
            angle_vals = [d[0] for d in data]
            line.set_data(range(n), angle_vals)
            ax.set_xlim(0, max(n, 10))
            rpm = compute_rpm(data)
            rpm_text.set_text(f'{rpm:.0f} RPM')
        if t_list:
            avg_temp = sum(t_list) / len(t_list)
            temp_text.set_text(f'{avg_temp:.1f} °C')
        return (line, rpm_text, temp_text)

    ani = animation.FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
