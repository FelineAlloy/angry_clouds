import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import colorsys

class Drone:
    def __init__(self, x, y, B, phi):
        self.x = x
        self.y = y
        self.B = B
        self.phi = phi

    def b(self, t):
        t = (t + self.phi) % 10
        if t < 2 or t > 7:
            return 0
        elif t == 2 or t == 7:
            return self.B / 2
        else:
            return self.B

def read_input(in_file_name):
    with open(in_file_name) as f:
        M, N, FN, T = map(int, f.readline().split())

        drones = [[None]*M for _ in range(N)]
        for _ in range(M * N):
            x, y, B, phi = f.readline().split()
            drones[int(x)][int(y)] = Drone(int(x), int(y), float(B), int(phi))

        flows = []
        for _ in range(FN):
            parts = f.readline().split()
            f_id = int(parts[0])
            x, y, t_start, s, m1, n1, m2, n2 = map(int, parts[1:])
            flows.append({
                'id': f_id,
                'x': x,
                'y': y,
                't_start': t_start,
                's': s,
                'm1': m1,
                'n1': n1,
                'm2': m2,
                'n2': n2
            })
    return M, N, T, drones, flows

def read_output(out_file_name):
    with open(out_file_name) as f:
        output = {}
        while True:
            header = f.readline()
            if not header:
                break
            fid, p = map(int, header.split())
            records = []
            for _ in range(p):
                t, x, y, z = f.readline().split()
                records.append((int(t), int(x), int(y), float(z)))
            output[fid] = records
    return output

_GOLDEN = 0.618033988749895  # 1/phi^2

def id_intensity_to_rgb(obj_id: int, intensity: int,
                        max_intensity: int = 100,
                        sat: float = 0.85,
                        v_min: float = 0.35,
                        v_max: float = 0.95):
    """
    Map (id, intensity) -> RGB in [0,1] using HSV.
    - Hue from id via golden-angle spacing.
    - Value (brightness) increases monotonically with intensity.
    """
    h = (obj_id * _GOLDEN) % 1.0
    t = 0.0 if max_intensity <= 1 else (intensity - 1) / (max_intensity - 1)
    v = v_min + (v_max - v_min) * t         # monotone brightness
    r, g, b = np.clip(colorsys.hsv_to_rgb(h, sat, v), 0, 1)
    return r, g, b

def build_tensor(output, T, M, N, maxB):
    data = np.ones((T, M, N, 3), dtype=float)
    for flow, schedules in output.items():
        for t, x, y, z in schedules:
            data[t, x, y] = id_intensity_to_rgb(flow, z)
    return data

def setup_pixel_grid(ax, ny, nx):
    """Draw a pixel grid with cell borders and ticks centered on pixels."""
    # Major ticks at each pixel center
    ax.set_xticks(np.arange(nx))
    ax.set_yticks(np.arange(ny))
    # Minor ticks at cell borders (half-integers)
    ax.set_xticks(np.arange(nx + 1) - 0.5, minor=True)
    ax.set_yticks(np.arange(ny + 1) - 0.5, minor=True)
    # Show only minor grid (cell borders)
    ax.grid(False)
    ax.grid(which='minor', color='black', linestyle='-', linewidth=0.5)
    ax.tick_params(which='minor', bottom=False, left=False)
    ax.set_aspect('equal')
    # Keep grid above image (useful if you later switch to blit=True)
    ax.set_axisbelow(False)

def init_figure(data):
    if data.ndim != 4 or data.shape[-1] != 3:
        raise ValueError("Expected data of shape (T, M, N, 3) for RGB video frames.")

    T, M, N, _ = data.shape

    fig, ax = plt.subplots()
    im = ax.imshow(
        data[0],
        interpolation='nearest',
        origin='lower'
    )

    setup_pixel_grid(ax, M, N)  # keep grid visualisation
    title = ax.set_title(f"Frame 0 / {T-1}")

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    fig.tight_layout()
    return fig, ax, im, title

def animate_tensor(data, interval_ms, use_blit=False):
    fig, ax, im, title = init_figure(data)

    if use_blit:
        # Make animated artists explicit when blitting
        im.set_animated(True)
        title.set_animated(True)

    def update(t):
        im.set_data(data[t])
        title.set_text(f"Frame {t} / {data.shape[0]-1}")
        return (im, title) if use_blit else tuple()

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=data.shape[0],
        interval=interval_ms,
        blit=use_blit
    )
    return fig, ani

def save(ani):
    html_str = ani.to_jshtml()
    with open("animation.html", "w", encoding="utf-8") as f:
        f.write(html_str)

def main():
    in_file_name = "in.txt"
    out_file_name = "out.txt"
    M, N, T, drones, flows = read_input(in_file_name)
    output = read_output(out_file_name)
    maxB = max(drone.B for row in drones for drone in row)
    data = build_tensor(output, T, M, N, maxB)


    interval_ms = 500
    fig, ani = animate_tensor(data, interval_ms, use_blit=False)
    plt.show()
    # save(ani)

if __name__ == '__main__':
    main()