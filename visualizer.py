import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

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

def build_tensor(output, T, M, N):
    data = np.zeros((T, M, N), dtype=float)
    for schedules in output.values():
        for t, x, y, z in schedules:
            data[t, x, y] = z
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

def init_figure(data, vmin, vmax):
    fig, ax = plt.subplots()
    im = ax.imshow(
        data,
        vmin=vmin, vmax=vmax,
        cmap='coolwarm',
        interpolation='nearest',
        origin='lower'
    )
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label("z", rotation=0, labelpad=10)
    ny, nx = data.shape
    setup_pixel_grid(ax, ny, nx)
    title = ax.set_title("Frame 0")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    fig.tight_layout()
    return fig, ax, im, title

def animate_tensor(data, interval_ms=250, use_blit=False):
    vmin, vmax = data.min(), data.max()
    fig, ax, im, title = init_figure(data[0], vmin, vmax)

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
    data = build_tensor(output, T, M, N)


    interval_ms = 250
    fig, ani = animate_tensor(data, interval_ms=interval_ms, use_blit=False)
    plt.show()
    # save(ani)

if __name__ == '__main__':
    main()