import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patheffects as patheffects
import colorsys
from file_utils import read_input, read_output

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
    r, g, b = colorsys.hsv_to_rgb(h, sat, v)
    r, g, b = np.clip([r, g, b], 0, 1)
    return r, g, b

def build_tensor(output, T, M, N):
    raw_data = np.zeros((T, M, N), dtype=float)
    data = np.ones((T, M, N, 3), dtype=float)
    for flow, schedules in output.items():
        for t, x, y, z in schedules:
            raw_data[t, x, y] = z
            data[t, x, y] = id_intensity_to_rgb(flow, z)
    return raw_data, data

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

def _make_text_overlay(ax, M, N, fmt="{:.0f}", fontsize=10):
    texts = []
    for i in range(M):
        row = []
        for j in range(N):
            txt = ax.text(
                j, i, "", ha="center", va="center", zorder=3,
                fontsize=fontsize, clip_on=True,
                path_effects=[patheffects.withStroke(linewidth=2, foreground="white")]
            )
            row.append(txt)
        texts.append(row)
    return texts, fmt

def init_figure(data, numbers=None, fmt="{:.0f}"):
    if data.ndim != 4 or data.shape[-1] != 3:
        raise ValueError("Expected data of shape (T, M, N, 3) for RGB video frames.")

    T, M, N, _ = data.shape

    if numbers is not None:
        if numbers.shape != (T, M, N):
            raise ValueError(f"`numbers` must have shape (T, M, N), got {numbers.shape}")

    fig, ax = plt.subplots()
    im = ax.imshow(
        data[0],
        interpolation='nearest',
        origin='lower'
    )
    setup_pixel_grid(ax, M, N)
    title = ax.set_title(f"Frame 0 / {T-1}")
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    texts = None
    if numbers is not None:
        texts, fmt = _make_text_overlay(ax, M, N, fmt=fmt)

        # initialize frame-0 labels
        nums0 = numbers[0]
        for i in range(M):
            for j in range(N):
                texts[i][j].set_text(fmt.format(nums0[i, j]))

    fig.tight_layout()
    return fig, ax, im, title, texts

def animate_tensor(data, interval_ms, use_blit=False, numbers=None, fmt="{:.0f}"):
    fig, ax, im, title, texts = init_figure(data, numbers=numbers, fmt=fmt)

    animated_artists = [im, title]
    if texts is not None:
        # mark all text artists animated when blitting
        for row in texts:
            for t in row:
                animated_artists.append(t)

    if use_blit:
        for a in animated_artists:
            a.set_animated(True)

    def update(t):
        im.set_data(data[t])
        title.set_text(f"Frame {t} / {data.shape[0]-1}")

        if numbers is not None:
            nums = numbers[t]
            M, N = nums.shape
            for i in range(M):
                for j in range(N):
                    texts[i][j].set_text(fmt.format(nums[i, j]))

        return tuple(animated_artists) if use_blit else tuple()

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
    in_file_name = "in.in"
    out_file_name = "out.out"
    M, N, T, drones, flows = read_input(in_file_name)
    output = read_output(out_file_name)
    raw_data, data = build_tensor(output, T, M, N)
    print(raw_data.shape, data.shape)
    interval_ms = 500
    fig, ani = animate_tensor(data, interval_ms=interval_ms, use_blit=False, numbers=raw_data, fmt="{:.1f}")
    plt.show()
    # save(ani)

if __name__ == '__main__':
    main()