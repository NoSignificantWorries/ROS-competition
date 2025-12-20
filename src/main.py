import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as pchs


def draw_circle_and_points(arr2d, colors, name):
    fig, ax = plt.subplots(1, 1, figsize=(6, 6))
    
    h, w = arr2d.shape[:2]
    
    yp = np.sin(arr2d[..., 0]) * arr2d[..., 1]
    xp = np.cos(arr2d[..., 0]) * arr2d[..., 1]
    
    xp_flat = xp.flatten()
    yp_flat = yp.flatten()
    
    colors_flat = colors.reshape(-1, 3) / 255.0
    
    step = max(1, (h * w) // 10000)
    
    circle = pchs.Circle((0, 0), 1, fill=False, edgecolor='black', linewidth=2, linestyle='-')
    ax.add_patch(circle)

    wedge = pchs.Wedge((0, 0), 1, -15, 15, facecolor="#ff0000", alpha=0.2, edgecolor='black', linewidth=1)
    ax.add_patch(wedge)

    ax.scatter(xp_flat[::step], yp_flat[::step], c=colors_flat[::step], edgecolor="k", alpha=0.4)

    ax.set_aspect('equal')
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_title(f"Points: {len(xp_flat[::step])}")

    plt.savefig(f"points_{name}.png", dpi=150)


def main():
    depth = np.load("etc/depth.npy")
    img_raw = cv2.imread("etc/conuses.png")
    img_rgb = cv2.cvtColor(img_raw, cv2.COLOR_BGR2RGB)
    img_hsv = cv2.cvtColor(img_raw, cv2.COLOR_BGR2HSV).astype(np.float32)
    img_hsv[..., 0] = img_hsv[..., 0] / 179 * 2 * np.pi
    img_hsv[..., 1] /= 255
    img_hsv[..., 2] /= 255

    draw_circle_and_points(img_hsv, img_rgb, "main")

    fig, ax = plt.subplots(2, 2, figsize=(24, 24))

    ax[0][0].imshow(depth)
    ax[0][1].imshow(img_rgb)
    yellow_way_mask = (img_hsv[..., 1] > 0.9) & (img_hsv[..., 0] >= np.deg2rad(55)) & (img_hsv[..., 0] <= np.deg2rad(65))
    ax[1][0].imshow(yellow_way_mask, cmap="gray")
    line_depth = depth.copy()
    line_depth[~yellow_way_mask] = 0
    line_depth[line_depth > 1.0] = 0
    print(line_depth.max())
    ax[1][1].imshow(line_depth)

    plt.tight_layout()
    plt.savefig("res.png", dpi=200)


if __name__ == "__main__":
    main()
