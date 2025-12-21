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
    img_raw = cv2.imread("etc/arrow.png")
    img_raw = img_raw[:, :-100, :]
    img_rgb = cv2.cvtColor(img_raw, cv2.COLOR_BGR2RGB)
    img_hsv = cv2.cvtColor(img_raw, cv2.COLOR_BGR2HSV).astype(np.float32)
    img_hsv[..., 0] = img_hsv[..., 0] / 179 * 2 * np.pi
    img_hsv[..., 1] /= 255
    img_hsv[..., 2] /= 255

    draw_circle_and_points(img_hsv, img_rgb, "arrow")

    fig, ax = plt.subplots(1, 3, figsize=(24, 12))

    # ax[0][0].imshow(depth)
    ax[0].imshow(img_rgb)
    yellow_way_mask = (img_hsv[..., 1] > 0.9) & (img_hsv[..., 0] >= np.deg2rad(55)) & (img_hsv[..., 0] <= np.deg2rad(65))
    blue_mask = (img_hsv[..., 1] > 0.9) & (img_hsv[..., 0] >= np.deg2rad(200)) & (img_hsv[..., 0] <= np.deg2rad(260))

    ax[1].imshow(blue_mask, cmap="gray")
    if np.any(blue_mask):
        y_coords, x_coords = np.where(blue_mask)
        bbox = (x_coords.min(), y_coords.min(), 
                x_coords.max() - x_coords.min(), 
                y_coords.max() - y_coords.min())
        print(bbox)

        sign = blue_mask[bbox[1]:bbox[1] + bbox[3] + 1,
                           bbox[0]:bbox[0] + bbox[2] + 1].astype(np.uint8)

        resized_sign = cv2.resize(sign, (100, 100), interpolation=cv2.INTER_NEAREST)
        resized_sign = resized_sign[:, ::-1]

        mask_ref = cv2.imread("mask.png", cv2.IMREAD_GRAYSCALE)
        _, soft_mask_ref = cv2.threshold(mask_ref, 127, 255, cv2.THRESH_BINARY)

        intersection = np.logical_and(soft_mask_ref > 0, resized_sign > 0).sum()
        union = np.logical_or(soft_mask_ref > 0, resized_sign > 0).sum()
        
        iou = intersection / (union + 1e-10)
        
        print(iou)

        kernel_size = (5, 5)
        sigma = 1.0

        blurred = cv2.GaussianBlur(resized_sign.astype(np.float32), kernel_size, sigma)

        # cv2.imwrite("mask.png", blurred)

        rect = pchs.Rectangle((bbox[0], bbox[1]), bbox[2], bbox[3], linewidth=2, edgecolor='r', facecolor='none')
        ax[1].add_patch(rect)

        ax[2].imshow(blurred, cmap="gray")

    # line_depth = depth.copy()
    # line_depth[~yellow_way_mask] = 0
    # line_depth[line_depth > 1.0] = 0
    # print(line_depth.max())
    # ax[1][1].imshow(line_depth)

    plt.tight_layout()
    plt.savefig("res.png", dpi=200)


if __name__ == "__main__":
    main()
