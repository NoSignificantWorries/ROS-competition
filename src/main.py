import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def draw_circle_and_points(arr2d, colors, name):
        fig, ax = plt.subplots(1, 1, figsize=(6, 6))
        
        # Получаем форму массива
        h, w = arr2d.shape[:2]
        
        # Преобразуем в полярные координаты
        yp = np.sin(arr2d[..., 0]) * arr2d[..., 1]
        xp = np.cos(arr2d[..., 0]) * arr2d[..., 1]
        
        # Преобразуем массивы в 1D для scatter
        xp_flat = xp.flatten()
        yp_flat = yp.flatten()
        
        # Преобразуем цвета в правильный формат
        # colors должен быть массивом формы (n_points, 3) или (n_points, 4)
        colors_flat = colors.reshape(-1, 3) / 255.0  # нормализуем RGB от 0 до 1
        
        # Ограничиваем количество точек для производительности (если нужно)
        # Можно взять каждый N-й пиксель
        step = max(1, (h * w) // 10000)  # примерно 10000 точек максимум
        
        ax.scatter(xp_flat[::step], yp_flat[::step], c=colors_flat[::step], edgecolor="k", alpha=0.3)  # уменьшаем размер точек

        circle = plt.Circle((0, 0), 1, fill=False, edgecolor='black', linewidth=2, linestyle='-')

        ax.add_patch(circle)
        ax.set_aspect('equal')
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_title(f"Points: {len(xp_flat[::step])}")

        plt.savefig(f"points_{name}.png", dpi=150)


def main():
    img_g_raw = cv2.imread("etc/green_light.png")
    img_g_rgb = cv2.cvtColor(img_g_raw, cv2.COLOR_BGR2RGB)
    img_g_hsv = cv2.cvtColor(img_g_raw, cv2.COLOR_BGR2HSV).astype(np.float32)
    img_g_hsv[..., 0] = cv2.normalize(img_g_hsv[..., 0], None, 0, np.pi * 2, cv2.NORM_MINMAX)
    img_g_hsv[..., 1] = cv2.normalize(img_g_hsv[..., 1], None, 0, 1, cv2.NORM_MINMAX)
    img_g_hsv[..., 2] = cv2.normalize(img_g_hsv[..., 2], None, 0, 1, cv2.NORM_MINMAX)

    draw_circle_and_points(img_g_hsv, img_g_rgb, "green")

    fig, ax = plt.subplots(1, 1, figsize=(10, 10))

    mask = (img_g_hsv[..., 0] >= (np.pi / 2)) & (img_g_hsv[..., 0] <= (2 * np.pi / 3))
    img_g_rgb[mask] = (0, 0, 0)
    ax.imshow(img_g_rgb)

    plt.tight_layout()
    plt.savefig("green.png", dpi=200)

    # ===========================================================================================

    depth = np.load("etc/depth.npy")
    img_raw = cv2.imread("etc/conuses.png")
    img_rgb = cv2.cvtColor(img_raw, cv2.COLOR_BGR2RGB)
    img_hsv = cv2.cvtColor(img_raw, cv2.COLOR_BGR2HSV).astype(np.float32)
    img_hsv[..., 0] = cv2.normalize(img_hsv[..., 0], None, 0, np.pi * 2, cv2.NORM_MINMAX)
    img_hsv[..., 1] = cv2.normalize(img_hsv[..., 1], None, 0, 1, cv2.NORM_MINMAX)
    img_hsv[..., 2] = cv2.normalize(img_hsv[..., 2], None, 0, 1, cv2.NORM_MINMAX)

    draw_circle_and_points(img_hsv, img_rgb, "main")

    fig, ax = plt.subplots(1, 2, figsize=(24, 10))

    ax[0].imshow(depth)
    ax[1].imshow(img_rgb)

    plt.tight_layout()
    plt.savefig("res.png", dpi=200)


    # ===========================================================================================

    fig, ax = plt.subplots(2, 3, figsize=(33, 15))

    ax[0][0].imshow(img_hsv[..., 0], cmap="hsv")
    ax[0][0].set_title(f"Hue: {img_hsv[..., 0].mean()}")
    ax[0][1].imshow(img_hsv[..., 1], cmap="gray")
    ax[0][0].set_title(f"Saturation: {img_hsv[..., 1].mean()}")
    ax[0][2].imshow(img_hsv[..., 2], cmap="gray")
    ax[0][0].set_title(f"Value: {img_hsv[..., 2].mean()}")

    ax[1][0].hist(img_hsv[..., 0].flatten(), bins=100, color="g", alpha=0.5, edgecolor="black")
    ax[1][1].hist(img_hsv[..., 1].flatten(), bins=100, color="g", alpha=0.5, edgecolor="black")
    ax[1][2].hist(img_hsv[..., 2].flatten(), bins=100, color="g", alpha=0.5, edgecolor="black")

    plt.tight_layout()
    plt.savefig("chroma.png", dpi=200)


if __name__ == "__main__":
    main()
