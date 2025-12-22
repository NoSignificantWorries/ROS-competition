import json
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


def get_path(data):
    paths = {
        "p1": {
            "x": [],
            "y": []
        },
        "p2": {
            "x": [],
            "y": []
        }
    }
    for name, data in data.items():
        if name.startswith("fork"):
            for subname, subdata in data["w1"].items():
                x, y = subdata
                paths["p1"]["x"].append(x)
                paths["p1"]["y"].append(y)
            for subname, subdata in data["w2"].items():
                x, y = subdata
                paths["p2"]["x"].append(x)
                paths["p2"]["y"].append(y)
        else:
            x, y = data
            paths["p1"]["x"].append(x)
            paths["p1"]["y"].append(y)

    return paths


def compile(data, map_size):
    res_points = {
        "points": {}
    }
    offset_x = data["offset"]["x"][0]
    width = data["offset"]["x"][1] - data["offset"]["x"][0]
    offset_y = data["offset"]["y"][0]
    height = data["offset"]["y"][1] - data["offset"]["y"][0]

    new_w = map_size["x"]
    new_h = map_size["y"]

    kx = (new_w[1] - new_w[0]) / (width)
    ky = (new_h[1] - new_h[0]) / (height)

    for name, points in data["points"].items():
        if name.startswith("fork"):
            res_points["points"][name] = {}
            for wname, wdata in points.items():
                res_points["points"][name][wname] = {}
                for subname, subpoints in wdata.items():
                    x, y = subpoints
                    xnew = new_w[0] + (x - offset_x) * kx
                    ynew = new_h[0] + (y - offset_y) * ky
                    res_points["points"][name][wname][subname] = (xnew, ynew)
        else:
            x, y = points
            xnew = new_w[0] + (x - offset_x) * kx
            ynew = new_h[0] + (y - offset_y) * ky
            res_points["points"][name] = (xnew, ynew)

    print(res_points["points"]["start"])

    return res_points


def main():
    img = cv2.imread("trace.jpg")
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    map_size = {
        "x": [2.36, -2.34],
        "y": [-2.37, 2.48]
    }

    offset = {
        "x": [12, img.shape[1] - 14],
        "y": [12, img.shape[0]]
    }
    points_conf = {
        "start": (141, 30),
        "l0": (60, 30),
        "r1": (35, 55),
        "l1": (35, 130),
        "r2": (65, 155),
        "l2": (80, 155),
        "fork": {
            "w1": {
                "r3": (95, 120),
                "r4": (130, 92),
                "r5": (160, 120),
                "r6": (176, 155),
            },
            "w2": {
                "r3": (95, 190),
                "r4": (130, 218),
                "r5": (160, 190),
                "r6": (176, 155),
            }
        },
        "l3": (194, 155),
    }

    compiled_points = compile({"points": points_conf, "offset": offset}, map_size)

    with open("path.json", "w") as file:
        json.dump(compiled_points, file, indent=2)


    fig, ax = plt.subplots(1, 1, figsize=(16, 16))

    ax.imshow(img_rgb)
    ax.axvline(offset["x"][0], color="b", linestyle="--")
    ax.axvline(offset["x"][1], color="b", linestyle="--")
    ax.axhline(offset["y"][0], color="b", linestyle="--")
    ax.axhline(offset["y"][1], color="b", linestyle="--")

    paths = get_path(points_conf)
    ax.plot(paths["p1"]["x"], paths["p1"]["y"], "r-o")
    ax.plot(paths["p2"]["x"], paths["p2"]["y"], "b-o")

    loc = ticker.MultipleLocator(base=10)
    ax.xaxis.set_major_locator(loc)
    ax.yaxis.set_major_locator(loc)
    ax.grid(which='major', axis='both', linestyle='-', alpha=0.5)

    plt.tight_layout()
    plt.savefig("res.png", dpi=200)


    fig, ax = plt.subplots(1, 1, figsize=(16, 16))

    paths = get_path(compiled_points["points"])
    ax.plot(paths["p1"]["x"], paths["p1"]["y"], "r-o")
    ax.plot(paths["p2"]["x"], paths["p2"]["y"], "b-o")

    ax.set_xlim(*map_size["x"])
    ax.set_ylim(*map_size["y"])
    ax.invert_yaxis()

    plt.tight_layout()
    plt.savefig("final.png", dpi=200)


if __name__ == "__main__":
    main()
