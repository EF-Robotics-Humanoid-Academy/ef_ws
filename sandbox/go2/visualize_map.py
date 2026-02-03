import os
import sys
import matplotlib.pyplot as plt


def _read_pgm(path):
    with open(path, "rb") as handle:
        magic = handle.readline().strip()
        if magic != b"P5":
            raise ValueError(f"unsupported PGM format: {magic!r}")
        line = handle.readline()
        while line.startswith(b"#"):
            line = handle.readline()
        width_height = line.split()
        if len(width_height) != 2:
            raise ValueError("invalid PGM header")
        width, height = [int(v) for v in width_height]
        maxval = int(handle.readline().strip())
        if maxval <= 0 or maxval > 65535:
            raise ValueError(f"unsupported maxval: {maxval}")
        data = handle.read()
        expected = width * height
        if maxval < 256:
            if len(data) < expected:
                raise ValueError("PGM data is truncated")
            pixels = list(data[:expected])
        else:
            if len(data) < expected * 2:
                raise ValueError("PGM data is truncated")
            pixels = [data[i] * 256 + data[i + 1] for i in range(0, expected * 2, 2)]
        return width, height, pixels


def _load_yaml_image(path):
    image_name = None
    with open(path, "r", encoding="ascii") as handle:
        for line in handle:
            if line.strip().startswith("image:"):
                image_name = line.split(":", 1)[1].strip()
                break
    if not image_name:
        raise ValueError("no image entry in yaml")
    if not os.path.isabs(image_name):
        image_name = os.path.join(os.path.dirname(path), image_name)
    return image_name


def _list_maps(map_dir):
    maps = []
    for name in sorted(os.listdir(map_dir)):
        if name.endswith(".yaml"):
            maps.append(os.path.join(map_dir, name))
    return maps


def _select_map(map_dir):
    maps = _list_maps(map_dir)
    if not maps:
        raise SystemExit(f"no maps found in {map_dir}")
    print("Available maps:")
    for idx, path in enumerate(maps, start=1):
        print(f"{idx:2d}) {os.path.basename(path)}")
    while True:
        choice = input("Select map number: ").strip()
        if not choice:
            continue
        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(maps):
                return maps[idx - 1]
        print("Invalid selection.")


def main():
    map_dir = os.path.join(os.getcwd(), "maps")
    yaml_path = _select_map(map_dir)
    pgm_path = _load_yaml_image(yaml_path)
    width, height, pixels = _read_pgm(pgm_path)
    grid = [pixels[i * width:(i + 1) * width] for i in range(height)]
    plt.figure(figsize=(7, 6))
    plt.imshow(grid, cmap="inferno", origin="lower")
    plt.colorbar(label="Occupancy")
    plt.title(os.path.basename(yaml_path))
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
