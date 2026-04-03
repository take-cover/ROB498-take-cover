from pathlib import Path
import cv2

def create_mp4_from_jpgs(output_name="output.mp4", fps=5):
    cwd = Path.cwd()
    jpgs = sorted(cwd.glob("*.jpg"), key=lambda p: p.name)

    if not jpgs:
        raise FileNotFoundError("No .jpg files found in the current working directory.")

    first = cv2.imread(str(jpgs[0]))
    if first is None:
        raise ValueError(f"Could not read image: {jpgs[0]}")

    height, width = first.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(cwd / output_name), fourcc, fps, (width, height))

    if not writer.isOpened():
        raise RuntimeError("Could not open video writer.")

    for img_path in jpgs:
        img = cv2.imread(str(img_path))
        if img is None:
            raise ValueError(f"Could not read image: {img_path}")

        if img.shape[:2] != (height, width):
            img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

        writer.write(img)

    writer.release()
    print(f"Created: {cwd / output_name}")

if __name__ == "__main__":
    create_mp4_from_jpgs()