import cv2
import numpy as np
from pathlib import Path

default_input = Path(__file__).resolve().parent.parent / "map"
src = cv2.imread(str(default_input / "map_nav_origin.pgm"), cv2.IMREAD_GRAYSCALE)

kernal = np.ones((3, 3), dtype=np.uint8)

# morphological opening to remove small noise
opened = cv2.morphologyEx(src, cv2.MORPH_OPEN, kernal)

# morphological closing to fill small holes
closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernal)

cv2.imwrite(str(default_input / "map_nav_open.pgm"), opened)
cv2.imwrite(str(default_input / "map_nav_close.pgm"), closed)