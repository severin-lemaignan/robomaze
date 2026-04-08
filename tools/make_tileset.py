#!/usr/bin/env python3
"""Combine individual tile images into a single tileset.png.

Run from the repository root:
    python3 tools/make_tileset.py

Reads individual tiles from res/ and produces res/tileset.png.
The grid layout must match TILESET_POSITIONS and TILESET_FLOOR_POSITIONS
defined in robomaze/maze.py.
"""

import sys
import os

import cv2
import numpy as np

# Grid layout — must stay in sync with robomaze/maze.py
TILE_SIZE = 128
COLS = 5
ROWS = 5

# (row, col) -> filename mapping
# Wall/edge tiles (matches TILESET_POSITIONS in maze.py)
GRID = {
    (0, 0): "top.jpg",
    (0, 1): "bottom.jpg",
    (0, 2): "left.jpg",
    (0, 3): "right.jpg",
    (0, 4): "top_left_cvx.jpg",
    (1, 0): "top_left_ccv.jpg",
    (1, 1): "top_right_cvx.jpg",
    (1, 2): "top_right_ccv.jpg",
    (1, 3): "bottom_left_cvx.jpg",
    (1, 4): "bottom_left_ccv.jpg",
    (2, 0): "bottom_right_cvx.jpg",
    (2, 1): "bottom_right_ccv.jpg",
    (2, 2): "bottom_top_right.jpg",
    (2, 3): "bottom_top_left.jpg",
    (2, 4): "bottom_left_right.jpg",
    (3, 0): "top_left_right.jpg",
    (3, 1): "bottom_top.jpg",
    (3, 2): "left_right.jpg",
    # Floor variants (matches TILESET_FLOOR_POSITIONS in maze.py)
    (3, 3): "floor.jpg",
    (3, 4): "floor2.jpg",
    (4, 0): "floor3.jpg",
}


def main():
    res_dir = os.path.join(os.path.dirname(__file__), '..', 'res')
    res_dir = os.path.abspath(res_dir)

    sheet = np.zeros((ROWS * TILE_SIZE, COLS * TILE_SIZE, 4), dtype=np.uint8)

    for (row, col), filename in GRID.items():
        path = os.path.join(res_dir, filename)
        tile = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if tile is None:
            print(f"ERROR: could not load {path}", file=sys.stderr)
            sys.exit(1)
        # Add alpha channel if missing (opaque)
        if tile.shape[2] == 3:
            alpha = np.full((*tile.shape[:2], 1), 255, dtype=np.uint8)
            tile = np.concatenate([tile, alpha], axis=2)
        tile = cv2.resize(tile, (TILE_SIZE, TILE_SIZE))
        y = row * TILE_SIZE
        x = col * TILE_SIZE
        sheet[y:y + TILE_SIZE, x:x + TILE_SIZE] = tile

    out_path = os.path.join(res_dir, "tileset.png")
    cv2.imwrite(out_path, sheet)
    print(f"Written {out_path} ({sheet.shape[1]}x{sheet.shape[0]} pixels)")


if __name__ == '__main__':
    main()
