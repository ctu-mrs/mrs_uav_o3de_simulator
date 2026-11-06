#!/usr/bin/env python3
"""Bake a sky-only HDRI, for levels whose ground is not part of the skybox.

    python3 Tools/bake_sky.py [path to the O3DE engine]

An HDRI is a 360 degree photo, so its lower half and horizon show where it was taken, and they
appear around the edge of a splat. The mountains in this one reach 17 degrees and hide clouds, so
they are warped away instead of painted over: the ridge line becomes the horizon, and each column
of sky is stretched down to it. Below the horizon is haze in the colors of the sky just above it.

Writes Assets/Sky/berlin_sky_skyboxcm.exr, which the _skyboxcm suffix imports as a skybox cubemap
without IBL outputs. Needs numpy, scipy, and OpenCV.
"""

import os
import sys
from pathlib import Path

os.environ["OPENCV_IO_ENABLE_OPENEXR"] = "1"  # Off by default, and read when cv2 is imported.

import cv2
import numpy as np
from scipy import ndimage

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_ENGINE = "/opt/O3DE/26.05"
SOURCE = "Gems/Atom/Feature/Common/Assets/LightingPresets/LowContrast/champagne_castle_1_4k_iblskyboxcm.exr"
OUTPUT = PROJECT_ROOT / "Assets" / "Sky" / "berlin_sky_skyboxcm.exr"

GROUND_LUMINANCE = 0.2  # Darker than the darkest clouds; 0.3 already eats into them.
GROUND_GREEN_LUMINANCE = 0.6  # Sunlit grass is brighter, but greener than the sky.
SKY_ABOVE = 25.0  # [deg] Nothing above this elevation is ground.
MIN_RIDGE = 3.0  # [deg] Also removes the distant hills in the valley, too faint for the ground test.
RIDGE_MARGIN = 1.0  # [deg] Keeps the bright rim of the ridge out of the sky.
RIDGE_ENVELOPE = 3.0  # [deg] Width of the peaks that stay under the new horizon.
RIDGE_SMOOTHING = 4.0  # [deg] Keeps the warp from showing in the clouds.

HORIZON_SAMPLE = (3.0, 8.0)  # [deg] Band of warped sky that gives the haze its color.
HAZE_SMOOTHING = 45.0  # [deg] Wide, or the clouds above show up as vertical bands.
HAZE_BLEND = (-0.5, 4.0)  # [deg] Elevations between which the sky fades into the haze.
HAZE_NADIR_GAIN = 0.55  # Haze brightness straight down, as it is relative to the horizon.
HAZE_FADE = 30.0  # [deg] Depth at which the haze reaches that brightness.


def elevations(height):
    return 90.0 - (np.arange(height) + 0.5) * 180.0 / height


def read_rgb(path):
    return cv2.imread(str(path), cv2.IMREAD_UNCHANGED)[:, :, 2::-1].astype(np.float32)


def write_rgb(image, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), image[:, :, ::-1], [cv2.IMWRITE_EXR_TYPE, cv2.IMWRITE_EXR_TYPE_HALF])


def find_ridge(image):
    """The new horizon as a smooth elevation per column [deg], above everything that is ground."""
    height, width, _ = image.shape
    red, green, blue = image[:, :, 0], image[:, :, 1], image[:, :, 2]
    luminance = (red + green + blue) / 3.0

    ground = (luminance < GROUND_LUMINANCE) | ((green >= 0.98 * blue) & (luminance < GROUND_GREEN_LUMINANCE))
    ground[elevations(height) > SKY_ABOVE] = False
    ground = ndimage.binary_opening(ground, iterations=2)

    # Clouds near the ridge can pass the test as well, but do not connect to the bottom of the image.
    labels, _ = ndimage.label(ground)
    connected = np.unique(labels[height // 2 + 40])
    ground = np.isin(labels, connected[connected > 0])

    pixels_per_degree = width / 360.0
    ridge = elevations(height)[np.argmax(ground, axis=0)]
    ridge = np.maximum(ridge, MIN_RIDGE)
    ridge = ndimage.maximum_filter1d(ridge, int(RIDGE_ENVELOPE * pixels_per_degree), mode="wrap")
    ridge = ndimage.gaussian_filter1d(ridge, RIDGE_SMOOTHING * pixels_per_degree, mode="wrap")
    return ridge + RIDGE_MARGIN


def warp_sky(image, ridge):
    """Moves each column's ridge to elevation 0, and leaves the zenith in place."""
    height, width, _ = image.shape
    output_elevation = np.clip(elevations(height), 0.0, 90.0)[:, None]
    source_elevation = ridge[None, :] + output_elevation * (90.0 - ridge[None, :]) / 90.0
    map_y = ((90.0 - source_elevation) * height / 180.0 - 0.5).astype(np.float32)
    map_x = np.tile(np.arange(width, dtype=np.float32), (height, 1))
    return cv2.remap(image, map_x, map_y, cv2.INTER_CUBIC, borderMode=cv2.BORDER_WRAP).clip(0.0)


def add_haze(sky):
    height, width, _ = sky.shape
    elevation = elevations(height)

    band = (elevation > HORIZON_SAMPLE[0]) & (elevation < HORIZON_SAMPLE[1])
    color = sky[band].mean(axis=0)
    color = ndimage.gaussian_filter1d(color, HAZE_SMOOTHING * width / 360.0, axis=0, mode="wrap")

    fade = np.clip(-elevation / HAZE_FADE, 0.0, 1.0)
    gain = 1.0 + (HAZE_NADIR_GAIN - 1.0) * fade * fade * (3.0 - 2.0 * fade)
    haze = gain[:, None, None] * color[None, :, :]

    blend = np.clip((elevation - HAZE_BLEND[0]) / (HAZE_BLEND[1] - HAZE_BLEND[0]), 0.0, 1.0)
    blend = (blend * blend * (3.0 - 2.0 * blend))[:, None, None]
    return blend * sky + (1.0 - blend) * haze


def main():
    engine = Path(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_ENGINE)
    image = read_rgb(engine / SOURCE)
    ridge = find_ridge(image)
    print(f"horizon moved up by {ridge.min():.1f} to {ridge.max():.1f} deg")
    write_rgb(add_haze(warp_sky(image, ridge)).astype(np.float32), OUTPUT)
    print(f"wrote {OUTPUT}")


if __name__ == "__main__":
    main()
