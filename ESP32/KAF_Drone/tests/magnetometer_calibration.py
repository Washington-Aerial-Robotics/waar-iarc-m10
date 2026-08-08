"""
Magnetometer hard-iron / soft-iron calibration fit.

Fits a general ellipsoid to a cloud of raw magnetometer samples (collected by
rotating the sensor through many orientations) and computes the correction
A (3x3 matrix) and b (3x1 offset) such that:

    calibrated = A @ (raw - b)

maps the distorted ellipsoid back onto a sphere centered at the origin -
correcting hard-iron offset (b) and soft-iron distortion (A). This is the
same calibration model already implemented in periph_mpu9250.cpp, and the
same algorithm the "Magneto" tool uses internally.

Input format: a text file containing lines copied straight from the Serial
Monitor, e.g.:
    [P] AK8963 Magnetometer: Value=[ 123.000, -45.000, 678.000 ]
Any non-matching lines (other DPRINTF output, boot messages, etc.) are
ignored, so you can paste the raw serial log directly without cleaning it up.

Usage:
    python magnetometer_calibration.py raw_mag_log.txt
"""

import re
import sys

import numpy as np

try:
    import matplotlib.pyplot as plt
    HAVE_PLOTTING = True
except ImportError:
    HAVE_PLOTTING = False

LINE_PATTERN = re.compile(
    r"AK8963 Magnetometer: Value=\[\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\s*\]"
)


def load_samples(path):
    samples = []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            match = LINE_PATTERN.search(line)
            if match:
                samples.append([float(match.group(i)) for i in (1, 2, 3)])
    return np.array(samples)


def fit_ellipsoid(points):
    """Fit p^T M p + 2 n^T p - 1 = 0 to points via least squares."""
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    design = np.column_stack([
        x * x, y * y, z * z,
        2 * x * y, 2 * x * z, 2 * y * z,
        2 * x, 2 * y, 2 * z,
    ])
    coeffs, *_ = np.linalg.lstsq(design, np.ones_like(x), rcond=None)
    a, b_, c, d, e, f, g, h, i = coeffs

    m = np.array([
        [a, d, e],
        [d, b_, f],
        [e, f, c],
    ])
    n = np.array([g, h, i])
    return m, n


def calibration_from_ellipsoid(m, n):
    center = -np.linalg.solve(m, n)  # hard-iron offset
    k = 1.0 - n @ center
    if k <= 0:
        raise ValueError(
            "Degenerate ellipsoid fit (k <= 0) - check the input data covers "
            "a wide range of orientations, not just a small cluster."
        )
    eigvals, eigvecs = np.linalg.eigh(m / k)
    if np.any(eigvals <= 0):
        raise ValueError(
            "Non-positive-definite fit - the sample orientations likely "
            "don't span enough of the sphere for a reliable fit."
        )
    # Symmetric PD square root of M/k, NOT sqrt(eigvals) @ eigvecs.T - the latter
    # is a valid ellipsoid->sphere map too, but picks an arbitrary rotation of the
    # output frame (whatever orientation eigh happens to return). The symmetric
    # square root is the unique solution that reduces to identity when there is
    # no real distortion, so it corrects distortion without silently rotating
    # the board's axis convention (which would otherwise corrupt yaw).
    soft_iron = eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T
    return soft_iron, center


def apply_calibration(points, soft_iron, center):
    return (points - center) @ soft_iron.T


def format_cpp(soft_iron, center):
    rows = ",\n      ".join(
        "{ " + ", ".join(f"{v: .6f}" for v in row) + " }" for row in soft_iron
    )
    offs = ",\n      ".join(f"{v: .6f}" for v in center)
    return (
        "    double A[3][3] = {\n"
        f"      {rows}\n"
        "    };\n"
        "    double b[3] = {\n"
        f"      {offs}\n"
        "    };"
    )


def main():
    if len(sys.argv) != 2:
        print("Usage: python magnetometer_calibration.py raw_mag_log.txt")
        sys.exit(1)

    points = load_samples(sys.argv[1])
    print(f"Loaded {len(points)} magnetometer samples.")
    if len(points) < 100:
        print(
            "Warning: fewer than 100 samples - collect more data while "
            "rotating through a wider range of orientations for a reliable fit."
        )

    m, n = fit_ellipsoid(points)
    soft_iron, center = calibration_from_ellipsoid(m, n)

    raw_mag = np.linalg.norm(points, axis=1)
    calibrated = apply_calibration(points, soft_iron, center)
    cal_mag = np.linalg.norm(calibrated, axis=1)

    print(f"\nRaw magnitude:        mean={raw_mag.mean():.2f}  std={raw_mag.std():.2f}")
    print(f"Calibrated magnitude: mean={cal_mag.mean():.2f}  std={cal_mag.std():.2f}")
    print(
        "\n(A well-calibrated sensor should show calibrated std much smaller "
        "than raw std, close to 0 relative to mean=1 - since the target is a "
        "unit sphere - regardless of how the sensor was rotated.)"
    )

    print("\nPaste this into periph_mpu9250.cpp, replacing the magCal struct body:\n")
    print(format_cpp(soft_iron, center))

    if HAVE_PLOTTING:
        fig = plt.figure(figsize=(10, 5))
        ax1 = fig.add_subplot(121, projection="3d")
        ax1.scatter(points[:, 0], points[:, 1], points[:, 2], s=2)
        ax1.set_title("Raw (should look like a tilted/offset ellipsoid)")
        ax2 = fig.add_subplot(122, projection="3d")
        ax2.scatter(calibrated[:, 0], calibrated[:, 1], calibrated[:, 2], s=2)
        ax2.set_title("Calibrated (should look like a sphere centered at origin)")
        plt.tight_layout()
        plt.show()
    else:
        print("\n(matplotlib not installed - skipping the verification plot.)")


if __name__ == "__main__":
    main()
