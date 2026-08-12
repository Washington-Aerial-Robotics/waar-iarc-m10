"""Pure camera-calibration helpers for the stereo splitter."""


def scaled_camera_info(calib_data, scale=0.5):
    """Return CameraInfo-compatible fields for a uniformly resized image."""
    if not 0.0 < scale <= 1.0:
        raise ValueError("camera scale must be in (0, 1]")
    source_width = int(calib_data["image_width"])
    source_height = int(calib_data["image_height"])
    k = [float(value) for value in calib_data["camera_matrix"]["data"]]
    p = [float(value) for value in calib_data["projection_matrix"]["data"]]
    return {
        "width": int(round(source_width * scale)),
        "height": int(round(source_height * scale)),
        "distortion_model": calib_data["distortion_model"],
        "d": [
            float(value)
            for value in calib_data["distortion_coefficients"]["data"]
        ],
        "k": [
            k[0] * scale,
            k[1],
            k[2] * scale,
            k[3],
            k[4] * scale,
            k[5] * scale,
            k[6],
            k[7],
            k[8],
        ],
        "r": [
            float(value)
            for value in calib_data["rectification_matrix"]["data"]
        ],
        "p": [
            p[0] * scale,
            p[1],
            p[2] * scale,
            p[3] * scale,
            p[4],
            p[5] * scale,
            p[6] * scale,
            p[7] * scale,
            p[8],
            p[9],
            p[10],
            p[11],
        ],
    }
