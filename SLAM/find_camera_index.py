import cv2

MAX_INDEX = 10


def read_default(idx):
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        cap.release()
        return None
    ret, frame = cap.read()
    cap.release()
    if not ret:
        return None
    h, w = frame.shape[:2]
    return w, h


def read_forced(idx):
    # some USB stereo cameras only reach their full side-by-side resolution
    # in MJPG mode; try requesting that explicitly in case default mode
    # reports a lower resolution
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        return None
    h, w = frame.shape[:2]
    return w, h


def is_stereo_shaped(w, h):
    return h == 720 and w in (2560, 1280)


def scan():
    print(f"Scanning camera indices 0-{MAX_INDEX - 1}...\n")
    candidates = []  # (idx, w, h, mode)

    for idx in range(MAX_INDEX):
        default = read_default(idx)
        forced = read_forced(idx)

        if default is None and forced is None:
            print(f"  index {idx}: not available")
            continue

        print(f"  index {idx}: default={default}  forced(MJPG 2560x720 request)={forced}")

        if default is not None and is_stereo_shaped(*default):
            candidates.append((idx, default[0], default[1], "default"))
        elif forced is not None and is_stereo_shaped(*forced):
            candidates.append((idx, forced[0], forced[1], "forced"))

    return candidates


def preview_and_select(candidates):
    if not candidates:
        print("\nNo camera reporting a 2560x720 or 1280x720 frame was found on any index.")
        print("Possible causes: the stereo camera needs a different resolution/fourcc")
        print("request than tried here, it's on a higher index than MAX_INDEX, or it's")
        print("a USB connection/driver problem on this machine.")
        return None

    print(f"\nFound {len(candidates)} candidate(s): {[c[0] for c in candidates]}")
    print("Showing a live preview of each. Press 'y' to accept, 'n' for next, 'q' to quit.\n")

    for idx, w, h, mode in candidates:
        cap = cv2.VideoCapture(idx)
        if mode == "forced":
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        print(f"Previewing index {idx} ({w}x{h}, {mode} mode)")
        accepted = False
        while True:
            ret, frame = cap.read()
            if not ret:
                print(f"  Lost connection to index {idx}")
                break

            display = frame.copy()
            cv2.putText(display, f"index {idx}  {w}x{h}  ({mode})  y=accept n=next q=quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("Camera Preview", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('y'):
                accepted = True
                break
            if key == ord('n'):
                break
            if key == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                return None

        cap.release()
        cv2.destroyAllWindows()
        if accepted:
            return idx, w, h, mode

    return None


def main():
    candidates = scan()
    result = preview_and_select(candidates)

    if result is None:
        print("\nNo camera index selected.")
        return

    idx, w, h, mode = result
    print(f"\nSelected camera index: {idx}  ({w}x{h}, {mode} mode)")
    if mode == "forced":
        print("This camera needs MJPG fourcc + a 2560x720 request to reach its full")
        print("stereo resolution — stereo_depth.py's camera init needs those cap.set()")
        print("calls restored (they were removed in an earlier version).")
    else:
        print("This camera reports the stereo resolution by default — no cap.set()")
        print("calls needed to reach it.")


if __name__ == "__main__":
    main()
