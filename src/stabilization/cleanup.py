import numpy as np

try:
    from scipy.signal import medfilt as _scipy_medfilt

    def medfilt(x, kernel_size):
        return _scipy_medfilt(x, kernel_size=kernel_size)

except ImportError:
    def medfilt(x, kernel_size):
        r = kernel_size // 2
        padded = np.pad(x, (r, r), mode="edge")
        out = np.empty_like(x, dtype=np.float64)
        for i in range(len(x)):
            out[i] = np.median(padded[i:i + kernel_size])
        return out


def robust_std(x):
    mad = np.median(np.abs(x - np.median(x)))
    return mad / 0.6745


def clean_transforms(transforms, median_kernel=5, mad_k=4.0, verbose=False):
    cleaned = transforms.copy()

    for i in range(3):
        cleaned[:, i] = medfilt(cleaned[:, i], kernel_size=median_kernel)

    for i, name in enumerate(("dx", "dy", "da")):
        thr = max(mad_k * robust_std(cleaned[:, i]), 1e-6)
        n_clip = int(np.sum(np.abs(cleaned[:, i]) > thr))
        cleaned[:, i] = np.clip(cleaned[:, i], -thr, thr)
        if verbose:
            print(f"[clean] {name}: thr={thr:.3f}  clipped={n_clip}")

    return cleaned
