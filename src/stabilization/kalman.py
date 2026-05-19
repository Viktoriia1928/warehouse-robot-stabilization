import numpy as np


def kalman_smooth_axis(z, q=1e-3, r=1.0, mode="rts"):
    """Constant-velocity 1D Kalman with optional RTS backward pass."""
    n = len(z)
    F = np.array([[1.0, 1.0], [0.0, 1.0]])
    Q = q * np.array([[0.25, 0.5], [0.5, 1.0]])

    x_pred = np.zeros((n, 2)); P_pred = np.zeros((n, 2, 2))
    x_filt = np.zeros((n, 2)); P_filt = np.zeros((n, 2, 2))

    x_filt[0] = [z[0], 0.0]
    P_filt[0] = np.eye(2) * 10.0

    for k in range(1, n):
        x_pred[k] = F @ x_filt[k - 1]
        P_pred[k] = F @ P_filt[k - 1] @ F.T + Q
        S = P_pred[k][0, 0] + r
        K = P_pred[k][:, 0] / S
        x_filt[k] = x_pred[k] + K * (z[k] - x_pred[k][0])
        P_filt[k] = P_pred[k] - np.outer(K, P_pred[k][0, :])

    if mode == "filter":
        return x_filt[:, 0]

    x_smooth = np.zeros((n, 2))
    x_smooth[-1] = x_filt[-1]
    for k in range(n - 2, -1, -1):
        try:
            C = P_filt[k] @ F.T @ np.linalg.inv(P_pred[k + 1])
        except np.linalg.LinAlgError:
            C = np.zeros((2, 2))
        x_smooth[k] = x_filt[k] + C @ (x_smooth[k + 1] - x_pred[k + 1])
    return x_smooth[:, 0]


def kalman_smooth_trajectory(
    trajectory,
    q=(1e-3, 1e-3, 1e-5),
    r=(1.0, 1.0, 1e-3),
    mode="rts",
):
    out = np.zeros_like(trajectory, dtype=np.float64)
    for i in range(3):
        out[:, i] = kalman_smooth_axis(trajectory[:, i], q=q[i], r=r[i], mode=mode)
    return out
