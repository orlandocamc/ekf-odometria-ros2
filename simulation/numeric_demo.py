"""ROS-free numeric demo built from the equations in the PMED report.

What comes from the report's annex: `runge_kutta_step` (src/anexo_a_rk4.py).
What is written here from the report's *equations* (the report does not print
this code): the Euler step, the pose composition and `OdomNoiseModel`
(Thrun et al. odometry motion model, chapter II). The alpha values are
illustrative choices made for the figures, NOT values from the report.
The EKF (robot_localization) needs ROS 2 and is not reproduced here.
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from anexo_a_rk4 import KinematicSimNode  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "figures"
OUT.mkdir(exist_ok=True)
_rk4 = KinematicSimNode().runge_kutta_step


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def euler_step(x, y, th, v, w, dt):
    """Euler: q_{k+1} = q_k + h f(q_k, u)."""
    return x + dt * v * np.cos(th), y + dt * v * np.sin(th), wrap(th + dt * w)


class OdomNoiseModel:
    """Thrun odometry model: variances per the report (chapter II):
    s_rot1^2 = a1 d_rot1^2 + a2 d_trans^2
    s_trans^2 = a3 d_trans^2 + a4 (d_rot1^2 + d_rot2^2)
    s_rot2^2 = a1 d_rot2^2 + a2 d_trans^2
    """

    def __init__(self, alphas, seed=0):
        self.a = alphas
        self.rng = np.random.default_rng(seed)  # fixed seed, as in the report

    def perturb(self, d_rot1, d_trans, d_rot2):
        a1, a2, a3, a4 = self.a
        s1 = np.sqrt(a1 * d_rot1**2 + a2 * d_trans**2)
        st = np.sqrt(a3 * d_trans**2 + a4 * (d_rot1**2 + d_rot2**2))
        s2 = np.sqrt(a1 * d_rot2**2 + a2 * d_trans**2)
        return (d_rot1 + self.rng.normal(0, s1),
                d_trans + self.rng.normal(0, st),
                d_rot2 + self.rng.normal(0, s2))


def deltas(p0, p1):
    """Decompose motion p0 -> p1 into rot1, trans, rot2."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    trans = np.hypot(dx, dy)
    rot1 = 0.0 if trans < 1e-9 else wrap(np.arctan2(dy, dx) - p0[2])
    return rot1, trans, wrap(p1[2] - p0[2] - rot1)


def compose(p, rot1, trans, rot2):
    th = p[2] + rot1
    return (p[0] + trans * np.cos(th), p[1] + trans * np.sin(th), wrap(th + rot2))


def run(controls, dt, step):
    p, out = (0.0, 0.0, 0.0), [(0.0, 0.0, 0.0)]
    for v, w in controls:
        p = step(*p, v, w, dt)
        out.append(p)
    return np.array(out)


def square_controls(side=4.0, v=0.5, w=0.5, dt=0.1):
    """Scenario C of the report: 4x4 m square, 90 deg turns at the corners."""
    c = [(v, 0.0)] * round(side / v / dt)
    n = round((np.pi / 2) / w / dt)
    turn = [(0.0, (np.pi / 2) / (n * dt))] * n  # w adjusted so each turn is exactly 90 deg
    return (c + turn) * 4


def main():
    dt = 0.1
    # 1) Euler vs RK4 on a constant arc (v = 0.5 m/s, w = 0.5 rad/s, 4*pi s = 2 laps)
    arc = [(0.5, 0.5)] * round(4 * np.pi / dt)
    rk, eu = run(arc, dt, _rk4), run(arc, dt, euler_step)
    t = np.linspace(0, 2 * np.pi, 400)
    r = 0.5 / 0.5
    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    ax.plot(r * np.sin(t), r * (1 - np.cos(t)), "k:", label="analytic circle")
    ax.plot(rk[:, 0], rk[:, 1], label="RK4 (Annex A)")
    ax.plot(eu[:, 0], eu[:, 1], label="Euler")
    ax.set(aspect="equal", xlabel="x [m]", ylabel="y [m]", title=f"Constant arc, h = {dt} s")
    ax.legend()
    fig.savefig(OUT / "trajectory_euler_vs_rk4.png", dpi=150, bbox_inches="tight")
    err_rk = np.max(np.abs(np.hypot(rk[:, 0], rk[:, 1] - r) - r))
    err_eu = np.max(np.abs(np.hypot(eu[:, 0], eu[:, 1] - r) - r))
    print(f"max radial deviation from the analytic circle: RK4 {err_rk:.2e} m, Euler {err_eu:.2e} m")

    # 2) Scenario C square: ground truth (RK4) vs noisy odometry
    ctrl = square_controls(dt=dt)
    gt = run(ctrl, dt, _rk4)
    plt.figure(figsize=(6, 6))
    plt.plot(gt[:, 0], gt[:, 1], "k", lw=2, label="ground truth (RK4)")
    for seed in range(5):
        nm = OdomNoiseModel((0.01, 0.01, 0.02, 0.01), seed=seed)
        p, odo = gt[0], [gt[0]]
        for k in range(1, len(gt)):
            p = compose(p, *nm.perturb(*deltas(gt[k - 1], gt[k])))
            odo.append(p)
        odo = np.array(odo)
        plt.plot(odo[:, 0], odo[:, 1], lw=0.9, alpha=0.8, label="noisy odometry (5 seeds)" if seed == 0 else None)
        print(f"seed {seed}: final position error {np.hypot(*(odo[-1, :2] - gt[-1, :2])):.3f} m")
    plt.gca().set(aspect="equal", xlabel="x [m]", ylabel="y [m]",
                  title="Square 4x4 m: noise model (illustrative alphas)")
    plt.legend()
    plt.savefig(OUT / "odometry_noisy_square.png", dpi=150, bbox_inches="tight")


if __name__ == "__main__":
    main()
