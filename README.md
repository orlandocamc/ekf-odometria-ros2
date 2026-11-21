# Differential-drive odometry error and EKF in ROS 2 (simulation)

Final project for *Programación de Modelos en Ecuaciones Diferenciales* (PMED),
Universidad Iberoamericana, Mexico City. Developed in **fall 2025**; the report
is dated 1 December 2025.

**Author:** Daniel Orlando Camacho Campos (sole author). Instructor: Roberto Serna Herrera.
Simulation only: no physical robot was involved.

## Problem

A differential-drive robot estimates its pose by integrating wheel velocities
(dead reckoning). Because the integration also integrates the sensor error,
uncertainty grows without bound and the estimated pose drifts from the real one.
The project models that drift and how to manage it.

## Method (as described in the report)

- Kinematic model as a nonlinear ODE system with a non-holonomic constraint
  (state `[x, y, theta]`, inputs `[v, w]`).
- Numerical integration: Euler vs. 4th-order Runge-Kutta (RK4). RK4 generates the
  ground truth so that deviations come from the noise model, not the integrator.
- Probabilistic odometry noise model of Thrun et al. (`OdomNoiseModel`):
  rotation-translation-rotation decomposition with Gaussian noise and four
  alpha parameters; fixed seeds (`numpy.random.default_rng`) for reproducibility.
- Sensor fusion with an Extended Kalman Filter from `robot_localization`
  (ROS 2 Humble), fusing odometry **velocities** (`vx, vy, vyaw`) rather than
  absolute poses.
- Three test scenarios inspired by UMBmark: straight line, pure rotation, closed square.

## Results (reported in the paper, not re-measured here)

- In linear motion the variance of `x` grew linearly with distance; the variance of `y`
  grew quadratically, so angular uncertainty dominates at long range.
- RK4 kept the trajectory geometry with a reported error below 10^-6 m, while Euler
  introduced a systematic drift in closed curves.
- Fusing velocities in the EKF gave a more stable, consistent estimate than fusing
  absolute poses.
- Limitations stated by the author: pure-rolling kinematic model only; dynamic model
  and automatic calibration of the alphas are listed as future work.

## What is in this repository

| Path | Content |
|---|---|
| `docs/informe-pmed-ekf-odometria.pdf` | The full report (Spanish) |
| `src/anexo_a_rk4.py` | RK4 step, transcribed from Annex A of the report |
| `config/ekf.yaml` | EKF configuration, transcribed from Annex B |
| `simulation/numeric_demo.py` | ROS-free demo (see note below) |
| `figures/` | Output of the demo |

**Honest scope notes**

- The original ROS 2 nodes (`kinematic_sim_node`, the teleoperation node) were not
  kept; only the code printed in the report's annexes is available, and it is
  reproduced here as printed. `KinematicSimNode` in `src/anexo_a_rk4.py` is a shell
  added so the method can be imported without ROS 2.
- `OdomNoiseModel` and the Euler step are **not** in the annexes. The demo
  re-implements them from the equations in the report. The alpha values it uses are
  illustrative and are not the report's.
- The figures come from that demo. They illustrate the method; they are not the
  report's experimental results, and no EKF figure is provided because the EKF runs
  inside ROS 2 (`robot_localization`), which was not run here.
- The annex ends the `ekf.yaml` after `odom0_queue_size`; nothing beyond that is invented.

## Run the numeric demo (no ROS needed)

```bash
python -m venv .venv
.venv/Scripts/activate        # Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt
python simulation/numeric_demo.py
```

It writes `figures/trajectory_euler_vs_rk4.png` and `figures/odometry_noisy_square.png`.

## Run the EKF (requires ROS 2 Humble; not run for this repository)

The report used Ubuntu 20.04 and ROS 2 Humble with `robot_localization`. With a node
publishing `/odom_noisy` (the report's simulator), the filter would be launched with
`config/ekf.yaml`, e.g.:

```bash
ros2 run robot_localization ekf_node --ros-args --params-file config/ekf.yaml
```

## References

Thrun, Burgard, Fox, *Probabilistic Robotics* (MIT Press, 2005); Moore and Stouch,
*A Generalized Extended Kalman Filter Implementation for the Robot Operating System*
(IAS-13, 2014). Full list in the report.
