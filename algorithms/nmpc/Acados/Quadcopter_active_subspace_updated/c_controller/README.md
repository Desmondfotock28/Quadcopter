# Embedded active-subspace NMPC controller (C)

Self-contained C implementation of the reduced-space NMPC (thesis Algorithm 1):
dense Gauss-Newton SQP over `z = [v; mu]` (11 variables), single shooting,
exact linear input constraints, DAQP for the QP subproblems, Algorithm 1
fallback rule and LQR terminal-feedback candidate shift.

## Why this exists

The acados encoding of this problem (augmented states with zero dynamics) is
*slower* than full NMPC because stage-structured solvers pay for the inflated
state and never see the variable reduction. The thesis prescribes single
shooting; implemented that way the reduction wins.

## Controlled comparison

`src/controller_full.c` is a full 40-variable NMPC in the IDENTICAL framework:
same CasADi-generated kernel structure, same DAQP, same SQP loop and
tolerances, same LQR warm-start shift, same plant and benchmark loop. The only
difference is the active-subspace parametrisation `U = T1 v + mu w`, so the
measured gap is attributable to the reduction alone (400 steps, idle x86
desktop -- absolute times shift with machine load; the speedup ratio and all
quality metrics are load-insensitive and fully deterministic):

| controller                  | vars | mean     | p95      | max      | closed-loop cost |
|-----------------------------|------|----------|----------|----------|------------------|
| full NMPC (this framework)  | 40   | 2.47 ms  | 3.10 ms  | 4.83 ms  | 466.91           |
| reduced active-subspace     | 11   | **0.91 ms** | **1.08 ms** | **1.87 ms** | 483.16 (+3.5%) |

**Reduction speedup: ~2.7x mean, for 3.5% closed-loop cost.**

Context against acados (same problem, 400 steps): acados full NMPC (ERK)
3.5 ms mean, acados reduced (best config) 6.4 ms. Both C controllers
reproduce their Python references (`reduced_sqp.py`) exactly: J = 483.16 /
466.91, mean position error 0.0398 / 0.0079 m — verified ports, not
approximations. The initial candidate u~_0 baked into params.h is the true
nominal P(x0) optimum (solved with IPOPT at export time, Algorithm 1 line 1),
and the yaw torque coefficient matches the repo's Gazebo F450 model
(momentConstant = 0.017).

## Layout

- `export_c_code.py` — regenerates `gen/` (CasADi kernel + plant as plain C,
  `params.h` with the projector T1, LQR gain, initial candidate). Rerun after
  any model/cost/horizon change.
- `gen/kernel.c|h` — generated; do not edit.
- `src/controller.c|h` — the controller: warm start, SQP loop, fallback, shift.
- `src/main.c` — 400-step closed-loop benchmark (RK4 plant, timing stats).
- DAQP lives at `<repo_root>/third_party/daqp` (gitignored — clone it once).

## Build (desktop or Raspberry Pi)

```bash
# one-time: fetch DAQP and generate the kernels
git clone https://github.com/darnstrom/daqp.git <repo_root>/third_party/daqp
python export_c_code.py        # needs casadi, numpy, scipy (e.g. .venv-acados)

cd c_controller
mkdir -p build && cd build
cmake .. && make          # kernels are big TUs; first compile takes a few min
./as_nmpc_benchmark 400
```

On a Raspberry Pi (4 or 5, 64-bit OS) the identical commands work — everything
is portable C99 with no x86 intrinsics. Expect roughly 2.5–4x the desktop
solve times on a Pi 5 (Cortex-A76) and 4–8x on a Pi 4 (Cortex-A72), i.e. a
worst case around 10–20 ms — comfortably inside the 100 ms sampling time.
Add `-mcpu=native` to the compile options in CMakeLists.txt for a free few
percent. Pin the control thread to an isolated core (`taskset -c 3`) and use
`SCHED_FIFO` for jitter-free timing in flight.

## Using it on the real system

`as_ctrl_step(&ctrl, x, t, u0)` is the whole interface: feed the current state
estimate, get 4 motor thrusts [N]. Heap allocation happens only in
`as_ctrl_init`; the step itself is allocation-free apart from DAQP's internal
workspace (replace `daqp_quadprog` with the `setup_daqp`/`daqp_solve` pair to
make it fully static if required).
