# Active-subspace NMPC (updated) — review guide

Implements **Algorithm 1** of `documents/DesmondFotock_master_thesis.pdf`
(active-subspace NMPC with fallback and recursive feasibility) for the F450
quadcopter, in three forms that produce identical solutions:

| file | what it is | role |
|---|---|---|
| `main.py` (+ `Quadcopter.py`) | acados encoding (augmented states `v, mu`) | correctness testbed vs acados; `--legacy` reproduces the first broken version |
| `reduced_sqp.py` | dense single-shooting Gauss-Newton SQP (CasADi + DAQP), 11 variables | the solver the thesis prescribes; Python reference |
| `c_controller/` | embedded C port of the same (generated kernels + DAQP) | for the Raspberry Pi; cross-validated digit-for-digit against `reduced_sqp.py` |
| `subspace_tools.py` | shared pieces: projectors, horizon cost, nominal solve, LQR `kappa` | used by all of the above |
| `full_nmpc_baseline.py` | standard full NMPC in acados | optimality/timing baseline |

## Algorithm 1 → code map (for checking the algorithm)

| thesis | what it says | where implemented |
|---|---|---|
| eq. (2.8) | `u = [T1 T2][v; w]`, `T` orthonormal | `subspace_tools.py::_finish_projector` (QR + null space; residuals ~1e-15) |
| eq. (2.9) | reduced program over `(v, mu)`, constraints mapped | `reduced_sqp.py::make_*`/`solve_reduced_sqp` (constraints `U_MIN <= [T1|w] z <= U_MAX`, exact/linear); `Quadcopter.py::export_active_subspace_quadcopter_model` + `main.py::create_ocp_solver_description` (via `con_h_expr`); `c_controller/src/controller.c` |
| eqs. (2.10)–(2.11), `C = H` route | projector from eigendecomposition of the cost Hessian | `subspace_tools.py::build_hessian_projector` (exact `d2J/dU2` at hover; nv=10 captures 99.8% curvature) |
| line 1 | solve nominal `P(x0)` for `u~_0` | `subspace_tools.py::solve_nominal_ocp` (IPOPT single shooting); baked into the C controller at export (`c_controller/export_c_code.py`) |
| line 2 | solve reduced `P(x_k, w~_k)` | SQP loop in `reduced_sqp.py::solve_reduced_sqp` / `controller.c::as_ctrl_step`; acados solve in `main.py` |
| lines 3–7 | fallback: apply reduced solution only if `J <= J(u~_k)` | `main.py` (`j_reduced <= j_candidate`), `reduced_sqp.py::run_closed_loop`, `controller.c` (`j_candidate` = cost at the warm start, which reconstructs `u~` exactly) |
| line 8 | apply first input | all closed loops |
| line 9 | shift chosen sequence, append `kappa(x_{k+N|k})`, update `w~` | terminal rollout of the **chosen** stack + clipped LQR `kappa` (`subspace_tools.py::TerminalFeedback`, 12-dim quaternion error state) in all three paths |

Documented deviations from the thesis (deliberate):
- `mu` is bounded to `[0, 1.5]` instead of `mu in R` (keeps `mu=0` reachable, helps the solver).
- `main.py`'s acados encoding is *not* single shooting (acados is stage-structured); the
  thesis-conforming single-shooting solver is `reduced_sqp.py` / `c_controller`.
- `kappa` is a clipped LQR taking `(x, x_ref)` — a tracking instantiation of the generic
  state feedback the thesis allows.

## Model note

`Quadcopter.py` fixes two issues vs the first version: the input cost is centred at hover
thrust (`m g / 4`), not max thrust; and the yaw coefficient is `cd = 0.017` N·m/N (the
torque-to-thrust ratio, matching `model/quad_f450_camera/model.sdf` momentConstant — the
old `1.6e-7` was a rotor-speed-squared coefficient applied to forces, leaving yaw
uncontrolled).

## Results (400-step closed loop, all deterministic & cross-validated C ≡ Python)

| controller | vars | closed-loop cost | mean pos err | solve time (idle x86) |
|---|---|---|---|---|
| full NMPC (C, same framework) | 40 | 466.91 | 0.0079 m | 2.5 ms mean / 4.8 max |
| **reduced active-subspace (C)** | **11** | **483.16 (+3.5%)** | 0.0398 m | **0.9 ms mean / 1.9 max** |
| acados full NMPC (ERK) | — | 466.9 | 0.0079 m | 3.5 ms mean |
| acados reduced (augmented states) | — | 483.2 | 0.0398 m | 6.4–9 ms (slower than full: the encoding inflates the per-stage state; see `c_controller/README.md`) |

Reduction speedup in the controlled C comparison: **~2.6–2.7x mean** for **+3.5%** cost.
Fallback fires 1/400 (step 0, an exact tie — the rule working as designed).

## How to run

```bash
# Python reference (full vs reduced, prints both + speedup)
../../../../.venv-acados/bin/python reduced_sqp.py

# acados paths (needs third_party/acados built; see setup_env.sh in the sibling folder)
export ACADOS_SOURCE_DIR=<repo>/third_party/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
../../../../.venv-acados/bin/python main.py --integrator ERK          # fixed version
../../../../.venv-acados/bin/python main.py --legacy                  # original broken version
../../../../.venv-acados/bin/python full_nmpc_baseline.py --integrator ERK

# embedded C controller (see c_controller/README.md, incl. Raspberry Pi)
cd c_controller && python export_c_code.py && mkdir -p build && cd build && cmake .. && make
./as_nmpc_benchmark 400
```
