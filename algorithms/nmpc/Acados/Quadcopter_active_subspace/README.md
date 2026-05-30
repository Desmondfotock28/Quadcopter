# Acados Active-Subspace Quadcopter NMPC

An acados implementation of the active-subspace NMPC scheme of
`NMPC_active_Subspace.pdf` (Algorithm 1, "Active-subspace NMPC with
P(x_k, w̃_k)"), implemented up to and including step 3 of that algorithm and
**excluding** any reinforcement-learning component. The supporting thesis is
`DesmondFotock_master_thesis.pdf`.

The physical stacked horizon input is parametrised exactly as in PDF eq. (2.8):

```text
U = T1 V + mu T2 w̃            (w̃ = T2ᵀ ũ)
```

where

- `V` is the active-subspace vector optimised by acados,
- `mu` is the optimised scalar multiplier (PDF eq. (2.9)),
- `w̃ = T2ᵀ ũ` is the inactive part of the shifted candidate `ũ`,
- physical motor-voltage bounds `[0.5, 11]` are enforced on the reconstructed `U`.

The acados model augments the quadcopter state with `[V, mu]` and gives those
variables zero dynamics, so acados optimises one horizon-level active vector
instead of 4 controls at every stage. The physical motor voltages are
reconstructed per stage from stage parameters: `u_real = T1_stage V + mu·w̃_stage`.

## Mapping to the PDF algorithm

`NMPC_active_Subspace.pdf`, **Algorithm 1** (page 9). Every line is implemented
in `main.py:solve_active_subspace_closed_loop` (and helpers):

| PDF (Algorithm 1)                                                                 | Code |
|-----------------------------------------------------------------------------------|------|
| Input `T1, T2, κ, x0`                                                              | `build_projector(...)` builds `T1, T2`; `TerminalFeedback` is `κ`; `X0_QUAD` is `x0`. |
| Line 1: feasible init `ũ0`, `w̃0 ← T2ᵀ ũ0`                                         | `u_tilde = tile(U_HOVER)`, `inactive_stack = T2 (T2ᵀ ũ)`; `initialize_active_variables` warm-starts `v0 = T1ᵀ ũ`, `mu=1` (reconstructs `ũ`). |
| Line 2: solve `P(x_k, w̃_k)` for `(v*, mu*)`                                       | `solver.solve()`; reduced OCP defined in `Quadcopter.export_active_subspace_quadcopter_model`. |
| Line 3: **fallback** `if J(x_k, T1v* + mu*T2w̃) ≤ J(x_k, ũ_k)`                      | `j_reduced = horizon_cost(x, U_reduced, t0)`, `j_candidate = horizon_cost(x, ũ, t0)`; `if j_reduced <= j_candidate`. |
| Line 4: `u_k ← T1v* + mu*T2w̃`                                                     | `chosen_stack = reconstruct_input_stack(T1, v*, mu*, inactive_stack)`. |
| Line 6: `u_k ← ũ_k` (else)                                                         | `chosen_stack = u_tilde`. |
| Line 8: apply first step                                                          | `u0 = clip(chosen_stack[0:4], LB_U, UB_U)` fed to the real-plant integrator. |
| Line 9: `ũ_k = [u_{k|k-1}, …, u_{k+N-2|k-1}, κ(x_{k+N-1|k-1})]`, `w̃_k ← T2ᵀ ũ_k` | `terminal_u = terminal_feedback(x_terminal, x_ref_terminal)`; `u_tilde = shift_input_stack(chosen_stack, terminal_u)`; `inactive_stack = T2 (T2ᵀ ũ)`. |

### The cost `J(x_k, U)` (fallback rule)

`subspace_tools.HorizonCost` evaluates the **exact** OCP objective for a stacked
input `U` by rolling the disturbance-free prediction dynamics (RK4) and summing
the same stage/terminal cost acados uses (Q, R, hover from `Quadcopter.py`):

```text
J(x_k, U) = Σ_{j=0}^{N-1} 0.5(‖x_j-ref_j‖²_Q + ‖u_j-u_hover‖²_R) + ‖x_N-ref_N‖²_Q
```

The small `1e-3(mu-1)²` regulariser of the augmented OCP is excluded so the
reduced candidate and the shifted candidate are compared on the same physical
cost. Both candidates are scored with the identical function, so the comparison
is fair and exact. Per-step `J_reduced`, `J_candidate` and the decision are
saved to `active_subspace_results.npz` and summarised in `metrics.json`.

### Terminal feedback `κ` (recursive feasibility / stability)

`subspace_tools.TerminalFeedback` is a discrete-time **LQR** law obtained by
linearising the nominal dynamics about the physical hover equilibrium
`(x*=0, u*=U_EQUILIBRIUM≈7.075)` — note this differs from the cost reference
`U_HOVER_COST=5.75` — and discretising over `Ts` (zero-order-hold via matrix
exponential). It is applied as a tracking feedback

```text
κ(x) = clip( u_eq - K (x - x_ref),  [0.5, 11] )
```

so the appended terminal input is always within the physical motor bounds and
steers the predicted terminal state toward the reference.

### Projector generation `T1, T2` (PDF eqs. (2.8), (2.10)-(2.11))

`subspace_tools` builds `T1 ∈ ℝ^{40×10}` with orthonormal columns and
`T2 = null_space(T1ᵀ)` so `[T1 T2]` is orthonormal (`T Tᵀ = I`). Selectable on
the CLI via `--projector`:

- **`identity`** — `T1` = first `nv` canonical directions of the stacked input
  (baseline).
- **`pca`** — non-RL active subspace from nominal data. Optimal nominal input
  stacks `U*(x_i)` (the feature `S`) are computed with IPOPT over a sampled set
  of initial conditions `X0` (`solve_nominal_ocp`, the nominal `P(x_k)` of PDF
  eq. (2.3)); the covariance `C = (1/M) Σ S_i S_iᵀ` (eq. (2.10)) is
  eigendecomposed (`C = TΣTᵀ`, eq. (2.11)) and the leading `nv` eigenvectors
  form `T1`.
- **`hessian`** — non-RL active subspace from cost curvature: the leading
  eigenvectors of `∂²J/∂U²` (a sensitivity / curvature feature).

Diagnostics (eigen/singular values, orthonormality residuals `‖T1ᵀT1-I‖` and
`‖T1ᵀT2‖`, captured energy/curvature) are written to `projector_info.json` and
echoed in `metrics.json`.

## acados setup

This repo expects acados to be available separately. The verified local setup:

```bash
git clone https://github.com/acados/acados.git third_party/acados
cd third_party/acados
git submodule update --recursive --init
mkdir -p build && cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_INSTALL_DIR="$(pwd)/.." ..
make install -j4
cd ..
pip install -e interfaces/acados_template
python -c "from acados_template import get_tera; print(get_tera(force_download=True))"
```

`scipy`, `casadi` (with the bundled IPOPT plugin) and `matplotlib` are also
required (the PCA projector solves nominal OCPs with IPOPT).

## Running

```bash
source algorithms/nmpc/Acados/Quadcopter_active_subspace/setup_env.sh
cd algorithms/nmpc/Acados/Quadcopter_active_subspace

MPLBACKEND=Agg python main.py --nsim 120 --projector identity --output-dir results_identity
MPLBACKEND=Agg python main.py --nsim 120 --projector pca      --output-dir results_pca
MPLBACKEND=Agg python main.py --nsim 120 --projector hessian  --output-dir results_hessian
```

CLI options: `--projector {identity,pca,hessian}`, `--nsim`, `--output-dir`,
`--pca-samples` (initial-condition samples for the PCA covariance, default 24),
`--seed`, `--no-plots`. Each run writes `metrics.json`, `projector_info.json`,
`active_subspace_results.npz`, `position_tracking.png`, `trajectory_3d.png`.

## Verified results (nsim = 120)

Both runs completed with **all 120 acados OCP solves returning status 0** and
all applied motor commands inside `[0.5, 11]`. The projector matrices satisfy
`‖T1ᵀT1 - I‖ ≈ 1e-15` and `‖T1ᵀT2‖ ≈ 1e-15`.

| Metric                         | `identity` | `pca`      |
|--------------------------------|-----------:|-----------:|
| solver status 0                | 120 / 120  | 120 / 120  |
| min / max motor command        | 5.65 / 10.62 | 5.59 / 9.83 |
| mean position error (m)        | 0.175      | 0.075      |
| max position error (m)         | 0.274      | 0.109      |
| reduced applied (fallback)     | 34 / 120   | 60 / 120   |
| candidate applied (fallback)   | 86 / 120   | 60 / 120   |
| mean `J_reduced`               | 16.69      | 13.80      |
| mean `J_candidate`             | 24.40      | 21.92      |
| mean solve time (s)            | 0.022      | 0.055      |

The fallback rule always keeps `J(applied) ≤ J(ũ_k)` (per step
`applied_reduced == (J_reduced ≤ J_candidate)`), which is the recursive-feasibility
safeguard of Algorithm 1: at step 0 the reduced solution beats the poor hover
candidate (`J ≈ 32` vs `≈ 955`), and thereafter the shifted candidate — carrying
a fuller previous solution forward — is sometimes cheaper than the
restricted reduced re-solve.

The `pca` projector captures **99.9997 %** of the optimal-input-stack energy in
10 of 40 directions (eigenvalues decay `2078 → 31 → 7.8 → … → ~1e-13`), applies
the reduced solution twice as often (50 % vs 28 %) and tracks ~2.3× better than
the `identity` baseline.

> Generated acados code (`c_generated_code/`, `acados_*.json`), run outputs
> (`results*/`) and `third_party/acados/` are git-ignored and not committed.
