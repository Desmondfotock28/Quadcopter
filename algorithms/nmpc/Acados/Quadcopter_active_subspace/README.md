# Acados Active-Subspace Quadcopter NMPC

This is an acados prototype of the active-subspace NMPC idea used in
`algorithms/nmpc/active_subspace`.

The physical stacked horizon input is reconstructed as:

```text
U = T1 V + mu T2 w
```

where:

- `V` is the active vector optimized by acados,
- `mu` is an optimized scalar multiplier,
- `w = T2.T @ u_tilde` is updated from the shifted previous solution,
- physical motor-voltage bounds are enforced on the reconstructed `U`.

The acados model augments the quadcopter state with `[V, mu]` and gives those
variables zero dynamics. This keeps `V` and `mu` constant across the prediction
horizon while allowing acados to optimize them as horizon-level variables.

## acados setup

This repo expects acados to be available separately. The verified local setup used:

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

For this workspace, use:

```bash
source algorithms/nmpc/Acados/Quadcopter_active_subspace/setup_env.sh
```

or run with explicit environment variables:

```bash
env ACADOS_SOURCE_DIR=/home/jkan67/Downloads/Quadcopter/third_party/acados \
    LD_LIBRARY_PATH=/home/jkan67/Downloads/Quadcopter/third_party/acados/lib \
    MPLBACKEND=Agg \
    python main.py --nsim 120 --output-dir results
```

## Verified result

A 120-step closed-loop run completed with all acados OCP solves returning status `0`.
The result files are written under `results/`:

- `metrics.json`
- `active_subspace_results.npz`
- `position_tracking.png`
- `trajectory_3d.png`

Observed metrics from the verified run:

```json
{
  "nsim": 120,
  "active_dimension": 10,
  "stacked_input_dimension": 40,
  "mean_solve_time_s": 0.01686671773592631,
  "max_solve_time_s": 0.04539752006530762,
  "mean_position_error_m": 0.05371550736296165,
  "final_position_error_m": 0.013964206745301275,
  "max_position_error_m": 0.09438021065655558,
  "min_motor_command": 6.658590262427323,
  "max_motor_command": 10.62381228798958,
  "solver_status_counts": {
    "0": 120
  }
}
```
