# NMPC Algorithm Development

This directory contains NMPC (Nonlinear Model Predictive Control) algorithm implementations for quadcopter trajectory tracking. These are development/research implementations - not for direct deployment on the Raspberry Pi.

## Directory Structure

| Folder | Description |
|--------|-------------|
| `Multiple_Single_shooting/` | Single and multiple shooting NMPC using CasADi |
| `Feedback_Linearisation/` | Feedback linearization MPC with disturbance observer |
| `active_subspace/` | Active subspace optimization for computational efficiency |
| `Acados/` | Production-ready implementations using Acados solver |

## Quick Start

```bash
# Single shooting NMPC
python Multiple_Single_shooting/single_shooting_nmpc.py

# Multiple shooting NMPC
python Multiple_Single_shooting/multiple_shooting_nmpc.py

# Feedback linearization with disturbance observer
python Feedback_Linearisation/multiple_shooting_DOB.py

# Active subspace optimization
python active_subspace/single_shooting_active_subspace.py
```

## Dependencies

- CasADi (for optimization)
- NumPy, SciPy, Matplotlib
- Acados (for Acados implementations)

See `requirements.txt` in the project root.

## Documentation

For detailed theory and implementation details, see the wiki:
- [Control Theory](https://github.com/Desmondfotock28/Quadcopter/wiki/Control-Theory) - Mathematical foundations
- [NMPC Implementation](https://github.com/Desmondfotock28/Quadcopter/wiki/NMPC-Implementation) - Algorithm details and performance analysis
