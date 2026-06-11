/* Embedded active-subspace NMPC controller (Algorithm 1, reduced OCP).
 *
 * Dense Gauss-Newton SQP over z = [v; mu] (AS_NZ vars), single shooting.
 * Kernel (cost / gradient / GN Hessian) is CasADi-generated C; QP subproblems
 * are solved exactly by DAQP. Input constraints are linear in z and therefore
 * exact in every QP. Includes the Algorithm 1 fallback rule and the LQR
 * terminal-feedback candidate shift.
 */
#ifndef AS_CONTROLLER_H
#define AS_CONTROLLER_H

#include "params.h"

typedef struct {
    /* candidate input stack u~ and its inactive projection w = (I - T1 T1^T) u~ */
    double u_tilde[AS_NSTACK];
    double w[AS_NSTACK];

    /* scratch for the SQP */
    double z[AS_NZ];
    double A_cons[AS_NSTACK * AS_NZ]; /* [T1 | w], row-major, rebuilt per step */

    /* casadi work arrays (sized at init) */
    long long *iw;
    double *rw;

    /* diagnostics for the last step */
    int sqp_iters;
    int used_fallback;
    double j_reduced;
    double j_candidate;
} as_ctrl;

/* initialise with the precomputed feasible candidate AS_U_TILDE0 */
int as_ctrl_init(as_ctrl *c);
void as_ctrl_free(as_ctrl *c);

/* one closed-loop step: state x (AS_NX) at time t0 -> applied input u0 (AS_NU).
 * Solves the reduced OCP, applies the fallback rule, shifts the candidate with
 * the LQR terminal feedback. Returns 0 on success. */
int as_ctrl_step(as_ctrl *c, const double *x, double t0, double *u0);

/* reference trajectory (matches utils.reference_trajectory) */
void as_reference(double t, double *ref13);

/* RK4 plant integration over one sample time (casadi-generated, 30 substeps) */
int as_plant(const double *x, const double *u, double *x_next);

/* ------------------------------------------------------------------------
 * Full NMPC (40 variables) in the IDENTICAL dense-SQP framework: same
 * generated-kernel structure, same DAQP, same SQP loop, same LQR shift.
 * Exists purely as the controlled baseline -- the only difference from the
 * reduced controller is the absence of the active-subspace reduction.
 * ------------------------------------------------------------------------ */

typedef struct {
    double U[AS_NSTACK];   /* current iterate / shifted warm start */
    long long *iw;
    double *rw;
    int sqp_iters;
} as_full_ctrl;

int as_full_init(as_full_ctrl *c);
void as_full_free(as_full_ctrl *c);
int as_full_step(as_full_ctrl *c, const double *x, double t0, double *u0);

#endif
