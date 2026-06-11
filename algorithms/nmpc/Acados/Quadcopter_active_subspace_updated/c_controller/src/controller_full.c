/* Full single-shooting NMPC (40 vars) -- the controlled baseline.
 * Everything (kernel generation, SQP loop, QP solver, tolerances, warm-start
 * shift) is identical to controller.c; only the reduction is absent. */

#include "controller.h"
#include "kernel.h"      /* plant */
#include "kernel_full.h" /* full_fgH */
#include "api.h"         /* DAQP */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define SQP_MAX_ITER 15
#define SQP_TOL 1e-6
#define HESS_REG 1e-8

static void make_ref_stack(double t0, double *ref_stack)
{
    for (int j = 0; j <= AS_N; ++j)
        as_reference(t0 + j * AS_TS, ref_stack + j * AS_NX);
}

static void terminal_feedback(const double *x, const double *x_ref, double *u)
{
    static const int idx[12] = { 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
    double e[12];
    for (int i = 0; i < 12; ++i)
        e[i] = x[idx[i]] - x_ref[idx[i]];
    for (int r = 0; r < AS_NU; ++r) {
        double s = AS_U_HOVER;
        for (int i = 0; i < 12; ++i)
            s -= AS_LQR_K[r * 12 + i] * e[i];
        u[r] = s < AS_U_MIN ? AS_U_MIN : (s > AS_U_MAX ? AS_U_MAX : s);
    }
}

int as_full_init(as_full_ctrl *c)
{
    memset(c, 0, sizeof(*c));
    memcpy(c->U, AS_U_TILDE0, sizeof(c->U));

    long long sz_arg, sz_res, sz_iw, sz_w;
    full_fgH_work(&sz_arg, &sz_res, &sz_iw, &sz_w);
    c->iw = (long long *)malloc((sz_iw > 0 ? sz_iw : 1) * sizeof(long long));
    c->rw = (double *)malloc((sz_w > 0 ? sz_w : 1) * sizeof(double));
    return (c->iw && c->rw) ? 0 : -1;
}

void as_full_free(as_full_ctrl *c)
{
    free(c->iw);
    free(c->rw);
}

int as_full_step(as_full_ctrl *c, const double *x, double t0, double *u0)
{
    double ref_stack[(AS_N + 1) * AS_NX];
    make_ref_stack(t0, ref_stack);

    double f, g[AS_NSTACK], H[AS_NSTACK * AS_NSTACK];
    double blower[AS_NSTACK], bupper[AS_NSTACK];
    int sense[AS_NSTACK];

    DAQPSettings settings;
    daqp_default_settings(&settings);

    c->sqp_iters = 0;
    for (int it = 0; it < SQP_MAX_ITER; ++it) {
        const double *arg[3] = { c->U, x, ref_stack };
        double *res[3] = { &f, g, H };
        if (full_fgH(arg, res, c->iw, c->rw, 0)) return -1;
        for (int i = 0; i < AS_NSTACK; ++i)
            H[i * AS_NSTACK + i] += HESS_REG;

        for (int i = 0; i < AS_NSTACK; ++i) {
            blower[i] = AS_U_MIN - c->U[i];
            bupper[i] = AS_U_MAX - c->U[i];
        }
        memset(sense, 0, sizeof(sense));

        DAQPProblem qp = {
            .n = AS_NSTACK, .m = AS_NSTACK, .ms = AS_NSTACK,
            .H = H, .f = g, .A = NULL,
            .bupper = bupper, .blower = blower, .sense = sense,
        };
        DAQPResult result;
        double d[AS_NSTACK], lam[AS_NSTACK];
        result.x = d;
        result.lam = lam;
        daqp_quadprog(&result, &qp, &settings);
        if (result.exitflag != 1) return -2;

        double step_inf = 0.0;
        for (int i = 0; i < AS_NSTACK; ++i) {
            c->U[i] += d[i];
            double a = fabs(d[i]);
            if (a > step_inf) step_inf = a;
        }
        c->sqp_iters = it + 1;
        if (step_inf < SQP_TOL) break;
    }

    for (int i = 0; i < AS_NU; ++i) {
        double v = c->U[i];
        u0[i] = v < AS_U_MIN ? AS_U_MIN : (v > AS_U_MAX ? AS_U_MAX : v);
    }

    /* shift warm start: identical scheme to the reduced controller */
    double xk[AS_NX], xk1[AS_NX];
    if (as_plant(x, u0, xk)) return -4;
    for (int j = 1; j < AS_N; ++j) {
        if (as_plant(xk, c->U + j * AS_NU, xk1)) return -4;
        memcpy(xk, xk1, sizeof(xk));
    }
    double x_ref_term[AS_NX], u_term[AS_NU];
    as_reference(t0 + AS_N * AS_TS, x_ref_term);
    terminal_feedback(xk, x_ref_term, u_term);

    memmove(c->U, c->U + AS_NU, (AS_NSTACK - AS_NU) * sizeof(double));
    memcpy(c->U + AS_NSTACK - AS_NU, u_term, AS_NU * sizeof(double));

    return 0;
}
