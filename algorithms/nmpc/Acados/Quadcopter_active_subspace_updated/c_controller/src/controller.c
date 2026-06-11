#include "controller.h"
#include "kernel.h"
#include "api.h" /* DAQP */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define SQP_MAX_ITER 15
#define SQP_TOL 1e-6
#define HESS_REG 1e-8
#define BIG 1e20

/* ------------------------------------------------------------------ utils */

void as_reference(double t, double *ref)
{
    memset(ref, 0, AS_NX * sizeof(double));
    ref[0] = sin(M_PI * t / 10.0);
    ref[1] = cos(M_PI * t / 10.0) - 1.0;
    ref[2] = sin(M_PI * t / 10.0) + t + 1.0;
    ref[3] = 1.0; /* qw */
}

static void make_ref_stack(double t0, double *ref_stack)
{
    for (int j = 0; j <= AS_N; ++j)
        as_reference(t0 + j * AS_TS, ref_stack + j * AS_NX);
}

/* z_active = T1^T u (AS_NV), T1 row-major (AS_NSTACK x AS_NV) */
static void t1_t_mul(const double *u, double *out)
{
    for (int k = 0; k < AS_NV; ++k) out[k] = 0.0;
    for (int i = 0; i < AS_NSTACK; ++i)
        for (int k = 0; k < AS_NV; ++k)
            out[k] += AS_T1[i * AS_NV + k] * u[i];
}

/* u = T1 v + mu * w */
static void reconstruct(const double *v, double mu, const double *w, double *u)
{
    for (int i = 0; i < AS_NSTACK; ++i) {
        double s = mu * w[i];
        for (int k = 0; k < AS_NV; ++k)
            s += AS_T1[i * AS_NV + k] * v[k];
        u[i] = s;
    }
}

/* w = (I - T1 T1^T) u  -- inactive component, avoids storing T2 */
static void inactive_proj(const double *u, double *w)
{
    double v[AS_NV];
    t1_t_mul(u, v);
    reconstruct(v, 0.0, u /*unused*/, w); /* w = T1 v */
    for (int i = 0; i < AS_NSTACK; ++i)
        w[i] = u[i] - w[i];
}

int as_plant(const double *x, const double *u, double *x_next)
{
    const double *arg[2] = { x, u };
    double *res[1] = { x_next };
    static long long iw[64];
    static double rw[1024];
    return plant(arg, res, iw, rw, 0);
}

/* clipped LQR terminal feedback on the 12-dim error state (qw excluded) */
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

/* ---------------------------------------------------------------- kernel */

static int eval_fgH(as_ctrl *c, const double *z, const double *x, const double *ref,
                    double *f, double *g, double *H)
{
    const double *arg[4] = { z, x, ref, c->w };
    double *res[3] = { f, g, H };
    return red_fgH(arg, res, c->iw, c->rw, 0);
}

static int eval_f(as_ctrl *c, const double *z, const double *x, const double *ref, double *f)
{
    const double *arg[4] = { z, x, ref, c->w };
    double *res[1] = { f };
    return red_f(arg, res, c->iw, c->rw, 0);
}

/* ------------------------------------------------------------------ init */

int as_ctrl_init(as_ctrl *c)
{
    memset(c, 0, sizeof(*c));
    memcpy(c->u_tilde, AS_U_TILDE0, sizeof(c->u_tilde));
    inactive_proj(c->u_tilde, c->w);

    long long sz_arg, sz_res, sz_iw, sz_w;
    red_fgH_work(&sz_arg, &sz_res, &sz_iw, &sz_w);
    long long sz_arg2, sz_res2, sz_iw2, sz_w2;
    red_f_work(&sz_arg2, &sz_res2, &sz_iw2, &sz_w2);
    if (sz_iw2 > sz_iw) sz_iw = sz_iw2;
    if (sz_w2 > sz_w) sz_w = sz_w2;
    c->iw = (long long *)malloc((sz_iw > 0 ? sz_iw : 1) * sizeof(long long));
    c->rw = (double *)malloc((sz_w > 0 ? sz_w : 1) * sizeof(double));
    return (c->iw && c->rw) ? 0 : -1;
}

void as_ctrl_free(as_ctrl *c)
{
    free(c->iw);
    free(c->rw);
}

/* ------------------------------------------------------------------ step */

int as_ctrl_step(as_ctrl *c, const double *x, double t0, double *u0)
{
    double ref_stack[(AS_N + 1) * AS_NX];
    make_ref_stack(t0, ref_stack);

    /* warm start: v = T1^T u~, mu = 1 reconstructs the candidate exactly */
    t1_t_mul(c->u_tilde, c->z);
    c->z[AS_NV] = 1.0;

    /* constraint matrix [T1 | w] (rebuilt because w changes every step) */
    for (int i = 0; i < AS_NSTACK; ++i) {
        memcpy(c->A_cons + i * AS_NZ, AS_T1 + i * AS_NV, AS_NV * sizeof(double));
        c->A_cons[i * AS_NZ + AS_NV] = c->w[i];
    }

    double f, g[AS_NZ], H[AS_NZ * AS_NZ];
    double blower[AS_NZ + AS_NSTACK], bupper[AS_NZ + AS_NSTACK];
    int sense[AS_NZ + AS_NSTACK];

    double j_candidate = -1.0; /* f at the warm start (= candidate cost)     */
    c->sqp_iters = 0;

    DAQPSettings settings;
    daqp_default_settings(&settings);

    for (int it = 0; it < SQP_MAX_ITER; ++it) {
        if (eval_fgH(c, c->z, x, ref_stack, &f, g, H)) return -1;
        if (it == 0) j_candidate = f;
        for (int i = 0; i < AS_NZ; ++i)
            H[i * AS_NZ + i] += HESS_REG;

        /* simple bounds on the step d (v free, mu in [MU_MIN, MU_MAX]) */
        for (int i = 0; i < AS_NV; ++i) { blower[i] = -BIG; bupper[i] = BIG; }
        blower[AS_NV] = AS_MU_MIN - c->z[AS_NV];
        bupper[AS_NV] = AS_MU_MAX - c->z[AS_NV];
        /* general rows: U_MIN <= A (z + d) <= U_MAX */
        for (int i = 0; i < AS_NSTACK; ++i) {
            double az = 0.0;
            for (int k = 0; k < AS_NZ; ++k)
                az += c->A_cons[i * AS_NZ + k] * c->z[k];
            blower[AS_NZ + i] = AS_U_MIN - az;
            bupper[AS_NZ + i] = AS_U_MAX - az;
        }
        memset(sense, 0, sizeof(sense));

        DAQPProblem qp = {
            .n = AS_NZ, .m = AS_NZ + AS_NSTACK, .ms = AS_NZ,
            .H = H, .f = g, .A = c->A_cons,
            .bupper = bupper, .blower = blower, .sense = sense,
        };
        DAQPResult result;
        double d[AS_NZ], lam[AS_NZ + AS_NSTACK];
        result.x = d;
        result.lam = lam;
        daqp_quadprog(&result, &qp, &settings);
        if (result.exitflag != 1) return -2;

        double step_inf = 0.0;
        for (int i = 0; i < AS_NZ; ++i) {
            c->z[i] += d[i];
            double a = fabs(d[i]);
            if (a > step_inf) step_inf = a;
        }
        c->sqp_iters = it + 1;
        if (step_inf < SQP_TOL) break;
    }

    /* reconstruct and apply the Algorithm 1 fallback rule */
    double u_reduced[AS_NSTACK];
    reconstruct(c->z, c->z[AS_NV], c->w, u_reduced);

    double j_reduced;
    if (eval_f(c, c->z, x, ref_stack, &j_reduced)) return -3;
    c->j_reduced = j_reduced;
    c->j_candidate = j_candidate;

    const double *chosen = u_reduced;
    c->used_fallback = 0;
    if (j_reduced > j_candidate) { /* reduced solve worse -> keep candidate */
        chosen = c->u_tilde;
        c->used_fallback = 1;
    }

    for (int i = 0; i < AS_NU; ++i) {
        double v = chosen[i];
        u0[i] = v < AS_U_MIN ? AS_U_MIN : (v > AS_U_MAX ? AS_U_MAX : v);
    }

    /* shift candidate: predict terminal state under `chosen`, append LQR */
    double xk[AS_NX], xk1[AS_NX];
    if (as_plant(x, u0, xk)) return -4;
    for (int j = 1; j < AS_N; ++j) {
        if (as_plant(xk, chosen + j * AS_NU, xk1)) return -4;
        memcpy(xk, xk1, sizeof(xk));
    }
    double x_ref_term[AS_NX], u_term[AS_NU];
    as_reference(t0 + AS_N * AS_TS, x_ref_term);
    terminal_feedback(xk, x_ref_term, u_term);

    double u_new[AS_NSTACK];
    memcpy(u_new, chosen + AS_NU, (AS_NSTACK - AS_NU) * sizeof(double));
    memcpy(u_new + AS_NSTACK - AS_NU, u_term, AS_NU * sizeof(double));
    memcpy(c->u_tilde, u_new, sizeof(u_new));
    inactive_proj(c->u_tilde, c->w);

    return 0;
}
