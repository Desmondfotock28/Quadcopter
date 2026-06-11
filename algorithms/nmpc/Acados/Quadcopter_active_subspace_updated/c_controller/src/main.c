/* Closed-loop benchmark: reduced (11-var) vs full (40-var) NMPC, both in the
 * IDENTICAL dense-SQP framework (same generated kernels, DAQP, tolerances,
 * warm-start shift, plant, reference). Any timing difference is attributable
 * to the active-subspace reduction alone.
 *
 * Cross-validation targets (from the Python reference implementations):
 *   reduced: J ~ 483.16, mean pos err ~ 0.0398
 *   full   : J ~ 466.91, mean pos err ~ 0.0079
 */

#include "controller.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static int cmp_double(const void *a, const void *b)
{
    double d = *(const double *)a - *(const double *)b;
    return d < 0 ? -1 : (d > 0 ? 1 : 0);
}

/* stage cost weights, must match Quadcopter.py */
static const double Q_DIAG[AS_NX] = { 40, 40, 50, 1.0, 0.043, 0.043, 0.043,
                                      2.0, 2.0, 2.0, 1.0, 1.0, 1.0 };
#define R_WEIGHT 0.1

typedef struct {
    double mean_ms, p95_ms, max_ms;
    double err_mean, err_max, err_final;
    double jcl;
    double iters_mean;
    int iters_max, n_fallback;
} loop_stats;

/* generic closed loop over a controller step callback */
typedef int (*step_fn)(void *ctrl, const double *x, double t0, double *u0,
                       int *iters, int *fallback);

static int run_loop(void *ctrl, step_fn step, int nsim, loop_stats *out)
{
    double x[AS_NX], ref[AS_NX];
    memcpy(x, AS_X0, sizeof(x));
    double t0 = 0.0;

    double *solve_ms = malloc(nsim * sizeof(double));
    double err_sum = 0.0, err_max = 0.0, err_final = 0.0, jcl = 0.0;
    long iter_sum = 0;
    int iter_max = 0, n_fallback = 0;

    for (int i = 0; i < nsim; ++i) {
        double u0[AS_NU];
        int iters = 0, fb = 0;

        double tic = now_s();
        int rc = step(ctrl, x, t0, u0, &iters, &fb);
        solve_ms[i] = 1e3 * (now_s() - tic);
        if (rc) { fprintf(stderr, "step %d failed rc=%d\n", i, rc); free(solve_ms); return rc; }

        iter_sum += iters;
        if (iters > iter_max) iter_max = iters;
        n_fallback += fb;

        as_reference(t0, ref);
        double jc = 0.0;
        for (int k = 0; k < AS_NX; ++k) {
            double e = x[k] - ref[k];
            jc += Q_DIAG[k] * e * e;
        }
        for (int k = 0; k < AS_NU; ++k) {
            double e = u0[k] - AS_U_HOVER;
            jc += R_WEIGHT * e * e;
        }
        jcl += 0.5 * jc;

        double x_next[AS_NX];
        if (as_plant(x, u0, x_next)) { free(solve_ms); return -10; }
        memcpy(x, x_next, sizeof(x));
        t0 += AS_TS;

        as_reference(t0, ref);
        double e2 = 0.0;
        for (int k = 0; k < 3; ++k) {
            double e = x[k] - ref[k];
            e2 += e * e;
        }
        double err = sqrt(e2);
        err_sum += err;
        err_final = err;
        if (err > err_max) err_max = err;
    }

    qsort(solve_ms, nsim, sizeof(double), cmp_double);
    double t_sum = 0.0;
    for (int i = 0; i < nsim; ++i) t_sum += solve_ms[i];

    out->mean_ms = t_sum / nsim;
    out->p95_ms = solve_ms[(int)(0.95 * nsim)];
    out->max_ms = solve_ms[nsim - 1];
    out->err_mean = err_sum / nsim;
    out->err_max = err_max;
    out->err_final = err_final;
    out->jcl = jcl;
    out->iters_mean = (double)iter_sum / nsim;
    out->iters_max = iter_max;
    out->n_fallback = n_fallback;

    free(solve_ms);
    return 0;
}

static int step_reduced(void *ctrl, const double *x, double t0, double *u0,
                        int *iters, int *fb)
{
    as_ctrl *c = (as_ctrl *)ctrl;
    int rc = as_ctrl_step(c, x, t0, u0);
    *iters = c->sqp_iters;
    *fb = c->used_fallback;
    return rc;
}

static int step_full(void *ctrl, const double *x, double t0, double *u0,
                     int *iters, int *fb)
{
    as_full_ctrl *c = (as_full_ctrl *)ctrl;
    int rc = as_full_step(c, x, t0, u0);
    *iters = c->sqp_iters;
    *fb = 0;
    return rc;
}

static void print_stats(const char *name, int nvars, int nsim, const loop_stats *s)
{
    printf("=== %s (%d vars, %d steps) ===\n", name, nvars, nsim);
    printf("sqp iters        : mean %5.1f  max %d\n", s->iters_mean, s->iters_max);
    printf("solve time [ms]  : mean %7.3f  p95 %7.3f  max %7.3f\n",
           s->mean_ms, s->p95_ms, s->max_ms);
    printf("pos error [m]    : mean %.4f  max %.4f  final %.4f\n",
           s->err_mean, s->err_max, s->err_final);
    printf("closed-loop cost : %.2f\n", s->jcl);
    printf("fallbacks        : %d/%d\n\n", s->n_fallback, nsim);
}

int main(int argc, char **argv)
{
    int nsim = argc > 1 ? atoi(argv[1]) : 400;

    as_full_ctrl fc;
    if (as_full_init(&fc)) { fprintf(stderr, "full init failed\n"); return 1; }
    loop_stats sf;
    if (run_loop(&fc, step_full, nsim, &sf)) return 1;
    print_stats("full NMPC", AS_NSTACK, nsim, &sf);
    as_full_free(&fc);

    as_ctrl rc;
    if (as_ctrl_init(&rc)) { fprintf(stderr, "reduced init failed\n"); return 1; }
    loop_stats sr;
    if (run_loop(&rc, step_reduced, nsim, &sr)) return 1;
    print_stats("reduced active-subspace NMPC", AS_NZ, nsim, &sr);
    as_ctrl_free(&rc);

    printf("=== controlled comparison (identical framework) ===\n");
    printf("speedup mean : %.2fx   p95 : %.2fx   max : %.2fx\n",
           sf.mean_ms / sr.mean_ms, sf.p95_ms / sr.p95_ms, sf.max_ms / sr.max_ms);
    printf("suboptimality: J_reduced / J_full = %.4f\n", sr.jcl / sf.jcl);
    return 0;
}
