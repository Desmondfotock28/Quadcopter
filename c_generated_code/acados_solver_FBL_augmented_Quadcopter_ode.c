/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific

#include "FBL_augmented_Quadcopter_ode_model/FBL_augmented_Quadcopter_ode_model.h"





#include "acados_solver_FBL_augmented_Quadcopter_ode.h"

#define NX     FBL_AUGMENTED_QUADCOPTER_ODE_NX
#define NZ     FBL_AUGMENTED_QUADCOPTER_ODE_NZ
#define NU     FBL_AUGMENTED_QUADCOPTER_ODE_NU
#define NP     FBL_AUGMENTED_QUADCOPTER_ODE_NP
#define NP_GLOBAL     FBL_AUGMENTED_QUADCOPTER_ODE_NP_GLOBAL
#define NY0    FBL_AUGMENTED_QUADCOPTER_ODE_NY0
#define NY     FBL_AUGMENTED_QUADCOPTER_ODE_NY
#define NYN    FBL_AUGMENTED_QUADCOPTER_ODE_NYN

#define NBX    FBL_AUGMENTED_QUADCOPTER_ODE_NBX
#define NBX0   FBL_AUGMENTED_QUADCOPTER_ODE_NBX0
#define NBU    FBL_AUGMENTED_QUADCOPTER_ODE_NBU
#define NG     FBL_AUGMENTED_QUADCOPTER_ODE_NG
#define NBXN   FBL_AUGMENTED_QUADCOPTER_ODE_NBXN
#define NGN    FBL_AUGMENTED_QUADCOPTER_ODE_NGN

#define NH     FBL_AUGMENTED_QUADCOPTER_ODE_NH
#define NHN    FBL_AUGMENTED_QUADCOPTER_ODE_NHN
#define NH0    FBL_AUGMENTED_QUADCOPTER_ODE_NH0
#define NPHI   FBL_AUGMENTED_QUADCOPTER_ODE_NPHI
#define NPHIN  FBL_AUGMENTED_QUADCOPTER_ODE_NPHIN
#define NPHI0  FBL_AUGMENTED_QUADCOPTER_ODE_NPHI0
#define NR     FBL_AUGMENTED_QUADCOPTER_ODE_NR

#define NS     FBL_AUGMENTED_QUADCOPTER_ODE_NS
#define NS0    FBL_AUGMENTED_QUADCOPTER_ODE_NS0
#define NSN    FBL_AUGMENTED_QUADCOPTER_ODE_NSN

#define NSBX   FBL_AUGMENTED_QUADCOPTER_ODE_NSBX
#define NSBU   FBL_AUGMENTED_QUADCOPTER_ODE_NSBU
#define NSH0   FBL_AUGMENTED_QUADCOPTER_ODE_NSH0
#define NSH    FBL_AUGMENTED_QUADCOPTER_ODE_NSH
#define NSHN   FBL_AUGMENTED_QUADCOPTER_ODE_NSHN
#define NSG    FBL_AUGMENTED_QUADCOPTER_ODE_NSG
#define NSPHI0 FBL_AUGMENTED_QUADCOPTER_ODE_NSPHI0
#define NSPHI  FBL_AUGMENTED_QUADCOPTER_ODE_NSPHI
#define NSPHIN FBL_AUGMENTED_QUADCOPTER_ODE_NSPHIN
#define NSGN   FBL_AUGMENTED_QUADCOPTER_ODE_NSGN
#define NSBXN  FBL_AUGMENTED_QUADCOPTER_ODE_NSBXN



// ** solver data **

FBL_augmented_Quadcopter_ode_solver_capsule * FBL_augmented_Quadcopter_ode_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(FBL_augmented_Quadcopter_ode_solver_capsule));
    FBL_augmented_Quadcopter_ode_solver_capsule *capsule = (FBL_augmented_Quadcopter_ode_solver_capsule *) capsule_mem;

    return capsule;
}


int FBL_augmented_Quadcopter_ode_acados_free_capsule(FBL_augmented_Quadcopter_ode_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int FBL_augmented_Quadcopter_ode_acados_create(FBL_augmented_Quadcopter_ode_solver_capsule* capsule)
{
    int N_shooting_intervals = FBL_AUGMENTED_QUADCOPTER_ODE_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return FBL_augmented_Quadcopter_ode_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int FBL_augmented_Quadcopter_ode_acados_update_time_steps(FBL_augmented_Quadcopter_ode_solver_capsule* capsule, int N, double* new_time_steps)
{

    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "FBL_augmented_Quadcopter_ode_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;

}

/**
 * Internal function for FBL_augmented_Quadcopter_ode_acados_create: step 1
 */
void FBL_augmented_Quadcopter_ode_acados_create_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->relaxed_ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->nlp_cost[0] = LINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = LINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = LINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = NO_REGULARIZE;

    nlp_solver_plan->globalization = FIXED_STEP;
}


static ocp_nlp_dims* FBL_augmented_Quadcopter_ode_acados_create_setup_dimensions(FBL_augmented_Quadcopter_ode_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 18
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;
    int* np  = intNp1mem + (N+1)*17;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
        np[i]     = NP;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    
    nbxe[0] = 17;
    
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "np", np);

    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "np_global", 0);
    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "n_global_data", 0);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);
    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for FBL_augmented_Quadcopter_ode_acados_create: step 3
 */
void FBL_augmented_Quadcopter_ode_acados_create_setup_functions(FBL_augmented_Quadcopter_ode_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_external_param_casadi_create(&capsule->__CAPSULE_FNC__, &ext_fun_opts); \
    } while(false)

    external_function_opts ext_fun_opts;
    external_function_opts_set_to_default(&ext_fun_opts);


    ext_fun_opts.external_workspace = true;




    // explicit ode
    capsule->expl_vde_forw = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_vde_forw[i], FBL_augmented_Quadcopter_ode_expl_vde_forw);
    }

    capsule->expl_ode_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_ode_fun[i], FBL_augmented_Quadcopter_ode_expl_ode_fun);
    }

    capsule->expl_vde_adj = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_vde_adj[i], FBL_augmented_Quadcopter_ode_expl_vde_adj);
    }



#undef MAP_CASADI_FNC
}


/**
 * Internal function for FBL_augmented_Quadcopter_ode_acados_create: step 5
 */
void FBL_augmented_Quadcopter_ode_acados_create_set_default_parameters(FBL_augmented_Quadcopter_ode_solver_capsule* capsule)
{

    // no parameters defined


    // no global parameters defined
}


/**
 * Internal function for FBL_augmented_Quadcopter_ode_acados_create: step 5
 */
void FBL_augmented_Quadcopter_ode_acados_setup_nlp_in(FBL_augmented_Quadcopter_ode_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    int tmp_int = 0;

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = capsule->nlp_in;
    /************************************************
    *  nlp_out
    ************************************************/
    ocp_nlp_out * nlp_out = capsule->nlp_out;

    // set up time_steps and cost_scaling

    if (new_time_steps)
    {
        // NOTE: this sets scaling and time_steps
        FBL_augmented_Quadcopter_ode_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {
        // set time_steps
    
        double time_step = 0.1;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
        }
        // set cost scaling
        double* cost_scaling = malloc((N+1)*sizeof(double));
        cost_scaling[0] = 0.1;
        cost_scaling[1] = 0.1;
        cost_scaling[2] = 0.1;
        cost_scaling[3] = 0.1;
        cost_scaling[4] = 0.1;
        cost_scaling[5] = 0.1;
        cost_scaling[6] = 0.1;
        cost_scaling[7] = 0.1;
        cost_scaling[8] = 0.1;
        cost_scaling[9] = 0.1;
        cost_scaling[10] = 1;
        for (int i = 0; i <= N; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &cost_scaling[i]);
        }
        free(cost_scaling);
    }



    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->expl_vde_forw[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_vde_adj", &capsule->expl_vde_adj[i]);
    }

    /**** Cost ****/
    double* yref_0 = calloc(NY0, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);

   double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[0+(NY0) * 0] = 10;
    W_0[1+(NY0) * 1] = 2;
    W_0[2+(NY0) * 2] = 2;
    W_0[3+(NY0) * 3] = 2;
    W_0[4+(NY0) * 4] = 10;
    W_0[5+(NY0) * 5] = 2;
    W_0[6+(NY0) * 6] = 2;
    W_0[7+(NY0) * 7] = 2;
    W_0[8+(NY0) * 8] = 10;
    W_0[9+(NY0) * 9] = 10;
    W_0[10+(NY0) * 10] = 10;
    W_0[11+(NY0) * 11] = 10;
    W_0[12+(NY0) * 12] = 10;
    W_0[13+(NY0) * 13] = 10;
    W_0[14+(NY0) * 14] = 0.001;
    W_0[15+(NY0) * 15] = 0.001;
    W_0[16+(NY0) * 16] = 0.001;
    W_0[17+(NY0) * 17] = 0.01;
    W_0[18+(NY0) * 18] = 0.01;
    W_0[19+(NY0) * 19] = 0.01;
    W_0[20+(NY0) * 20] = 0.01;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);
    double* Vx_0 = calloc(NY0*NX, sizeof(double));
    // change only the non-zero elements:
    Vx_0[0+(NY0) * 0] = 1;
    Vx_0[1+(NY0) * 1] = 1;
    Vx_0[2+(NY0) * 2] = 1;
    Vx_0[3+(NY0) * 3] = 1;
    Vx_0[4+(NY0) * 4] = 1;
    Vx_0[5+(NY0) * 5] = 1;
    Vx_0[6+(NY0) * 6] = 1;
    Vx_0[7+(NY0) * 7] = 1;
    Vx_0[8+(NY0) * 8] = 1;
    Vx_0[9+(NY0) * 9] = 1;
    Vx_0[10+(NY0) * 10] = 1;
    Vx_0[11+(NY0) * 11] = 1;
    Vx_0[12+(NY0) * 12] = 1;
    Vx_0[13+(NY0) * 13] = 1;
    Vx_0[14+(NY0) * 14] = 1;
    Vx_0[15+(NY0) * 15] = 1;
    Vx_0[16+(NY0) * 16] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vx", Vx_0);
    free(Vx_0);
    double* Vu_0 = calloc(NY0*NU, sizeof(double));
    // change only the non-zero elements:
    Vu_0[17+(NY0) * 0] = 1;
    Vu_0[18+(NY0) * 1] = 1;
    Vu_0[19+(NY0) * 2] = 1;
    Vu_0[20+(NY0) * 3] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vu", Vu_0);
    free(Vu_0);
    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(yref);
    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    W[0+(NY) * 0] = 10;
    W[1+(NY) * 1] = 2;
    W[2+(NY) * 2] = 2;
    W[3+(NY) * 3] = 2;
    W[4+(NY) * 4] = 10;
    W[5+(NY) * 5] = 2;
    W[6+(NY) * 6] = 2;
    W[7+(NY) * 7] = 2;
    W[8+(NY) * 8] = 10;
    W[9+(NY) * 9] = 10;
    W[10+(NY) * 10] = 10;
    W[11+(NY) * 11] = 10;
    W[12+(NY) * 12] = 10;
    W[13+(NY) * 13] = 10;
    W[14+(NY) * 14] = 0.001;
    W[15+(NY) * 15] = 0.001;
    W[16+(NY) * 16] = 0.001;
    W[17+(NY) * 17] = 0.01;
    W[18+(NY) * 18] = 0.01;
    W[19+(NY) * 19] = 0.01;
    W[20+(NY) * 20] = 0.01;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    free(W);
    double* Vx = calloc(NY*NX, sizeof(double));
    // change only the non-zero elements:
    Vx[0+(NY) * 0] = 1;
    Vx[1+(NY) * 1] = 1;
    Vx[2+(NY) * 2] = 1;
    Vx[3+(NY) * 3] = 1;
    Vx[4+(NY) * 4] = 1;
    Vx[5+(NY) * 5] = 1;
    Vx[6+(NY) * 6] = 1;
    Vx[7+(NY) * 7] = 1;
    Vx[8+(NY) * 8] = 1;
    Vx[9+(NY) * 9] = 1;
    Vx[10+(NY) * 10] = 1;
    Vx[11+(NY) * 11] = 1;
    Vx[12+(NY) * 12] = 1;
    Vx[13+(NY) * 13] = 1;
    Vx[14+(NY) * 14] = 1;
    Vx[15+(NY) * 15] = 1;
    Vx[16+(NY) * 16] = 1;
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vx", Vx);
    }
    free(Vx);

    
    double* Vu = calloc(NY*NU, sizeof(double));
    // change only the non-zero elements:
    Vu[17+(NY) * 0] = 1;
    Vu[18+(NY) * 1] = 1;
    Vu[19+(NY) * 2] = 1;
    Vu[20+(NY) * 3] = 1;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vu", Vu);
    }
    free(Vu);
    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);

    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    W_e[0+(NYN) * 0] = 196.238242363592;
    W_e[0+(NYN) * 1] = 172.73532671157798;
    W_e[0+(NYN) * 2] = 71.8170864919517;
    W_e[0+(NYN) * 3] = 7.0575216244102625;
    W_e[0+(NYN) * 4] = -0.0000000000001254923328840408;
    W_e[0+(NYN) * 5] = -0.0000000000002510885686008118;
    W_e[0+(NYN) * 6] = -0.000000000000054537395927477993;
    W_e[0+(NYN) * 7] = -0.000000000000024001540671046478;
    W_e[0+(NYN) * 8] = 0.00000000000032047034340499984;
    W_e[0+(NYN) * 9] = 0.0000000000009712887913980817;
    W_e[0+(NYN) * 10] = 0.0000000000006706995314482752;
    W_e[0+(NYN) * 11] = 0.00000000000005323296599333685;
    W_e[0+(NYN) * 12] = 0.00000000000003065408431864458;
    W_e[0+(NYN) * 13] = 0.000000000000035293907543213025;
    W_e[1+(NYN) * 0] = 172.73532671157798;
    W_e[1+(NYN) * 1] = 249.8821499166922;
    W_e[1+(NYN) * 2] = 126.69335797493667;
    W_e[1+(NYN) * 3] = 13.143804227731572;
    W_e[1+(NYN) * 4] = -0.0000000000002714355518217631;
    W_e[1+(NYN) * 5] = -0.0000000000003791150791892978;
    W_e[1+(NYN) * 6] = -0.00000000000006656231172485247;
    W_e[1+(NYN) * 7] = -0.00000000000002290121265341616;
    W_e[1+(NYN) * 8] = 0.0000000000006591663812535937;
    W_e[1+(NYN) * 9] = 0.0000000000018176865007853764;
    W_e[1+(NYN) * 10] = 0.0000000000012442383091143782;
    W_e[1+(NYN) * 11] = 0.00000000000009360288437072975;
    W_e[1+(NYN) * 12] = 0.000000000000045244573883346103;
    W_e[1+(NYN) * 13] = 0.00000000000004377860380740264;
    W_e[2+(NYN) * 0] = 71.8170864919517;
    W_e[2+(NYN) * 1] = 126.69335797493667;
    W_e[2+(NYN) * 2] = 98.24033896137604;
    W_e[2+(NYN) * 3] = 10.876452612891285;
    W_e[2+(NYN) * 4] = -0.00000000000005219961775941544;
    W_e[2+(NYN) * 5] = -0.000000000000044753777180336266;
    W_e[2+(NYN) * 6] = 0.00000000000009989416734648131;
    W_e[2+(NYN) * 7] = 0.000000000000007540771670883108;
    W_e[2+(NYN) * 8] = 0.0000000000004450302324121358;
    W_e[2+(NYN) * 9] = 0.0000000000011314797835642644;
    W_e[2+(NYN) * 10] = 0.0000000000007363173443957935;
    W_e[2+(NYN) * 11] = 0.00000000000005811833105591204;
    W_e[2+(NYN) * 12] = 0.00000000000002068953182563419;
    W_e[2+(NYN) * 13] = 0.000000000000025462859694899063;
    W_e[3+(NYN) * 0] = 7.0575216244102625;
    W_e[3+(NYN) * 1] = 13.143804227731572;
    W_e[3+(NYN) * 2] = 10.876452612891285;
    W_e[3+(NYN) * 3] = 3.980861147901267;
    W_e[3+(NYN) * 4] = 0.000000000000013206413930937986;
    W_e[3+(NYN) * 5] = 0.000000000000029223854272649755;
    W_e[3+(NYN) * 6] = 0.000000000000029221030843096555;
    W_e[3+(NYN) * 7] = 0.0000000000000024987784650324196;
    W_e[3+(NYN) * 8] = 0.00000000000004928743233225247;
    W_e[3+(NYN) * 9] = 0.00000000000012588051401266717;
    W_e[3+(NYN) * 10] = 0.00000000000008525341344901718;
    W_e[3+(NYN) * 11] = 0.000000000000011653125646329452;
    W_e[3+(NYN) * 12] = 0.000000000000003442459013090337;
    W_e[3+(NYN) * 13] = 0.000000000000001442461913022831;
    W_e[4+(NYN) * 0] = -0.0000000000001254923328840408;
    W_e[4+(NYN) * 1] = -0.0000000000002714355518217631;
    W_e[4+(NYN) * 2] = -0.00000000000005219961775941544;
    W_e[4+(NYN) * 3] = 0.000000000000013206413930937986;
    W_e[4+(NYN) * 4] = 196.23824236359116;
    W_e[4+(NYN) * 5] = 172.73532671157662;
    W_e[4+(NYN) * 6] = 71.81708649195059;
    W_e[4+(NYN) * 7] = 7.057521624410117;
    W_e[4+(NYN) * 8] = 0.000000000001492289248076775;
    W_e[4+(NYN) * 9] = 0.0000000000027605415963791295;
    W_e[4+(NYN) * 10] = 0.0000000000015046663026019653;
    W_e[4+(NYN) * 11] = 0.00000000000007885363501438339;
    W_e[4+(NYN) * 12] = 0.00000000000006400917425190471;
    W_e[4+(NYN) * 13] = 0.00000000000000896982569722482;
    W_e[5+(NYN) * 0] = -0.0000000000002510885686008118;
    W_e[5+(NYN) * 1] = -0.0000000000003791150791892978;
    W_e[5+(NYN) * 2] = -0.000000000000044753777180336266;
    W_e[5+(NYN) * 3] = 0.000000000000029223854272649755;
    W_e[5+(NYN) * 4] = 172.73532671157662;
    W_e[5+(NYN) * 5] = 249.8821499166903;
    W_e[5+(NYN) * 6] = 126.69335797493528;
    W_e[5+(NYN) * 7] = 13.143804227731378;
    W_e[5+(NYN) * 8] = 0.0000000000015184071789592586;
    W_e[5+(NYN) * 9] = 0.000000000003465781517027014;
    W_e[5+(NYN) * 10] = 0.0000000000021794137071956425;
    W_e[5+(NYN) * 11] = 0.00000000000011996311017797122;
    W_e[5+(NYN) * 12] = 0.0000000000000807820720308765;
    W_e[5+(NYN) * 13] = 0.000000000000024674886592622195;
    W_e[6+(NYN) * 0] = -0.000000000000054537395927477993;
    W_e[6+(NYN) * 1] = -0.00000000000006656231172485247;
    W_e[6+(NYN) * 2] = 0.00000000000009989416734648131;
    W_e[6+(NYN) * 3] = 0.000000000000029221030843096555;
    W_e[6+(NYN) * 4] = 71.81708649195059;
    W_e[6+(NYN) * 5] = 126.69335797493528;
    W_e[6+(NYN) * 6] = 98.2403389613752;
    W_e[6+(NYN) * 7] = 10.876452612891155;
    W_e[6+(NYN) * 8] = 0.0000000000006634359245412467;
    W_e[6+(NYN) * 9] = 0.000000000001778395819275857;
    W_e[6+(NYN) * 10] = 0.0000000000012307570866976633;
    W_e[6+(NYN) * 11] = 0.00000000000007548157430936968;
    W_e[6+(NYN) * 12] = 0.000000000000036182784831924;
    W_e[6+(NYN) * 13] = 0.00000000000002142597695958653;
    W_e[7+(NYN) * 0] = -0.000000000000024001540671046478;
    W_e[7+(NYN) * 1] = -0.00000000000002290121265341616;
    W_e[7+(NYN) * 2] = 0.000000000000007540771670883108;
    W_e[7+(NYN) * 3] = 0.0000000000000024987784650324196;
    W_e[7+(NYN) * 4] = 7.057521624410117;
    W_e[7+(NYN) * 5] = 13.143804227731378;
    W_e[7+(NYN) * 6] = 10.876452612891155;
    W_e[7+(NYN) * 7] = 3.980861147901259;
    W_e[7+(NYN) * 8] = 0.000000000000055381049340254736;
    W_e[7+(NYN) * 9] = 0.0000000000001520501599899603;
    W_e[7+(NYN) * 10] = 0.00000000000010619923856140637;
    W_e[7+(NYN) * 11] = 0.0000000000000005843263290557355;
    W_e[7+(NYN) * 12] = 0.0000000000000016466391177790969;
    W_e[7+(NYN) * 13] = 0.0000000000000033769258730712913;
    W_e[8+(NYN) * 0] = 0.00000000000032047034340499984;
    W_e[8+(NYN) * 1] = 0.0000000000006591663812535937;
    W_e[8+(NYN) * 2] = 0.0000000000004450302324121358;
    W_e[8+(NYN) * 3] = 0.00000000000004928743233225247;
    W_e[8+(NYN) * 4] = 0.000000000001492289248076775;
    W_e[8+(NYN) * 5] = 0.0000000000015184071789592586;
    W_e[8+(NYN) * 6] = 0.0000000000006634359245412467;
    W_e[8+(NYN) * 7] = 0.000000000000055381049340254736;
    W_e[8+(NYN) * 8] = 267.57972329229233;
    W_e[8+(NYN) * 9] = 294.61555542128116;
    W_e[8+(NYN) * 10] = 141.8798690914925;
    W_e[8+(NYN) * 11] = 12.31164989932678;
    W_e[8+(NYN) * 12] = 0.000000000001545490174906843;
    W_e[8+(NYN) * 13] = 0.0000000000001652943705607971;
    W_e[9+(NYN) * 0] = 0.0000000000009712887913980817;
    W_e[9+(NYN) * 1] = 0.0000000000018176865007853764;
    W_e[9+(NYN) * 2] = 0.0000000000011314797835642644;
    W_e[9+(NYN) * 3] = 0.00000000000012588051401266717;
    W_e[9+(NYN) * 4] = 0.0000000000027605415963791295;
    W_e[9+(NYN) * 5] = 0.000000000003465781517027014;
    W_e[9+(NYN) * 6] = 0.000000000001778395819275857;
    W_e[9+(NYN) * 7] = 0.0000000000001520501599899603;
    W_e[9+(NYN) * 8] = 294.61555542128116;
    W_e[9+(NYN) * 9] = 616.9900633386862;
    W_e[9+(NYN) * 10] = 353.14212431400284;
    W_e[9+(NYN) * 11] = 31.712313743401435;
    W_e[9+(NYN) * 12] = 0.0000000000024991026629247048;
    W_e[9+(NYN) * 13] = 0.00000000000028701762434205493;
    W_e[10+(NYN) * 0] = 0.0000000000006706995314482752;
    W_e[10+(NYN) * 1] = 0.0000000000012442383091143782;
    W_e[10+(NYN) * 2] = 0.0000000000007363173443957935;
    W_e[10+(NYN) * 3] = 0.00000000000008525341344901718;
    W_e[10+(NYN) * 4] = 0.0000000000015046663026019653;
    W_e[10+(NYN) * 5] = 0.0000000000021794137071956425;
    W_e[10+(NYN) * 6] = 0.0000000000012307570866976633;
    W_e[10+(NYN) * 7] = 0.00000000000010619923856140637;
    W_e[10+(NYN) * 8] = 141.8798690914925;
    W_e[10+(NYN) * 9] = 353.14212431400284;
    W_e[10+(NYN) * 10] = 350.9736381800651;
    W_e[10+(NYN) * 11] = 33.10080435808312;
    W_e[10+(NYN) * 12] = 0.0000000000017143742779030394;
    W_e[10+(NYN) * 13] = 0.00000000000021572034338067974;
    W_e[11+(NYN) * 0] = 0.00000000000005323296599333685;
    W_e[11+(NYN) * 1] = 0.00000000000009360288437072975;
    W_e[11+(NYN) * 2] = 0.00000000000005811833105591204;
    W_e[11+(NYN) * 3] = 0.000000000000011653125646329452;
    W_e[11+(NYN) * 4] = 0.00000000000007885363501438339;
    W_e[11+(NYN) * 5] = 0.00000000000011996311017797122;
    W_e[11+(NYN) * 6] = 0.00000000000007548157430936968;
    W_e[11+(NYN) * 7] = 0.0000000000000005843263290557355;
    W_e[11+(NYN) * 8] = 12.31164989932678;
    W_e[11+(NYN) * 9] = 31.712313743401435;
    W_e[11+(NYN) * 10] = 33.10080435808312;
    W_e[11+(NYN) * 11] = 14.157672324357698;
    W_e[11+(NYN) * 12] = 0.000000000000156861118720242;
    W_e[11+(NYN) * 13] = 0.000000000000027836437274325943;
    W_e[12+(NYN) * 0] = 0.00000000000003065408431864458;
    W_e[12+(NYN) * 1] = 0.000000000000045244573883346103;
    W_e[12+(NYN) * 2] = 0.00000000000002068953182563419;
    W_e[12+(NYN) * 3] = 0.000000000000003442459013090337;
    W_e[12+(NYN) * 4] = 0.00000000000006400917425190471;
    W_e[12+(NYN) * 5] = 0.0000000000000807820720308765;
    W_e[12+(NYN) * 6] = 0.000000000000036182784831924;
    W_e[12+(NYN) * 7] = 0.0000000000000016466391177790969;
    W_e[12+(NYN) * 8] = 0.000000000001545490174906843;
    W_e[12+(NYN) * 9] = 0.0000000000024991026629247048;
    W_e[12+(NYN) * 10] = 0.0000000000017143742779030394;
    W_e[12+(NYN) * 11] = 0.000000000000156861118720242;
    W_e[12+(NYN) * 12] = 115.99184840464154;
    W_e[12+(NYN) * 13] = 11.470952061395549;
    W_e[13+(NYN) * 0] = 0.000000000000035293907543213025;
    W_e[13+(NYN) * 1] = 0.00000000000004377860380740264;
    W_e[13+(NYN) * 2] = 0.000000000000025462859694899063;
    W_e[13+(NYN) * 3] = 0.000000000000001442461913022831;
    W_e[13+(NYN) * 4] = 0.00000000000000896982569722482;
    W_e[13+(NYN) * 5] = 0.000000000000024674886592622195;
    W_e[13+(NYN) * 6] = 0.00000000000002142597695958653;
    W_e[13+(NYN) * 7] = 0.0000000000000033769258730712913;
    W_e[13+(NYN) * 8] = 0.0000000000001652943705607971;
    W_e[13+(NYN) * 9] = 0.00000000000028701762434205493;
    W_e[13+(NYN) * 10] = 0.00000000000021572034338067974;
    W_e[13+(NYN) * 11] = 0.000000000000027836437274325943;
    W_e[13+(NYN) * 12] = 11.470952061395549;
    W_e[13+(NYN) * 13] = 12.158274119483668;
    W_e[14+(NYN) * 14] = 0.001;
    W_e[15+(NYN) * 15] = 0.001;
    W_e[16+(NYN) * 16] = 0.001;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);
    double* Vx_e = calloc(NYN*NX, sizeof(double));
    // change only the non-zero elements:
    Vx_e[0+(NYN) * 0] = 1;
    Vx_e[1+(NYN) * 1] = 1;
    Vx_e[2+(NYN) * 2] = 1;
    Vx_e[3+(NYN) * 3] = 1;
    Vx_e[4+(NYN) * 4] = 1;
    Vx_e[5+(NYN) * 5] = 1;
    Vx_e[6+(NYN) * 6] = 1;
    Vx_e[7+(NYN) * 7] = 1;
    Vx_e[8+(NYN) * 8] = 1;
    Vx_e[9+(NYN) * 9] = 1;
    Vx_e[10+(NYN) * 10] = 1;
    Vx_e[11+(NYN) * 11] = 1;
    Vx_e[12+(NYN) * 12] = 1;
    Vx_e[13+(NYN) * 13] = 1;
    Vx_e[14+(NYN) * 14] = 1;
    Vx_e[15+(NYN) * 15] = 1;
    Vx_e[16+(NYN) * 16] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Vx", Vx_e);
    free(Vx_e);







    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;
    idxbx0[13] = 13;
    idxbx0[14] = 14;
    idxbx0[15] = 15;
    idxbx0[16] = 16;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(17 * sizeof(int));
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    idxbxe_0[10] = 10;
    idxbxe_0[11] = 11;
    idxbxe_0[12] = 12;
    idxbxe_0[13] = 13;
    idxbxe_0[14] = 14;
    idxbxe_0[15] = 15;
    idxbxe_0[16] = 16;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);








    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    idxbu[3] = 3;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    lbu[0] = -1;
    ubu[0] = 1;
    lbu[1] = -0.05;
    ubu[1] = 0.05;
    lbu[2] = -0.05;
    ubu[2] = 0.05;
    lbu[3] = -0.05;
    ubu[3] = 0.05;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);















    /* terminal constraints */













}


static void FBL_augmented_Quadcopter_ode_acados_create_set_opts(FBL_augmented_Quadcopter_ode_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/



    int fixed_hess = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "fixed_hess", &fixed_hess);

    double globalization_fixed_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization_fixed_step_length", &globalization_fixed_step_length);




    int with_solution_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params", &with_solution_sens_wrt_params);

    int with_value_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_value_sens_wrt_params", &with_value_sens_wrt_params);

    double solution_sens_qp_t_lam_min = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "solution_sens_qp_t_lam_min", &solution_sens_qp_t_lam_min);

    int globalization_full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_full_step_dual", &globalization_full_step_dual);

    // set collocation type (relevant for implicit integrators)
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_collocation_type", &collocation_type);

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 1;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps);

    // set up sim_method_num_stages
    // all sim_method_num_stages are identical
    int sim_method_num_stages = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);

    double newton_tol_val = 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_tol", &newton_tol_val);

    // set up sim_method_jac_reuse
    bool tmp_bool = (bool) 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;const int qp_solver_cond_N_ori = 10;
    qp_solver_cond_N = N < qp_solver_cond_N_ori ? N : qp_solver_cond_N_ori; // use the minimum value here
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);

    bool store_iterates = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "store_iterates", &store_iterates);
    int log_primal_step_norm = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_primal_step_norm", &log_primal_step_norm);

    int log_dual_step_norm = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_dual_step_norm", &log_dual_step_norm);

    double nlp_solver_tol_min_step_norm = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_min_step_norm", &nlp_solver_tol_min_step_norm);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_t0_init = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_t0_init", &qp_solver_t0_init);




    // set SQP specific options
    double nlp_solver_tol_stat = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 100;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "max_iter", &nlp_solver_max_iter);

    // set options for adaptive Levenberg-Marquardt Update
    bool with_adaptive_levenberg_marquardt = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_adaptive_levenberg_marquardt", &with_adaptive_levenberg_marquardt);

    double adaptive_levenberg_marquardt_lam = 5;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_lam", &adaptive_levenberg_marquardt_lam);

    double adaptive_levenberg_marquardt_mu_min = 0.0000000000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu_min", &adaptive_levenberg_marquardt_mu_min);

    double adaptive_levenberg_marquardt_mu0 = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu0", &adaptive_levenberg_marquardt_mu0);

    double adaptive_levenberg_marquardt_obj_scalar = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_obj_scalar", &adaptive_levenberg_marquardt_obj_scalar);

    bool eval_residual_at_max_iter = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "eval_residual_at_max_iter", &eval_residual_at_max_iter);

    // QP scaling
    double qpscaling_ub_max_abs_eig = 100000;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_ub_max_abs_eig", &qpscaling_ub_max_abs_eig);

    double qpscaling_lb_norm_inf_grad_obj = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_lb_norm_inf_grad_obj", &qpscaling_lb_norm_inf_grad_obj);

    qpscaling_scale_objective_type qpscaling_scale_objective = NO_OBJECTIVE_SCALING;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_scale_objective", &qpscaling_scale_objective);

    ocp_nlp_qpscaling_constraint_type qpscaling_scale_constraints = NO_CONSTRAINT_SCALING;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_scale_constraints", &qpscaling_scale_constraints);

    bool with_anderson_acceleration = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_anderson_acceleration", &with_anderson_acceleration);

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);



    int print_level = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;
}


/**
 * Internal function for FBL_augmented_Quadcopter_ode_acados_create: step 7
 */
void FBL_augmented_Quadcopter_ode_acados_set_nlp_out(FBL_augmented_Quadcopter_ode_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for FBL_augmented_Quadcopter_ode_acados_create: step 9
 */
int FBL_augmented_Quadcopter_ode_acados_create_precompute(FBL_augmented_Quadcopter_ode_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int FBL_augmented_Quadcopter_ode_acados_create_with_discretization(FBL_augmented_Quadcopter_ode_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != FBL_AUGMENTED_QUADCOPTER_ODE_N && !new_time_steps) {
        fprintf(stderr, "FBL_augmented_Quadcopter_ode_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, FBL_AUGMENTED_QUADCOPTER_ODE_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    FBL_augmented_Quadcopter_ode_acados_create_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 2) create and set dimensions
    capsule->nlp_dims = FBL_augmented_Quadcopter_ode_acados_create_setup_dimensions(capsule);

    // 3) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    FBL_augmented_Quadcopter_ode_acados_create_set_opts(capsule);

    // 4) create and set nlp_out
    // 4.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 4.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    FBL_augmented_Quadcopter_ode_acados_set_nlp_out(capsule);

    // 5) create nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);

    // 6) setup functions, nlp_in and default parameters
    FBL_augmented_Quadcopter_ode_acados_create_setup_functions(capsule);
    FBL_augmented_Quadcopter_ode_acados_setup_nlp_in(capsule, N, new_time_steps);
    FBL_augmented_Quadcopter_ode_acados_create_set_default_parameters(capsule);

    // 7) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);


    // 8) do precomputations
    int status = FBL_augmented_Quadcopter_ode_acados_create_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int FBL_augmented_Quadcopter_ode_acados_update_qp_solver_cond_N(FBL_augmented_Quadcopter_ode_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from FBL_augmented_Quadcopter_ode_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);

    // -> 9) do precomputations
    int status = FBL_augmented_Quadcopter_ode_acados_create_precompute(capsule);
    return status;
}


int FBL_augmented_Quadcopter_ode_acados_reset(FBL_augmented_Quadcopter_ode_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+2*NS0+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NH0+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "pi", buffer);
        }
    }
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    free(buffer);
    return 0;
}




int FBL_augmented_Quadcopter_ode_acados_update_params(FBL_augmented_Quadcopter_ode_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 0;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    ocp_nlp_in_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, "parameter_values", p);

    return solver_status;
}


int FBL_augmented_Quadcopter_ode_acados_update_params_sparse(FBL_augmented_Quadcopter_ode_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    ocp_nlp_in_set_params_sparse(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, idx, p, n_update);

    return 0;
}


int FBL_augmented_Quadcopter_ode_acados_set_p_global_and_precompute_dependencies(FBL_augmented_Quadcopter_ode_solver_capsule* capsule, double* data, int data_len)
{

    // printf("No global_data, FBL_augmented_Quadcopter_ode_acados_set_p_global_and_precompute_dependencies does nothing.\n");
    return 0;
}




int FBL_augmented_Quadcopter_ode_acados_solve(FBL_augmented_Quadcopter_ode_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}



int FBL_augmented_Quadcopter_ode_acados_setup_qp_matrices_and_factorize(FBL_augmented_Quadcopter_ode_solver_capsule* capsule)
{
    int solver_status = ocp_nlp_setup_qp_matrices_and_factorize(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}






int FBL_augmented_Quadcopter_ode_acados_free(FBL_augmented_Quadcopter_ode_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_external_param_casadi_free(&capsule->expl_vde_forw[i]);
        external_function_external_param_casadi_free(&capsule->expl_ode_fun[i]);
        external_function_external_param_casadi_free(&capsule->expl_vde_adj[i]);
    }
    free(capsule->expl_vde_adj);
    free(capsule->expl_vde_forw);
    free(capsule->expl_ode_fun);

    // cost

    // constraints



    return 0;
}


void FBL_augmented_Quadcopter_ode_acados_print_stats(FBL_augmented_Quadcopter_ode_solver_capsule* capsule)
{
    int nlp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_solver, "nlp_iter", &nlp_iter);
    ocp_nlp_get(capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_solver, "stat_m", &stat_m);


    int stat_n_max = 16;
    if (stat_n > stat_n_max)
    {
        printf("stat_n_max = %d is too small, increase it in the template!\n", stat_n_max);
        exit(1);
    }
    double stat[1600];
    ocp_nlp_get(capsule->nlp_solver, "statistics", stat);

    int nrow = nlp_iter+1 < stat_m ? nlp_iter+1 : stat_m;


    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j == 5 || j == 6)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}

int FBL_augmented_Quadcopter_ode_acados_custom_update(FBL_augmented_Quadcopter_ode_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *FBL_augmented_Quadcopter_ode_acados_get_nlp_in(FBL_augmented_Quadcopter_ode_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *FBL_augmented_Quadcopter_ode_acados_get_nlp_out(FBL_augmented_Quadcopter_ode_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *FBL_augmented_Quadcopter_ode_acados_get_sens_out(FBL_augmented_Quadcopter_ode_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *FBL_augmented_Quadcopter_ode_acados_get_nlp_solver(FBL_augmented_Quadcopter_ode_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *FBL_augmented_Quadcopter_ode_acados_get_nlp_config(FBL_augmented_Quadcopter_ode_solver_capsule* capsule) { return capsule->nlp_config; }
void *FBL_augmented_Quadcopter_ode_acados_get_nlp_opts(FBL_augmented_Quadcopter_ode_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *FBL_augmented_Quadcopter_ode_acados_get_nlp_dims(FBL_augmented_Quadcopter_ode_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *FBL_augmented_Quadcopter_ode_acados_get_nlp_plan(FBL_augmented_Quadcopter_ode_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
