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

#include "kinodynamic_load_3cables_model/kinodynamic_load_3cables_model.h"


#include "kinodynamic_load_3cables_constraints/kinodynamic_load_3cables_constraints.h"
#include "kinodynamic_load_3cables_cost/kinodynamic_load_3cables_cost.h"



#include "acados_solver_kinodynamic_load_3cables.h"

#define NX     KINODYNAMIC_LOAD_3CABLES_NX
#define NZ     KINODYNAMIC_LOAD_3CABLES_NZ
#define NU     KINODYNAMIC_LOAD_3CABLES_NU
#define NP     KINODYNAMIC_LOAD_3CABLES_NP
#define NP_GLOBAL     KINODYNAMIC_LOAD_3CABLES_NP_GLOBAL
#define NY0    KINODYNAMIC_LOAD_3CABLES_NY0
#define NY     KINODYNAMIC_LOAD_3CABLES_NY
#define NYN    KINODYNAMIC_LOAD_3CABLES_NYN

#define NBX    KINODYNAMIC_LOAD_3CABLES_NBX
#define NBX0   KINODYNAMIC_LOAD_3CABLES_NBX0
#define NBU    KINODYNAMIC_LOAD_3CABLES_NBU
#define NG     KINODYNAMIC_LOAD_3CABLES_NG
#define NBXN   KINODYNAMIC_LOAD_3CABLES_NBXN
#define NGN    KINODYNAMIC_LOAD_3CABLES_NGN

#define NH     KINODYNAMIC_LOAD_3CABLES_NH
#define NHN    KINODYNAMIC_LOAD_3CABLES_NHN
#define NH0    KINODYNAMIC_LOAD_3CABLES_NH0
#define NPHI   KINODYNAMIC_LOAD_3CABLES_NPHI
#define NPHIN  KINODYNAMIC_LOAD_3CABLES_NPHIN
#define NPHI0  KINODYNAMIC_LOAD_3CABLES_NPHI0
#define NR     KINODYNAMIC_LOAD_3CABLES_NR

#define NS     KINODYNAMIC_LOAD_3CABLES_NS
#define NS0    KINODYNAMIC_LOAD_3CABLES_NS0
#define NSN    KINODYNAMIC_LOAD_3CABLES_NSN

#define NSBX   KINODYNAMIC_LOAD_3CABLES_NSBX
#define NSBU   KINODYNAMIC_LOAD_3CABLES_NSBU
#define NSH0   KINODYNAMIC_LOAD_3CABLES_NSH0
#define NSH    KINODYNAMIC_LOAD_3CABLES_NSH
#define NSHN   KINODYNAMIC_LOAD_3CABLES_NSHN
#define NSG    KINODYNAMIC_LOAD_3CABLES_NSG
#define NSPHI0 KINODYNAMIC_LOAD_3CABLES_NSPHI0
#define NSPHI  KINODYNAMIC_LOAD_3CABLES_NSPHI
#define NSPHIN KINODYNAMIC_LOAD_3CABLES_NSPHIN
#define NSGN   KINODYNAMIC_LOAD_3CABLES_NSGN
#define NSBXN  KINODYNAMIC_LOAD_3CABLES_NSBXN



// ** solver data **

kinodynamic_load_3cables_solver_capsule * kinodynamic_load_3cables_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(kinodynamic_load_3cables_solver_capsule));
    kinodynamic_load_3cables_solver_capsule *capsule = (kinodynamic_load_3cables_solver_capsule *) capsule_mem;

    return capsule;
}


int kinodynamic_load_3cables_acados_free_capsule(kinodynamic_load_3cables_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int kinodynamic_load_3cables_acados_create(kinodynamic_load_3cables_solver_capsule* capsule)
{
    int N_shooting_intervals = KINODYNAMIC_LOAD_3CABLES_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return kinodynamic_load_3cables_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int kinodynamic_load_3cables_acados_update_time_steps(kinodynamic_load_3cables_solver_capsule* capsule, int N, double* new_time_steps)
{

    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "kinodynamic_load_3cables_acados_update_time_steps: given number of time steps (= %d) " \
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
 * Internal function for kinodynamic_load_3cables_acados_create: step 1
 */
void kinodynamic_load_3cables_acados_create_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP_RTI;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = FULL_CONDENSING_HPIPM;
    nlp_solver_plan->relaxed_ocp_qp_solver_plan.qp_solver = FULL_CONDENSING_HPIPM;
    nlp_solver_plan->nlp_cost[0] = NONLINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = NONLINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = NONLINEAR_LS;

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


static ocp_nlp_dims* kinodynamic_load_3cables_acados_create_setup_dimensions(kinodynamic_load_3cables_solver_capsule* capsule)
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
    
    nbxe[0] = 55;
    
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
 * Internal function for kinodynamic_load_3cables_acados_create: step 3
 */
void kinodynamic_load_3cables_acados_create_setup_functions(kinodynamic_load_3cables_solver_capsule* capsule)
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
    if (N > 0)
    {
        // constraints.constr_type == "BGH" and dims.nh > 0
        capsule->nl_constr_h_fun_jac = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun_jac[i], kinodynamic_load_3cables_constr_h_fun_jac_uxt_zt);
        }
        capsule->nl_constr_h_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun[i], kinodynamic_load_3cables_constr_h_fun);
        }
    
        // nonlinear least squares function
        MAP_CASADI_FNC(cost_y_0_fun, kinodynamic_load_3cables_cost_y_0_fun);
        MAP_CASADI_FNC(cost_y_0_fun_jac_ut_xt, kinodynamic_load_3cables_cost_y_0_fun_jac_ut_xt);



    
        // explicit ode
        capsule->expl_vde_forw = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++) {
            MAP_CASADI_FNC(expl_vde_forw[i], kinodynamic_load_3cables_expl_vde_forw);
        }

        capsule->expl_ode_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++) {
            MAP_CASADI_FNC(expl_ode_fun[i], kinodynamic_load_3cables_expl_ode_fun);
        }

        capsule->expl_vde_adj = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++) {
            MAP_CASADI_FNC(expl_vde_adj[i], kinodynamic_load_3cables_expl_vde_adj);
        }

    
        // nonlinear least squares cost
        capsule->cost_y_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(cost_y_fun[i], kinodynamic_load_3cables_cost_y_fun);
        }

        capsule->cost_y_fun_jac_ut_xt = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(cost_y_fun_jac_ut_xt[i], kinodynamic_load_3cables_cost_y_fun_jac_ut_xt);
        }
    } // N > 0
    // nonlinear least square function
    MAP_CASADI_FNC(cost_y_e_fun, kinodynamic_load_3cables_cost_y_e_fun);
    MAP_CASADI_FNC(cost_y_e_fun_jac_ut_xt, kinodynamic_load_3cables_cost_y_e_fun_jac_ut_xt);

#undef MAP_CASADI_FNC
}


/**
 * Internal function for kinodynamic_load_3cables_acados_create: step 5
 */
void kinodynamic_load_3cables_acados_create_set_default_parameters(kinodynamic_load_3cables_solver_capsule* capsule)
{

    // no parameters defined


    // no global parameters defined
}


/**
 * Internal function for kinodynamic_load_3cables_acados_create: step 5
 */
void kinodynamic_load_3cables_acados_setup_nlp_in(kinodynamic_load_3cables_solver_capsule* capsule, const int N, double* new_time_steps)
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
        kinodynamic_load_3cables_acados_update_time_steps(capsule, N, new_time_steps);
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
        cost_scaling[10] = 0.1;
        cost_scaling[11] = 0.1;
        cost_scaling[12] = 0.1;
        cost_scaling[13] = 0.1;
        cost_scaling[14] = 0.1;
        cost_scaling[15] = 0.1;
        cost_scaling[16] = 0.1;
        cost_scaling[17] = 0.1;
        cost_scaling[18] = 0.1;
        cost_scaling[19] = 0.1;
        cost_scaling[20] = 1;
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
    yref_0[0] = 5;
    yref_0[6] = 1;
    yref_0[15] = -1;
    yref_0[25] = 3.27;
    yref_0[29] = -1;
    yref_0[39] = 3.27;
    yref_0[43] = -1;
    yref_0[53] = 3.27;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);

   double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[0+(NY0) * 0] = 50;
    W_0[1+(NY0) * 1] = 50;
    W_0[2+(NY0) * 2] = 50;
    W_0[3+(NY0) * 3] = 30;
    W_0[4+(NY0) * 4] = 30;
    W_0[5+(NY0) * 5] = 30;
    W_0[6+(NY0) * 6] = 400;
    W_0[7+(NY0) * 7] = 400;
    W_0[8+(NY0) * 8] = 400;
    W_0[9+(NY0) * 9] = 400;
    W_0[10+(NY0) * 10] = 20;
    W_0[11+(NY0) * 11] = 20;
    W_0[12+(NY0) * 12] = 20;
    W_0[13+(NY0) * 13] = 10;
    W_0[14+(NY0) * 14] = 10;
    W_0[15+(NY0) * 15] = 10;
    W_0[16+(NY0) * 16] = 10;
    W_0[17+(NY0) * 17] = 10;
    W_0[18+(NY0) * 18] = 10;
    W_0[19+(NY0) * 19] = 10;
    W_0[20+(NY0) * 20] = 10;
    W_0[21+(NY0) * 21] = 10;
    W_0[22+(NY0) * 22] = 10;
    W_0[23+(NY0) * 23] = 10;
    W_0[24+(NY0) * 24] = 10;
    W_0[25+(NY0) * 25] = 10;
    W_0[26+(NY0) * 26] = 10;
    W_0[27+(NY0) * 27] = 10;
    W_0[28+(NY0) * 28] = 10;
    W_0[29+(NY0) * 29] = 10;
    W_0[30+(NY0) * 30] = 10;
    W_0[31+(NY0) * 31] = 10;
    W_0[32+(NY0) * 32] = 10;
    W_0[33+(NY0) * 33] = 10;
    W_0[34+(NY0) * 34] = 10;
    W_0[35+(NY0) * 35] = 10;
    W_0[36+(NY0) * 36] = 10;
    W_0[37+(NY0) * 37] = 10;
    W_0[38+(NY0) * 38] = 10;
    W_0[39+(NY0) * 39] = 10;
    W_0[40+(NY0) * 40] = 10;
    W_0[41+(NY0) * 41] = 10;
    W_0[42+(NY0) * 42] = 10;
    W_0[43+(NY0) * 43] = 10;
    W_0[44+(NY0) * 44] = 10;
    W_0[45+(NY0) * 45] = 10;
    W_0[46+(NY0) * 46] = 10;
    W_0[47+(NY0) * 47] = 10;
    W_0[48+(NY0) * 48] = 10;
    W_0[49+(NY0) * 49] = 10;
    W_0[50+(NY0) * 50] = 10;
    W_0[51+(NY0) * 51] = 10;
    W_0[52+(NY0) * 52] = 10;
    W_0[53+(NY0) * 53] = 10;
    W_0[54+(NY0) * 54] = 10;
    W_0[55+(NY0) * 55] = 2;
    W_0[56+(NY0) * 56] = 2;
    W_0[57+(NY0) * 57] = 2;
    W_0[58+(NY0) * 58] = 0.5;
    W_0[59+(NY0) * 59] = 2;
    W_0[60+(NY0) * 60] = 2;
    W_0[61+(NY0) * 61] = 2;
    W_0[62+(NY0) * 62] = 0.5;
    W_0[63+(NY0) * 63] = 2;
    W_0[64+(NY0) * 64] = 2;
    W_0[65+(NY0) * 65] = 2;
    W_0[66+(NY0) * 66] = 0.5;
    W_0[67+(NY0) * 67] = 1000;
    W_0[68+(NY0) * 68] = 1000;
    W_0[69+(NY0) * 69] = 1000;
    W_0[70+(NY0) * 70] = 1000;
    W_0[71+(NY0) * 71] = 1000;
    W_0[72+(NY0) * 72] = 1000;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);
    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:
    yref[0] = 5;
    yref[6] = 1;
    yref[15] = -1;
    yref[25] = 3.27;
    yref[29] = -1;
    yref[39] = 3.27;
    yref[43] = -1;
    yref[53] = 3.27;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(yref);
    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    W[0+(NY) * 0] = 50;
    W[1+(NY) * 1] = 50;
    W[2+(NY) * 2] = 50;
    W[3+(NY) * 3] = 30;
    W[4+(NY) * 4] = 30;
    W[5+(NY) * 5] = 30;
    W[6+(NY) * 6] = 400;
    W[7+(NY) * 7] = 400;
    W[8+(NY) * 8] = 400;
    W[9+(NY) * 9] = 400;
    W[10+(NY) * 10] = 20;
    W[11+(NY) * 11] = 20;
    W[12+(NY) * 12] = 20;
    W[13+(NY) * 13] = 10;
    W[14+(NY) * 14] = 10;
    W[15+(NY) * 15] = 10;
    W[16+(NY) * 16] = 10;
    W[17+(NY) * 17] = 10;
    W[18+(NY) * 18] = 10;
    W[19+(NY) * 19] = 10;
    W[20+(NY) * 20] = 10;
    W[21+(NY) * 21] = 10;
    W[22+(NY) * 22] = 10;
    W[23+(NY) * 23] = 10;
    W[24+(NY) * 24] = 10;
    W[25+(NY) * 25] = 10;
    W[26+(NY) * 26] = 10;
    W[27+(NY) * 27] = 10;
    W[28+(NY) * 28] = 10;
    W[29+(NY) * 29] = 10;
    W[30+(NY) * 30] = 10;
    W[31+(NY) * 31] = 10;
    W[32+(NY) * 32] = 10;
    W[33+(NY) * 33] = 10;
    W[34+(NY) * 34] = 10;
    W[35+(NY) * 35] = 10;
    W[36+(NY) * 36] = 10;
    W[37+(NY) * 37] = 10;
    W[38+(NY) * 38] = 10;
    W[39+(NY) * 39] = 10;
    W[40+(NY) * 40] = 10;
    W[41+(NY) * 41] = 10;
    W[42+(NY) * 42] = 10;
    W[43+(NY) * 43] = 10;
    W[44+(NY) * 44] = 10;
    W[45+(NY) * 45] = 10;
    W[46+(NY) * 46] = 10;
    W[47+(NY) * 47] = 10;
    W[48+(NY) * 48] = 10;
    W[49+(NY) * 49] = 10;
    W[50+(NY) * 50] = 10;
    W[51+(NY) * 51] = 10;
    W[52+(NY) * 52] = 10;
    W[53+(NY) * 53] = 10;
    W[54+(NY) * 54] = 10;
    W[55+(NY) * 55] = 2;
    W[56+(NY) * 56] = 2;
    W[57+(NY) * 57] = 2;
    W[58+(NY) * 58] = 0.5;
    W[59+(NY) * 59] = 2;
    W[60+(NY) * 60] = 2;
    W[61+(NY) * 61] = 2;
    W[62+(NY) * 62] = 0.5;
    W[63+(NY) * 63] = 2;
    W[64+(NY) * 64] = 2;
    W[65+(NY) * 65] = 2;
    W[66+(NY) * 66] = 0.5;
    W[67+(NY) * 67] = 1000;
    W[68+(NY) * 68] = 1000;
    W[69+(NY) * 69] = 1000;
    W[70+(NY) * 70] = 1000;
    W[71+(NY) * 71] = 1000;
    W[72+(NY) * 72] = 1000;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    free(W);
    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    yref_e[0] = 5;
    yref_e[6] = 1;
    yref_e[15] = -1;
    yref_e[25] = 3.27;
    yref_e[29] = -1;
    yref_e[39] = 3.27;
    yref_e[43] = -1;
    yref_e[53] = 3.27;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);

    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    W_e[0+(NYN) * 0] = 1000;
    W_e[1+(NYN) * 1] = 1000;
    W_e[2+(NYN) * 2] = 1000;
    W_e[3+(NYN) * 3] = 600;
    W_e[4+(NYN) * 4] = 600;
    W_e[5+(NYN) * 5] = 600;
    W_e[6+(NYN) * 6] = 8000;
    W_e[7+(NYN) * 7] = 8000;
    W_e[8+(NYN) * 8] = 8000;
    W_e[9+(NYN) * 9] = 8000;
    W_e[10+(NYN) * 10] = 400;
    W_e[11+(NYN) * 11] = 400;
    W_e[12+(NYN) * 12] = 400;
    W_e[13+(NYN) * 13] = 200;
    W_e[14+(NYN) * 14] = 200;
    W_e[15+(NYN) * 15] = 200;
    W_e[16+(NYN) * 16] = 200;
    W_e[17+(NYN) * 17] = 200;
    W_e[18+(NYN) * 18] = 200;
    W_e[19+(NYN) * 19] = 200;
    W_e[20+(NYN) * 20] = 200;
    W_e[21+(NYN) * 21] = 200;
    W_e[22+(NYN) * 22] = 200;
    W_e[23+(NYN) * 23] = 200;
    W_e[24+(NYN) * 24] = 200;
    W_e[25+(NYN) * 25] = 200;
    W_e[26+(NYN) * 26] = 200;
    W_e[27+(NYN) * 27] = 200;
    W_e[28+(NYN) * 28] = 200;
    W_e[29+(NYN) * 29] = 200;
    W_e[30+(NYN) * 30] = 200;
    W_e[31+(NYN) * 31] = 200;
    W_e[32+(NYN) * 32] = 200;
    W_e[33+(NYN) * 33] = 200;
    W_e[34+(NYN) * 34] = 200;
    W_e[35+(NYN) * 35] = 200;
    W_e[36+(NYN) * 36] = 200;
    W_e[37+(NYN) * 37] = 200;
    W_e[38+(NYN) * 38] = 200;
    W_e[39+(NYN) * 39] = 200;
    W_e[40+(NYN) * 40] = 200;
    W_e[41+(NYN) * 41] = 200;
    W_e[42+(NYN) * 42] = 200;
    W_e[43+(NYN) * 43] = 200;
    W_e[44+(NYN) * 44] = 200;
    W_e[45+(NYN) * 45] = 200;
    W_e[46+(NYN) * 46] = 200;
    W_e[47+(NYN) * 47] = 200;
    W_e[48+(NYN) * 48] = 200;
    W_e[49+(NYN) * 49] = 200;
    W_e[50+(NYN) * 50] = 200;
    W_e[51+(NYN) * 51] = 200;
    W_e[52+(NYN) * 52] = 200;
    W_e[53+(NYN) * 53] = 200;
    W_e[54+(NYN) * 54] = 200;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nls_y_fun", &capsule->cost_y_0_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nls_y_fun_jac", &capsule->cost_y_0_fun_jac_ut_xt);
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nls_y_fun", &capsule->cost_y_fun[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nls_y_fun_jac", &capsule->cost_y_fun_jac_ut_xt[i-1]);
    }
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nls_y_fun", &capsule->cost_y_e_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nls_y_fun_jac", &capsule->cost_y_e_fun_jac_ut_xt);







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
    idxbx0[17] = 17;
    idxbx0[18] = 18;
    idxbx0[19] = 19;
    idxbx0[20] = 20;
    idxbx0[21] = 21;
    idxbx0[22] = 22;
    idxbx0[23] = 23;
    idxbx0[24] = 24;
    idxbx0[25] = 25;
    idxbx0[26] = 26;
    idxbx0[27] = 27;
    idxbx0[28] = 28;
    idxbx0[29] = 29;
    idxbx0[30] = 30;
    idxbx0[31] = 31;
    idxbx0[32] = 32;
    idxbx0[33] = 33;
    idxbx0[34] = 34;
    idxbx0[35] = 35;
    idxbx0[36] = 36;
    idxbx0[37] = 37;
    idxbx0[38] = 38;
    idxbx0[39] = 39;
    idxbx0[40] = 40;
    idxbx0[41] = 41;
    idxbx0[42] = 42;
    idxbx0[43] = 43;
    idxbx0[44] = 44;
    idxbx0[45] = 45;
    idxbx0[46] = 46;
    idxbx0[47] = 47;
    idxbx0[48] = 48;
    idxbx0[49] = 49;
    idxbx0[50] = 50;
    idxbx0[51] = 51;
    idxbx0[52] = 52;
    idxbx0[53] = 53;
    idxbx0[54] = 54;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:
    lbx0[6] = 1;
    ubx0[6] = 1;
    lbx0[15] = -1;
    ubx0[15] = -1;
    lbx0[25] = 3.27;
    ubx0[25] = 3.27;
    lbx0[29] = -1;
    ubx0[29] = -1;
    lbx0[39] = 3.27;
    ubx0[39] = 3.27;
    lbx0[43] = -1;
    ubx0[43] = -1;
    lbx0[53] = 3.27;
    ubx0[53] = 3.27;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(55 * sizeof(int));
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
    idxbxe_0[17] = 17;
    idxbxe_0[18] = 18;
    idxbxe_0[19] = 19;
    idxbxe_0[20] = 20;
    idxbxe_0[21] = 21;
    idxbxe_0[22] = 22;
    idxbxe_0[23] = 23;
    idxbxe_0[24] = 24;
    idxbxe_0[25] = 25;
    idxbxe_0[26] = 26;
    idxbxe_0[27] = 27;
    idxbxe_0[28] = 28;
    idxbxe_0[29] = 29;
    idxbxe_0[30] = 30;
    idxbxe_0[31] = 31;
    idxbxe_0[32] = 32;
    idxbxe_0[33] = 33;
    idxbxe_0[34] = 34;
    idxbxe_0[35] = 35;
    idxbxe_0[36] = 36;
    idxbxe_0[37] = 37;
    idxbxe_0[38] = 38;
    idxbxe_0[39] = 39;
    idxbxe_0[40] = 40;
    idxbxe_0[41] = 41;
    idxbxe_0[42] = 42;
    idxbxe_0[43] = 43;
    idxbxe_0[44] = 44;
    idxbxe_0[45] = 45;
    idxbxe_0[46] = 46;
    idxbxe_0[47] = 47;
    idxbxe_0[48] = 48;
    idxbxe_0[49] = 49;
    idxbxe_0[50] = 50;
    idxbxe_0[51] = 51;
    idxbxe_0[52] = 52;
    idxbxe_0[53] = 53;
    idxbxe_0[54] = 54;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);












    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    idxbu[3] = 3;
    idxbu[4] = 4;
    idxbu[5] = 5;
    idxbu[6] = 6;
    idxbu[7] = 7;
    idxbu[8] = 8;
    idxbu[9] = 9;
    idxbu[10] = 10;
    idxbu[11] = 11;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    lbu[0] = -5;
    ubu[0] = 5;
    lbu[1] = -5;
    ubu[1] = 5;
    lbu[2] = -5;
    ubu[2] = 5;
    lbu[3] = -50;
    ubu[3] = 50;
    lbu[4] = -5;
    ubu[4] = 5;
    lbu[5] = -5;
    ubu[5] = 5;
    lbu[6] = -5;
    ubu[6] = 5;
    lbu[7] = -50;
    ubu[7] = 50;
    lbu[8] = -5;
    ubu[8] = 5;
    lbu[9] = -5;
    ubu[9] = 5;
    lbu[10] = -5;
    ubu[10] = 5;
    lbu[11] = -50;
    ubu[11] = 50;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);






    /* Path constraints */

    // x
    int* idxbx = malloc(NBX * sizeof(int));
    idxbx[0] = 25;
    idxbx[1] = 39;
    idxbx[2] = 53;
    double* lubx = calloc(2*NBX, sizeof(double));
    double* lbx = lubx;
    double* ubx = lubx + NBX;
    lbx[0] = 0.1;
    ubx[0] = 10;
    lbx[1] = 0.1;
    ubx[1] = 10;
    lbx[2] = 0.1;
    ubx[2] = 10;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubx", ubx);
    }
    free(idxbx);
    free(lubx);


    // set up nonlinear constraints for stage 1 to N-1
    double* luh = calloc(2*NH, sizeof(double));
    double* lh = luh;
    double* uh = luh + NH;
    uh[0] = 1024;
    uh[1] = 1024;
    uh[2] = 1024;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                      &capsule->nl_constr_h_fun_jac[i-1]);
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                      &capsule->nl_constr_h_fun[i-1]);
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "uh", uh);
        
        
    }
    free(luh);











    /* terminal constraints */




















}


static void kinodynamic_load_3cables_acados_create_set_opts(kinodynamic_load_3cables_solver_capsule* capsule)
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

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);

    bool store_iterates = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "store_iterates", &store_iterates);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_t0_init = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_t0_init", &qp_solver_t0_init);




    int as_rti_iter = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "as_rti_iter", &as_rti_iter);

    int as_rti_level = 4;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "as_rti_level", &as_rti_level);

    int rti_log_residuals = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_log_residuals", &rti_log_residuals);

    int rti_log_only_available_residuals = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_log_only_available_residuals", &rti_log_only_available_residuals);

    bool with_anderson_acceleration = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_anderson_acceleration", &with_anderson_acceleration);

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);


    int qp_solver_warm_start = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_warm_start", &qp_solver_warm_start);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);

    int ext_cost_num_hess = 0;
}


/**
 * Internal function for kinodynamic_load_3cables_acados_create: step 7
 */
void kinodynamic_load_3cables_acados_set_nlp_out(kinodynamic_load_3cables_solver_capsule* capsule)
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
    x0[6] = 1;
    x0[15] = -1;
    x0[25] = 3.27;
    x0[29] = -1;
    x0[39] = 3.27;
    x0[43] = -1;
    x0[53] = 3.27;


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
 * Internal function for kinodynamic_load_3cables_acados_create: step 9
 */
int kinodynamic_load_3cables_acados_create_precompute(kinodynamic_load_3cables_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int kinodynamic_load_3cables_acados_create_with_discretization(kinodynamic_load_3cables_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != KINODYNAMIC_LOAD_3CABLES_N && !new_time_steps) {
        fprintf(stderr, "kinodynamic_load_3cables_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, KINODYNAMIC_LOAD_3CABLES_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    kinodynamic_load_3cables_acados_create_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 2) create and set dimensions
    capsule->nlp_dims = kinodynamic_load_3cables_acados_create_setup_dimensions(capsule);

    // 3) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    kinodynamic_load_3cables_acados_create_set_opts(capsule);

    // 4) create and set nlp_out
    // 4.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 4.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    kinodynamic_load_3cables_acados_set_nlp_out(capsule);

    // 5) create nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);

    // 6) setup functions, nlp_in and default parameters
    kinodynamic_load_3cables_acados_create_setup_functions(capsule);
    kinodynamic_load_3cables_acados_setup_nlp_in(capsule, N, new_time_steps);
    kinodynamic_load_3cables_acados_create_set_default_parameters(capsule);

    // 7) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);


    // 8) do precomputations
    int status = kinodynamic_load_3cables_acados_create_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int kinodynamic_load_3cables_acados_update_qp_solver_cond_N(kinodynamic_load_3cables_solver_capsule* capsule, int qp_solver_cond_N)
{
    printf("\nacados_update_qp_solver_cond_N() not implemented, since no partial condensing solver is used!\n\n");
    exit(1);
}


int kinodynamic_load_3cables_acados_reset(kinodynamic_load_3cables_solver_capsule* capsule, int reset_qp_solver_mem)
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

    free(buffer);
    return 0;
}




int kinodynamic_load_3cables_acados_update_params(kinodynamic_load_3cables_solver_capsule* capsule, int stage, double *p, int np)
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


int kinodynamic_load_3cables_acados_update_params_sparse(kinodynamic_load_3cables_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    ocp_nlp_in_set_params_sparse(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, idx, p, n_update);

    return 0;
}


int kinodynamic_load_3cables_acados_set_p_global_and_precompute_dependencies(kinodynamic_load_3cables_solver_capsule* capsule, double* data, int data_len)
{

    // printf("No global_data, kinodynamic_load_3cables_acados_set_p_global_and_precompute_dependencies does nothing.\n");
    return 0;
}




int kinodynamic_load_3cables_acados_solve(kinodynamic_load_3cables_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}



int kinodynamic_load_3cables_acados_setup_qp_matrices_and_factorize(kinodynamic_load_3cables_solver_capsule* capsule)
{
    int solver_status = ocp_nlp_setup_qp_matrices_and_factorize(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}






int kinodynamic_load_3cables_acados_free(kinodynamic_load_3cables_solver_capsule* capsule)
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
    external_function_external_param_casadi_free(&capsule->cost_y_0_fun);
    external_function_external_param_casadi_free(&capsule->cost_y_0_fun_jac_ut_xt);
    for (int i = 0; i < N - 1; i++)
    {
        external_function_external_param_casadi_free(&capsule->cost_y_fun[i]);
        external_function_external_param_casadi_free(&capsule->cost_y_fun_jac_ut_xt[i]);
    }
    free(capsule->cost_y_fun);
    free(capsule->cost_y_fun_jac_ut_xt);
    external_function_external_param_casadi_free(&capsule->cost_y_e_fun);
    external_function_external_param_casadi_free(&capsule->cost_y_e_fun_jac_ut_xt);

    // constraints
    for (int i = 0; i < N-1; i++)
    {
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun_jac[i]);
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun[i]);
    }
    free(capsule->nl_constr_h_fun_jac);
    free(capsule->nl_constr_h_fun);



    return 0;
}


void kinodynamic_load_3cables_acados_print_stats(kinodynamic_load_3cables_solver_capsule* capsule)
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


    printf("iter\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            tmp_int = (int) stat[i + j * nrow];
            printf("%d\t", tmp_int);
        }
        printf("\n");
    }
}

int kinodynamic_load_3cables_acados_custom_update(kinodynamic_load_3cables_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *kinodynamic_load_3cables_acados_get_nlp_in(kinodynamic_load_3cables_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *kinodynamic_load_3cables_acados_get_nlp_out(kinodynamic_load_3cables_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *kinodynamic_load_3cables_acados_get_sens_out(kinodynamic_load_3cables_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *kinodynamic_load_3cables_acados_get_nlp_solver(kinodynamic_load_3cables_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *kinodynamic_load_3cables_acados_get_nlp_config(kinodynamic_load_3cables_solver_capsule* capsule) { return capsule->nlp_config; }
void *kinodynamic_load_3cables_acados_get_nlp_opts(kinodynamic_load_3cables_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *kinodynamic_load_3cables_acados_get_nlp_dims(kinodynamic_load_3cables_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *kinodynamic_load_3cables_acados_get_nlp_plan(kinodynamic_load_3cables_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
