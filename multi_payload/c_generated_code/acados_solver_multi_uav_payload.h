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

#ifndef ACADOS_SOLVER_multi_uav_payload_H_
#define ACADOS_SOLVER_multi_uav_payload_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define MULTI_UAV_PAYLOAD_NX     43
#define MULTI_UAV_PAYLOAD_NZ     0
#define MULTI_UAV_PAYLOAD_NU     12
#define MULTI_UAV_PAYLOAD_NP     0
#define MULTI_UAV_PAYLOAD_NP_GLOBAL     0
#define MULTI_UAV_PAYLOAD_NBX    0
#define MULTI_UAV_PAYLOAD_NBX0   43
#define MULTI_UAV_PAYLOAD_NBU    12
#define MULTI_UAV_PAYLOAD_NSBX   0
#define MULTI_UAV_PAYLOAD_NSBU   0
#define MULTI_UAV_PAYLOAD_NSH    0
#define MULTI_UAV_PAYLOAD_NSH0   0
#define MULTI_UAV_PAYLOAD_NSG    0
#define MULTI_UAV_PAYLOAD_NSPHI  0
#define MULTI_UAV_PAYLOAD_NSHN   0
#define MULTI_UAV_PAYLOAD_NSGN   0
#define MULTI_UAV_PAYLOAD_NSPHIN 0
#define MULTI_UAV_PAYLOAD_NSPHI0 0
#define MULTI_UAV_PAYLOAD_NSBXN  0
#define MULTI_UAV_PAYLOAD_NS     0
#define MULTI_UAV_PAYLOAD_NS0    0
#define MULTI_UAV_PAYLOAD_NSN    0
#define MULTI_UAV_PAYLOAD_NG     0
#define MULTI_UAV_PAYLOAD_NBXN   0
#define MULTI_UAV_PAYLOAD_NGN    0
#define MULTI_UAV_PAYLOAD_NY0    55
#define MULTI_UAV_PAYLOAD_NY     55
#define MULTI_UAV_PAYLOAD_NYN    43
#define MULTI_UAV_PAYLOAD_N      20
#define MULTI_UAV_PAYLOAD_NH     0
#define MULTI_UAV_PAYLOAD_NHN    0
#define MULTI_UAV_PAYLOAD_NH0    0
#define MULTI_UAV_PAYLOAD_NPHI0  0
#define MULTI_UAV_PAYLOAD_NPHI   0
#define MULTI_UAV_PAYLOAD_NPHIN  0
#define MULTI_UAV_PAYLOAD_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct multi_uav_payload_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */

    // dynamics

    external_function_external_param_casadi *expl_vde_forw;
    external_function_external_param_casadi *expl_ode_fun;
    external_function_external_param_casadi *expl_vde_adj;




    // cost






    // constraints







} multi_uav_payload_solver_capsule;

ACADOS_SYMBOL_EXPORT multi_uav_payload_solver_capsule * multi_uav_payload_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_free_capsule(multi_uav_payload_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_create(multi_uav_payload_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_reset(multi_uav_payload_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of multi_uav_payload_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_create_with_discretization(multi_uav_payload_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_update_time_steps(multi_uav_payload_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_update_qp_solver_cond_N(multi_uav_payload_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_update_params(multi_uav_payload_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_update_params_sparse(multi_uav_payload_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_set_p_global_and_precompute_dependencies(multi_uav_payload_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_solve(multi_uav_payload_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_setup_qp_matrices_and_factorize(multi_uav_payload_solver_capsule* capsule);



ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_free(multi_uav_payload_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void multi_uav_payload_acados_print_stats(multi_uav_payload_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int multi_uav_payload_acados_custom_update(multi_uav_payload_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *multi_uav_payload_acados_get_nlp_in(multi_uav_payload_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *multi_uav_payload_acados_get_nlp_out(multi_uav_payload_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *multi_uav_payload_acados_get_sens_out(multi_uav_payload_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *multi_uav_payload_acados_get_nlp_solver(multi_uav_payload_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *multi_uav_payload_acados_get_nlp_config(multi_uav_payload_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *multi_uav_payload_acados_get_nlp_opts(multi_uav_payload_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *multi_uav_payload_acados_get_nlp_dims(multi_uav_payload_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *multi_uav_payload_acados_get_nlp_plan(multi_uav_payload_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_multi_uav_payload_H_
