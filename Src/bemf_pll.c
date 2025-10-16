/*
 * bemf_pll.c
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#include "bemf_pll.h"
#include "fixedpoint.h"


void BEMF_PLL_Init(BEMF_PLL_t *o)
{
	o->theta_q31 = 0;
	o->omega_q31 = 0;
	o->integ_q31 = 0;
	o->i_alpha_prev = 0;
	o->i_beta_prev = 0;
	o->di_alpha_q31 = 0;
	o->di_beta_q31 = 0;
	o->e_alpha_q31 = 0;
	o->e_beta_q31 = 0;
}

void BEMF_PLL_Step(BEMF_PLL_t *o, int32_t v_alpha_q31, int32_t v_beta_q31,
		int32_t i_alpha_q31, int32_t i_beta_q31)
{
	int32_t di_a = q31_sub_sat(i_alpha_q31, o->i_alpha_prev);
	int32_t di_b = q31_sub_sat(i_beta_q31, o->i_beta_prev);

	int32_t di_alpha_inst = q31_div(di_a, o->Ts_q31);
	int32_t di_beta_inst = q31_div(di_b, o->Ts_q31);

	int32_t one_minus_alpha = q31_sub_sat(Q31_ONE, o->alpha_q31);
	o->di_alpha_q31 = q31_add_sat(q31_mul(o->alpha_q31, di_alpha_inst),
			q31_mul(one_minus_alpha, o->di_alpha_q31));
	o->di_beta_q31 = q31_add_sat(q31_mul(o->alpha_q31, di_beta_inst),
			q31_mul(one_minus_alpha, o->di_beta_q31));

	o->i_alpha_prev = i_alpha_q31;
	o->i_beta_prev = i_beta_q31;

	int32_t Ri_a = q31_mul(o->Rs_q31, i_alpha_q31);
	int32_t Ri_b = q31_mul(o->Rs_q31, i_beta_q31);
	int32_t Ldidt_a = q31_mul(o->Ls_q31, o->di_alpha_q31);
	int32_t Ldidt_b = q31_mul(o->Ls_q31, o->di_beta_q31);

	int32_t sub_a = q31_add_sat(Ri_a, Ldidt_a);
	int32_t sub_b = q31_add_sat(Ri_b, Ldidt_b);

	int32_t e_a = q31_sub_sat(v_alpha_q31, sub_a);
	int32_t e_b = q31_sub_sat(v_beta_q31, sub_b);

	o->e_alpha_q31 = e_a;
	o->e_beta_q31 = e_b;

	int32_t s, c;
	sincos_q31(o->theta_q31, &s, &c);
	int32_t e_q1 = q31_mul(e_a, -s);
	int32_t e_q2 = q31_mul(e_b, c);
	int32_t eps = q31_add_sat(e_q1, e_q2);

	o->integ_q31 = q31_add_sat(o->integ_q31, q31_mul(o->ki_q31, eps));
	if (o->integ_q31 > o->integ_max_q31)
		o->integ_q31 = o->integ_max_q31;
	if (o->integ_q31 < o->integ_min_q31)
		o->integ_q31 = o->integ_min_q31;

	int32_t prop = q31_mul(o->kp_q31, eps);
	int32_t domega = q31_add_sat(prop, o->integ_q31);
	o->omega_q31 = q31_add_sat(o->omega_q31, domega);

	if (o->omega_q31 > o->omega_max_q31)
		o->omega_q31 = o->omega_max_q31;
}
