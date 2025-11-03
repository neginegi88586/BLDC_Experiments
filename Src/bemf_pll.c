/*
 * bemf_pll.c
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#include "bemf_pll.h"
#include "app.h"


void BEMF_PLL_Init(BEMF_PLL_t *o)
{
	o->theta_q16 = 0;
	o->omega_q16 = 0;
	o->integ_q16 = 0;
	o->i_alpha_prev = 0;
	o->i_beta_prev = 0;
	o->di_alpha_q16 = 0;
	o->di_beta_q16 = 0;
	o->e_alpha_q16 = 0;
	o->e_beta_q16 = 0;
}

void BEMF_PLL_Step(BEMF_PLL_t *o, q16_t v_alpha_q16, q16_t v_beta_q16,
		q16_t i_alpha_q16, q16_t i_beta_q16)
{
	q16_t di_a = q16_sub_sat(i_alpha_q16, o->i_alpha_prev);
	q16_t di_b = q16_sub_sat(i_beta_q16, o->i_beta_prev);

	q16_t di_alpha_inst = q16_div(di_a, o->Ts_q16);
	q16_t di_beta_inst = q16_div(di_b, o->Ts_q16);

	q16_t one_minus_alpha = q16_sub_sat(Q16_ONE, o->alpha_q16);
	o->di_alpha_q16 = q16_add_sat(q16_mul(o->alpha_q16, di_alpha_inst),
			q16_mul(one_minus_alpha, o->di_alpha_q16));
	o->di_beta_q16 = q16_add_sat(q16_mul(o->alpha_q16, di_beta_inst),
			q16_mul(one_minus_alpha, o->di_beta_q16));

	o->i_alpha_prev = i_alpha_q16;
	o->i_beta_prev = i_beta_q16;

	q16_t Ri_a = q16_mul(o->Rs_q16, i_alpha_q16);
	q16_t Ri_b = q16_mul(o->Rs_q16, i_beta_q16);
	q16_t Ldidt_a = q16_mul(o->Ls_q16, o->di_alpha_q16);
	q16_t Ldidt_b = q16_mul(o->Ls_q16, o->di_beta_q16);

	q16_t sub_a = q16_add_sat(Ri_a, Ldidt_a);
	q16_t sub_b = q16_add_sat(Ri_b, Ldidt_b);

	q16_t e_a = q16_sub_sat(v_alpha_q16, sub_a);
	q16_t e_b = q16_sub_sat(v_beta_q16, sub_b);

	o->e_alpha_q16 = e_a;
	o->e_beta_q16 = e_b;

	q16_t s, c;
	sincos_q16(o->theta_q16, &s, &c);
	q16_t e_q1 = q16_mul(e_a, -s);
	q16_t e_q2 = q16_mul(e_b, c);
	q16_t eps = q16_add_sat(e_q1, e_q2);

	o->integ_q16 = q16_add_sat(o->integ_q16, q16_mul(o->ki_q16, eps));
	if (o->integ_q16 > o->integ_max_q16)
		o->integ_q16 = o->integ_max_q16;
	if (o->integ_q16 < o->integ_min_q16)
		o->integ_q16 = o->integ_min_q16;

	q16_t prop = q16_mul(o->kp_q16, eps);
	q16_t domega = q16_add_sat(prop, o->integ_q16);
	o->omega_q16 = q16_add_sat(o->omega_q16, domega);

	if (o->omega_q16 > o->omega_max_q16)
		o->omega_q16 = o->omega_max_q16;
}
