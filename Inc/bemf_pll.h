/*
 * bemf_pll.h
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#ifndef BEMF_PLL_H
#define BEMF_PLL_H


#include "fixed_q16.h"


typedef struct
{
	q16_t theta_q16;
	q16_t omega_q16;
	q16_t kp_q16;
	q16_t ki_q16;
	q16_t kd_q16;
	q16_t integ_q16;
	q16_t Rs_q16;
	q16_t Ls_q16;
	q16_t Ts_q16;
	q16_t Ts_inv_q16;
	q16_t alpha_q16;
	q16_t i_alpha_prev;
	q16_t i_beta_prev;
	q16_t di_alpha_q16;
	q16_t di_beta_q16;
	q16_t e_alpha_q16;
	q16_t e_beta_q16;
	q16_t omega_min_q16;
	q16_t omega_max_q16;
	q16_t integ_min_q16;
	q16_t integ_max_q16;
} BEMF_PLL_t;

void BEMF_PLL_Init(BEMF_PLL_t *o);
void BEMF_PLL_Step(BEMF_PLL_t *o, q16_t v_alpha_q16, q16_t v_beta_q16,
		q16_t i_alpha_q16, q16_t i_beta_q16);


#endif
