/*
 * bemf_pll.h
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#ifndef BEMF_PLL_H
#define BEMF_PLL_H

#include <stdint.h>


typedef struct
{
	int32_t theta_q31;
	int32_t omega_q31;
	int32_t kp_q31;
	int32_t ki_q31;
	int32_t integ_q31;
	int32_t Rs_q31;
	int32_t Ls_q31;
	int32_t Ts_q31;
	int32_t Ts_inv_q31;
	int32_t alpha_q31;
	int32_t i_alpha_prev;
	int32_t i_beta_prev;
	int32_t di_alpha_q31;
	int32_t di_beta_q31;
	int32_t e_alpha_q31;
	int32_t e_beta_q31;
	int32_t omega_min_q31;
	int32_t omega_max_q31;
	int32_t integ_min_q31;
	int32_t integ_max_q31;
} BEMF_PLL_t;

void BEMF_PLL_Init(BEMF_PLL_t *o);
void BEMF_PLL_Step(BEMF_PLL_t *o, int32_t v_alpha_q31, int32_t v_beta_q31,
		int32_t i_alpha_q31, int32_t i_beta_q31);


#endif
