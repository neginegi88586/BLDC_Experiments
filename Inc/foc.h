/*
 * foc.h
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#ifndef FOC_H
#define FOC_H

#include <stdint.h>


typedef struct
{
	int32_t Id_ref_q31;
	int32_t Iq_ref_q31;

	int32_t Kp_d_q31;
	int32_t Ki_d_q31;
	int32_t Kp_q_q31;
	int32_t Ki_q_q31;

	int32_t Id_i_q31;
	int32_t Iq_i_q31;

	int32_t Vbus_q31;

	int32_t v_alpha_q31;
	int32_t v_beta_q31;

} FOC_t;


void FOC_Init(FOC_t *foc);
void FOC_CurrentLoopStep(FOC_t *foc, int32_t i_a_q31, int32_t i_b_q31,
		int32_t i_c_q31, int32_t theta_q31);


void FOC_AlphaBetaToSVPWM(FOC_t *foc, uint16_t *ccr1, uint16_t *ccr2,
		uint16_t *ccr3, uint16_t arr);


#endif
