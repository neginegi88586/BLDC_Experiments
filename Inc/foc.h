/*
 * foc.h
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#ifndef FOC_H
#define FOC_H


#include "fixed_q16.h"


typedef struct
{
	q16_t Id_ref_q16;
	q16_t Iq_ref_q16;

	q16_t Kp_d_q16;
	q16_t Ki_d_q16;
	q16_t Kp_q_q16;
	q16_t Ki_q_q16;

	q16_t Id_i_q16;
	q16_t Iq_i_q16;

	q16_t Vbus_q16;

	q16_t v_alpha_q16;
	q16_t v_beta_q16;

} FOC_t;


void FOC_Init(FOC_t *foc);
void FOC_CurrentLoopStep(FOC_t *foc, q16_t i_a_q16, q16_t i_b_q16,
		q16_t i_c_q16, q16_t theta_q16);


void FOC_AlphaBetaToSVPWM(FOC_t *foc, uint16_t *ccr1, uint16_t *ccr2,
		uint16_t *ccr3, uint16_t arr);


#endif
