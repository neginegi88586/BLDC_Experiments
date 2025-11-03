/* units_q16.h
 * 目的：
 *   物理量を SI 単位系 (V, A, rad/s, duty 0..1) に統一し、変換を一元管理する。
 */
#ifndef UNITS_Q16_H
#define UNITS_Q16_H


#include <stdint.h>
#include "config.h"               /* ← 物理定数は config.h に集約 */
#include "fixed_q16.h"
#include "adc_vcal_q16.h"


extern adc_vcal_t g_vcal; /* app.c で定義 */

static const q16_t UQ_RSHUNT_OHM = (q16_t)CONFIG_RSHUNT_OHM_Q16;
static const q16_t UQ_AMP_GAIN   = (q16_t)CONFIG_AMP_GAIN_Q16;
static const q16_t UQ_VOFFSET_V  = (q16_t)CONFIG_VOFFSET_V_Q16;

/* ADC 生→電圧[V]（較正 V/LSB 使用）*/
static inline q16_t uq_adc_to_volt(int32_t adc)
{
	q16_t vlsb = adc_vcal_get_v_per_lsb(&g_vcal);
	return q16_mul(q16_from_int(adc), vlsb);
}

/* 電圧→電流[A] */
static inline q16_t uq_volt_to_curr(q16_t v)
{
	q16_t num = q16_sub_sat(v, UQ_VOFFSET_V);
	q16_t den = q16_mul(UQ_RSHUNT_OHM, UQ_AMP_GAIN);
	return q16_div(num, den);
}


/* 角速度の相互変換 */
static const q16_t UQ_TWO_PI = (q16_t)CONFIG_TWO_PI_Q16; /* ≈ 2π */
static const q16_t UQ_60     = (q16_t)CONFIG_60_Q16;

static inline q16_t uq_rpm_to_rad_s(q16_t rpm)
{
	return q16_div(q16_mul(rpm, UQ_TWO_PI), UQ_60);
}

static inline q16_t uq_rad_s_to_rpm(q16_t rad_s)
{
	return q16_div(q16_mul(rad_s, UQ_60), UQ_TWO_PI);
}


/* duty 変換 */
static inline q16_t uq_permille_to_duty(uint16_t p)
{
	return Q16_FRAC(p, 1000);
}

static inline uint16_t uq_duty_to_permille(q16_t duty)
{
	q16_t x = q16_mul(duty, q16_from_int(1000));
	int32_t r = q16_to_int_rnd(x);
	if (r < 0)
		r = 0;
	if (r > 1000)
		r = 1000;
	return (uint16_t) r;
}

#endif
