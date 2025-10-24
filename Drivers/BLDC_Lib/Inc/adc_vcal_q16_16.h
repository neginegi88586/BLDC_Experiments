/* adc_vcal_q16_16.h
 * 目的：
 *   PA0(ADC1 CH0) の 1.235V±1% 基準から VDDA と V/LSB をランタイム推定。
 * 運用：
 *   BG(1kHz等)で update、ISR は読み取りのみ。
 */
#ifndef ADC_VCAL_Q16_16_H
#define ADC_VCAL_Q16_16_H
#include <stdint.h>
#include "fixed_q16_16.h"

typedef struct
{
	q16_16_t v_ref_in; /* PA0 基準電圧 [V] */
	q16_16_t v_per_lsb; /* 1 LSB あたり電圧 [V/LSB] */
	q16_16_t alpha; /* IIR 係数 0..1 */
	uint8_t valid; /* 有効更新済みフラグ */
} adc_vcal_t;

static inline void adc_vcal_init(adc_vcal_t *s, q16_16_t v_ref_in,
		q16_16_t alpha)
{
	s->v_ref_in = v_ref_in;
	s->alpha = alpha;
	s->valid = 0;
	/* 初期：名目 3.3V → V/LSB = 3.3/4095 */
	q16_16_t vdda_nom = q16_16_from_frac(33, 10);
	s->v_per_lsb = q16_16_div(vdda_nom, q16_16_from_int(4095));
}

static inline void adc_vcal_update(adc_vcal_t *s, int32_t adc_raw_pa0)
{
	if (adc_raw_pa0 <= 0)
		return;
	q16_16_t den = q16_16_from_int(adc_raw_pa0);
	q16_16_t q4095 = q16_16_from_int(4095);
	q16_16_t vdda_est = q16_16_div(q16_16_mul(s->v_ref_in, q4095), den);
	q16_16_t vlsb_est = q16_16_div(vdda_est, q4095);
	q16_16_t one_minus = q16_16_sub_sat(Q16_16_ONE, s->alpha);
	q16_16_t tmp = q16_16_mul(one_minus, s->v_per_lsb);
	q16_16_t tmp2 = q16_16_mul(s->alpha, vlsb_est);
	s->v_per_lsb = q16_16_add_sat(tmp, tmp2);
	s->valid = 1;
}

static inline q16_16_t adc_vcal_get_v_per_lsb(const adc_vcal_t *s)
{
	return s->v_per_lsb;
}

#endif
