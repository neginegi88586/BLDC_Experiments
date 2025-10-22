/* units_q16_16.h
 * 目的：
 *   物理量を SI 単位系 (V, A, rad/s, duty 0..1) に統一し、変換を一元管理する。
 */
#ifndef UNITS_Q16_16_H
#define UNITS_Q16_16_H
#include <stdint.h>
#include "fixed_q16_16.h"
#include "adc_vcal_q16_16.h"
extern adc_vcal_t g_vcal; /* app.c で定義 */
/* 実機値に合わせて修正：シャント・アンプ・オフセット */
static const q16_16_t UQ_RSHUNT_OHM = q16_16_from_frac(5, 100);   /* 0.05Ω */
static const q16_16_t UQ_AMP_GAIN   = q16_16_from_int(10);        /* x10   */
static const q16_16_t UQ_VOFFSET_V  = q16_16_from_frac(165, 100); /* 1.65V */
/* ADC 生→電圧[V]（較正 V/LSB 使用）*/
static inline q16_16_t uq_adc_to_volt(int32_t adc){
    q16_16_t vlsb = adc_vcal_get_v_per_lsb(&g_vcal);
    return q16_16_mul(q16_16_from_int(adc), vlsb);
}
/* 電圧→電流[A] */
static inline q16_16_t uq_volt_to_curr(q16_16_t v){
    q16_16_t num = q16_16_sub_sat(v, UQ_VOFFSET_V);
    q16_16_t den = q16_16_mul(UQ_RSHUNT_OHM, UQ_AMP_GAIN);
    return q16_16_div(num, den);
}
/* 角速度の相互変換 */
static const q16_16_t UQ_TWO_PI = q16_16_from_frac(710, 113); /* ≈ 2π */
static const q16_16_t UQ_60     = q16_16_from_int(60);
static inline q16_16_t uq_rpm_to_rad_s(q16_16_t rpm){
    return q16_16_div(q16_16_mul(rpm, UQ_TWO_PI), UQ_60);
}
static inline q16_16_t uq_rad_s_to_rpm(q16_16_t rad_s){
    return q16_16_div(q16_16_mul(rad_s, UQ_60), UQ_TWO_PI);
}
/* duty 変換 */
static inline q16_16_t uq_permille_to_duty(uint16_t p){
    return q16_16_from_frac(p, 1000);
}
static inline uint16_t uq_duty_to_permille(q16_16_t duty){
    q16_16_t x = q16_16_mul(duty, q16_16_from_int(1000));
    int32_t  r = q16_16_to_int_rnd(x);
    if (r < 0) r = 0;
    if (r > 1000) r = 1000;
    return (uint16_t)r;
}
#endif
