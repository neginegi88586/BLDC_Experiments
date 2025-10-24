/*
 * fixedpoint.h
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H

#include <stdint.h>


#define Q31_ONE ((int32_t)0x7FFFFFFF)
#define Q31_ZERO ((int32_t)0)
#define Q31_FROM_FLOAT(x) ((int32_t)((x) * 2147483648.0f))


// 汎用：分数 → Q31 整数（丸め）
#define Q31_FRAC(NUM, DEN) ((int32_t)((((int64_t)(NUM) << 31) + ((DEN)/2)) / (int64_t)(DEN)))


// ---- よく使う定数（すべて整数で定義） ----
#define Q31_HALF (Q31_ONE >> 1) // 0.5
#define Q31_INV_SQRT3 Q31_FRAC(577350269, 1000000000) // 1/sqrt(3)
#define Q31_SQRT3_OVER_2 Q31_FRAC(866025404, 1000000000) // sqrt(3)/2
#define Q31_MARGIN_2PCT Q31_FRAC(1, 50) // 0.02
#define Q31_INV_TWOPI Q31_FRAC(159154943, 1000000000) // 1/(2π) ≈ 0.159154943

static inline int32_t q31_mul(int32_t a, int32_t b)
{
	int64_t t = (int64_t) a * (int64_t) b;
	t >>= 31;
	if (t > 0x7FFFFFFFLL)
		t = 0x7FFFFFFFLL;
	if (t < -0x80000000LL)
		t = -0x80000000LL;
	return (int32_t) t;
}

static inline int32_t q31_div(int32_t a, int32_t b)
{
	if (b == 0)
		return (a >= 0) ? 0x7FFFFFFF : (int32_t) 0x80000000;
	int64_t t = ((int64_t) a << 31) / (int64_t) b;
	if (t > 0x7FFFFFFFLL)
		t = 0x7FFFFFFFLL;
	if (t < -0x80000000LL)
		t = -0x80000000LL;
	return (int32_t) t;
}

static inline int32_t q31_add_sat(int32_t a, int32_t b)
{
	int64_t t = (int64_t) a + (int64_t) b;
	if (t > 0x7FFFFFFFLL)
		t = 0x7FFFFFFFLL;
	if (t < -0x80000000LL)
		t = -0x80000000LL;
	return (int32_t) t;
}

static inline int32_t q31_sub_sat(int32_t a, int32_t b)
{
	int64_t t = (int64_t) a - (int64_t) b;
	if (t > 0x7FFFFFFFLL)
		t = 0x7FFFFFFFLL;
	if (t < -0x80000000LL)
		t = -0x80000000LL;
	return (int32_t) t;
}

// Q31の"turn"表現（1.0=2π）で ±1.0 の範囲に折り返す
static inline int32_t angle_wrap_q31(int32_t th)
{
	if (th > Q31_ONE)
		th -= (Q31_ONE + 1);
	else if (th < -(Q31_ONE))
		th += (Q31_ONE + 1);
	return th;
}

// sin/cos（CORDIC版）は app.c に定義
void sincos_q31(int32_t th, int32_t *s, int32_t *c);

#endif
