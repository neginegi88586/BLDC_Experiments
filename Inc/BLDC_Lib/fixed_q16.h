/* fixed_Q16.h
 * 目的：
 *   Q16.16 固定小数点（小数16bit）による整数演算ユーティリティ。
 * 注意：
 *   すべて整数演算。64bit 中間値で飽和処理を行う。
 */
#ifndef FIXED_Q16_H
#define FIXED_Q16_H

#include "config.h"
#include <stdint.h>


typedef int32_t q16_t;


static inline q16_t q16_from_int(int32_t x)
{
	return (q16_t) (x << Q16_FBITS);
}

static inline q16_t q16_to_int(q16_t x)
{
	return (q16_t) (x >> Q16_FBITS);
}

static inline q16_t q16_to_int_rnd(q16_t x)
{
	if (x >= 0)
	{
		return (q16_t) ((x + Q16_HALF) >> Q16_FBITS);
	}
	else
	{
		return (q16_t) ((x + 1 - Q16_HALF) >> Q16_FBITS);
	}
}

static inline q16_t q16_mul(q16_t a, q16_t b)
{
	int64_t t = (int64_t) a * (int64_t) b;
	t = (t + (int64_t) Q16_HALF) >> Q16_FBITS;
	if (t > (int64_t) Q16_MAX)
		return Q16_MAX;
	if (t < (int64_t) Q16_MIN)
		return Q16_MIN;
	return (q16_t) t;
}

static inline q16_t q16_div(q16_t a, q16_t b)
{
	if (b == 0)
		return (a >= 0) ? Q16_MAX : Q16_MIN;
	int64_t t = ((int64_t) a << Q16_FBITS);
	if ((t >= 0 && b > 0) || (t < 0 && b < 0))
		t += (b > 0) ? (b / 2) : (-b / 2);
	else
		t -= (b > 0) ? (b / 2) : (-b / 2);
	int64_t q = t / b;
	if (q > (int64_t) Q16_MAX)
		return Q16_MAX;
	if (q < (int64_t) Q16_MIN)
		return Q16_MIN;
	return (q16_t) q;
}

static inline q16_t q16_add_sat(q16_t a, q16_t b)
{
	int64_t s = (int64_t) a + (int64_t) b;
	if (s > (int64_t) Q16_MAX)
		return Q16_MAX;
	if (s < (int64_t) Q16_MIN)
		return Q16_MIN;
	return (q16_t) s;
}

static inline q16_t q16_sub_sat(q16_t a, q16_t b)
{
	int64_t d = (int64_t) a - (int64_t) b;
	if (d > (int64_t) Q16_MAX)
		return Q16_MAX;
	if (d < (int64_t) Q16_MIN)
		return Q16_MIN;
	return (q16_t) d;
}

static inline q16_t angle_wrap_q16(q16_t th)
{
	if (th > Q16_ONE)
		th -= (Q16_ONE + 1);
	else if (th < -(Q16_ONE))
		th += (Q16_ONE + 1);
	return th;
}


#endif
