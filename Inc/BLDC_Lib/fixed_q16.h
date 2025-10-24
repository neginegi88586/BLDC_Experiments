/* fixed_q16_16.h
 * 目的：
 *   Q16.16 固定小数点（小数16bit）による整数演算ユーティリティ。
 * 注意：
 *   すべて整数演算。64bit 中間値で飽和処理を行う。
 */
#ifndef FIXED_Q16_H
#define FIXED_Q16_H
#include <stdint.h>
typedef int32_t q16_16_t;
#define Q16_16_FBITS (16)
#define Q16_16_ONE   ((q16_16_t)(1 << Q16_16_FBITS))
#define Q16_16_HALF  ((q16_16_t)(1 << (Q16_16_FBITS - 1)))
#define Q16_16_MAX   ((q16_16_t)0x7FFFFFFF)
#define Q16_16_MIN   ((q16_16_t)0x80000000)

static inline q16_16_t q16_16_from_int(int32_t x)
{
	return (q16_16_t) (x << Q16_16_FBITS);
}

static inline int32_t q16_16_to_int(q16_16_t x)
{
	return (int32_t) (x >> Q16_16_FBITS);
}

static inline int32_t q16_16_to_int_rnd(q16_16_t x)
{
	if (x >= 0)
	{
		return (int32_t) ((x + Q16_16_HALF) >> Q16_16_FBITS);
	}
	else
	{
		return (int32_t) ((x + 1 - Q16_16_HALF) >> Q16_16_FBITS);
	}
}

static inline q16_16_t q16_16_from_frac(int32_t num, int32_t den)
{
	int64_t t = ((int64_t) num << Q16_16_FBITS);
	if ((t >= 0 && den > 0) || (t < 0 && den < 0))
		t += (den > 0) ? (den / 2) : (-den / 2);
	else
		t -= (den > 0) ? (den / 2) : (-den / 2);
	return (q16_16_t) (t / den);
}

static inline q16_16_t q16_16_mul(q16_16_t a, q16_16_t b)
{
	int64_t t = (int64_t) a * (int64_t) b;
	t = (t + (int64_t) Q16_16_HALF) >> Q16_16_FBITS;
	if (t > (int64_t) Q16_16_MAX)
		return Q16_16_MAX;
	if (t < (int64_t) Q16_16_MIN)
		return Q16_16_MIN;
	return (q16_16_t) t;
}

static inline q16_16_t q16_16_div(q16_16_t a, q16_16_t b)
{
	if (b == 0)
		return (a >= 0) ? Q16_16_MAX : Q16_16_MIN;
	int64_t t = ((int64_t) a << Q16_16_FBITS);
	if ((t >= 0 && b > 0) || (t < 0 && b < 0))
		t += (b > 0) ? (b / 2) : (-b / 2);
	else
		t -= (b > 0) ? (b / 2) : (-b / 2);
	int64_t q = t / b;
	if (q > (int64_t) Q16_16_MAX)
		return Q16_16_MAX;
	if (q < (int64_t) Q16_16_MIN)
		return Q16_16_MIN;
	return (q16_16_t) q;
}

static inline q16_16_t q16_16_add_sat(q16_16_t a, q16_16_t b)
{
	int64_t s = (int64_t) a + (int64_t) b;
	if (s > (int64_t) Q16_16_MAX)
		return Q16_16_MAX;
	if (s < (int64_t) Q16_16_MIN)
		return Q16_16_MIN;
	return (q16_16_t) s;
}

static inline q16_16_t q16_16_sub_sat(q16_16_t a, q16_16_t b)
{
	int64_t d = (int64_t) a - (int64_t) b;
	if (d > (int64_t) Q16_16_MAX)
		return Q16_16_MAX;
	if (d < (int64_t) Q16_16_MIN)
		return Q16_16_MIN;
	return (q16_16_t) d;
}

#endif
