/* softstart_Q16.h
 * 目的：
 *   指令全体に 0→1 のスケールを掛けるソフトスタート機能。
 */
#ifndef SOFTSTART_Q16_H
#define SOFTSTART_Q16_H
#include <stdint.h>
#include "fixed_q16.h"
typedef struct
{
	q16_t scale;
	q16_t step_per_s;
	uint8_t enabled;
} softstart_t;

static inline void softstart_init(softstart_t *s, q16_t rise_time_s)
{
	s->scale = 0;
	s->enabled = 0;
	if (rise_time_s <= 0)
		s->step_per_s = Q16_MAX;
	else
		s->step_per_s = q16_div(Q16_ONE, rise_time_s);
}

static inline void softstart_enable(softstart_t *s, uint8_t en)
{
	s->enabled = en ? 1 : 0;
}

static inline q16_t softstart_step(softstart_t *s, q16_t dt_s)
{
	if (!s->enabled)
	{
		s->scale = 0;
		return s->scale;
	}
	q16_t inc = q16_mul(s->step_per_s, dt_s);
	s->scale = q16_add_sat(s->scale, inc);
	if (s->scale > Q16_ONE)
		s->scale = Q16_ONE;
	return s->scale;
}
#endif
