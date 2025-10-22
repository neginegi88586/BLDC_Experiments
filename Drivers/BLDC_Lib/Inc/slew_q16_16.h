/* slew_q16_16.h
 * 目的：
 *   出力指令（duty 0..1 等）の変化速度を制限し、突入・ノイズを抑制する。
 */
#ifndef SLEW_Q16_16_H
#define SLEW_Q16_16_H
#include <stdint.h>
#include "fixed_q16_16.h"
static inline q16_16_t q16_16_slew_step(q16_16_t prev,
                                        q16_16_t target,
                                        q16_16_t slew_up_per_s,
                                        q16_16_t slew_dn_per_s,
                                        q16_16_t dt_s){
    q16_16_t diff = q16_16_sub_sat(target, prev);
    if (diff == 0) return prev;
    q16_16_t max_step = (diff > 0) ? q16_16_mul(slew_up_per_s, dt_s)
                                   : q16_16_mul(slew_dn_per_s, dt_s);
    if (diff > 0){
        if (diff > max_step) return q16_16_add_sat(prev, max_step);
        return target;
    }else{
        if (-diff > max_step) return q16_16_sub_sat(prev, max_step);
        return target;
    }
}
#endif
