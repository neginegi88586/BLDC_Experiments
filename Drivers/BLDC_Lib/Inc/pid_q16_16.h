/* pid_q16_16.h
 * 目的：
 *   Q16.16 で動作する位置型 PID（D-on-measurement、アンチワインドアップ付き）。
 */
#ifndef PID_Q16_16_H
#define PID_Q16_16_H
#include <stdint.h>
#include "fixed_q16_16.h"
typedef struct
{
    q16_16_t kp;
    q16_16_t ki;
    q16_16_t kd;
    q16_16_t out_min;
    q16_16_t out_max;
    q16_16_t integrator;
    q16_16_t prev_meas;
} pid_q16_16_t;
static inline void pid_q16_16_init(pid_q16_16_t *p){
    p->integrator = 0;
    p->prev_meas  = 0;
}
static inline q16_16_t pid_q16_16_step(pid_q16_16_t *p,
                                       q16_16_t setpoint,
                                       q16_16_t measurement,
                                       q16_16_t dt){
    q16_16_t error = q16_16_sub_sat(setpoint, measurement);
    q16_16_t P = q16_16_mul(p->kp, error);
    /* D は測定値微分（ノイズ増幅を抑える） */
    q16_16_t d_meas = q16_16_sub_sat(measurement, p->prev_meas);
    q16_16_t D = 0;
    if (dt != 0){
        D = q16_16_div(d_meas, dt);
        D = q16_16_mul(p->kd, D);
        D = -D;
    }
    p->prev_meas = measurement;
    q16_16_t u_noI = q16_16_add_sat(P, D);
    /* 積分項の候補を先に計算（アンチワインドアップ）*/
    q16_16_t I_cand = p->integrator;
    if (dt != 0 && p->ki != 0){
        q16_16_t inc = q16_16_mul(p->ki, q16_16_mul(error, dt));
        I_cand = q16_16_add_sat(I_cand, inc);
    }
    q16_16_t u = q16_16_add_sat(u_noI, I_cand);
    if (u > p->out_max){
        u = p->out_max;
        if ((p->ki != 0) && (u_noI < p->out_max)) p->integrator = I_cand;
    }else if (u < p->out_min){
        u = p->out_min;
        if ((p->ki != 0) && (u_noI > p->out_min)) p->integrator = I_cand;
    }else{
        p->integrator = I_cand;
    }
    return u;
}
#endif
