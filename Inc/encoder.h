/*
 * encoder.h
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "fixedpoint.h"

typedef struct
{
    uint8_t a, b;           // 現在の入力レベル
    uint8_t prev_state;     // 前回AB
    int32_t counter;        // 絶対カウント
    int32_t filt_counter;   // LPF後カウント
    int32_t step_q31;       // 1ステップ当たり増分（Q31）
    int32_t min_q31;        // 下限
    int32_t max_q31;        // 上限
    int32_t current_q31;    // 現在値（0〜1または-1〜1）
    uint8_t stable_count;   // チャタリング抑制カウンタ
    uint8_t debounce;       // ON/OFF用チャタリング
} Encoder_t;

void ENC_Init(Encoder_t *enc, int32_t step_q31, int32_t min_q31, int32_t max_q31);
void ENC_Scan(Encoder_t *enc, uint8_t pin_a, uint8_t pin_b);
void ENC_Update(Encoder_t *enc);

#endif
