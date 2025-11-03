/*
 * encoder.h
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "fixed_q16.h"

typedef struct
{
    uint8_t a, b;           // 現在の入力レベル
    uint8_t prev_state;     // 前回AB
    q16_t counter;        // 絶対カウント
    q16_t filt_counter;   // LPF後カウント
    q16_t step_q16;       // 1ステップ当たり増分（q16）
    q16_t min_q16;        // 下限
    q16_t max_q16;        // 上限
    q16_t current_q16;    // 現在値（0〜1または-1〜1）
    uint8_t stable_count;   // チャタリング抑制カウンタ
    uint8_t debounce;       // ON/OFF用チャタリング
} Encoder_t;

void ENC_Init(Encoder_t *enc, int32_t step_q16, int32_t min_q16, int32_t max_q16);
void ENC_Scan(Encoder_t *enc, uint8_t pin_a, uint8_t pin_b);
void ENC_Update(Encoder_t *enc);

#endif
