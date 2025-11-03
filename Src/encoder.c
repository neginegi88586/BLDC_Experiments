/*
 * encoder.c
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#include "encoder.h"

// ---- 初期化 ----
void ENC_Init(Encoder_t *enc, q16_t step_q16, q16_t min_q16, q16_t max_q16)
{
    enc->a = 0;
    enc->b = 0;
    enc->prev_state = 0;
    enc->counter = 0;
    enc->filt_counter = 0;
    enc->step_q16 = step_q16;
    enc->min_q16 = min_q16;
    enc->max_q16 = max_q16;
    enc->current_q16 = 0;
    enc->stable_count = 0;
    enc->debounce = 0;
}

// ---- A/B入力から回転方向を算出する ----
// 呼び出し周期: 約1〜2ms（タイマ割り込み内）
void ENC_Scan(Encoder_t *enc, uint8_t pin_a, uint8_t pin_b)
{
	static uint8_t buff0 = 0, buff1 = 0, buff2 = 0;
    uint8_t ab = ((pin_a ? 0 : 1) << 1) | (pin_b ? 0 : 1);
    uint8_t prev = buff2;

    buff0 = ab;
    buff2 &= (buff1 | buff0);
    buff2 |= (buff1 & buff0);
    buff1 = buff0;


    if(prev != buff2)
    {
    	if((buff2 & 1) != 0)
    	{
    		if((buff2 & 2) != 0)
    		{
    			enc->counter++;
    		}
    		else
    		{
    			enc->counter--;
    		}
    	}
    }
}

// ---- カウント値からq16出力へ更新 ----
void ENC_Update(Encoder_t *enc)
{
    // LPFで滑らかにする（フィルタ係数 α=1/8 固定）
    int32_t diff = enc->counter - enc->filt_counter;
    enc->filt_counter += diff >> 3;

    // q16正規化：1クリックあたり step_q16 増減
    int64_t tmp = (int64_t)enc->filt_counter * (int64_t)enc->step_q16;
    q16_t q16_val = (int32_t)tmp;

    if (q16_val > enc->max_q16) q16_val = enc->max_q16;
    if (q16_val < enc->min_q16) q16_val = enc->min_q16;

    enc->current_q16 = q16_val;
}
