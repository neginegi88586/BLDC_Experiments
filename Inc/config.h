/*
 * config.h
 *
 *  Created on: Oct 12, 2025
 *      Author: idune
 */

#ifndef CONFIG_H
#define CONFIG_H


// ===== クロック/タイミング =====
#define SYSCLK_HZ 		168000000
#define APB1_HZ 		42000000
#define APB2_HZ 		84000000

// TIM1 は APB2 x2 = 168MHz
#define TIM1_CLK_HZ 	168000000

// PWM 21kHz, center-aligned → ARR = TIM1/(2*Fpwm)-1
#define PWM_FREQ_HZ 	21000
#define TIM1_ARR 		(TIM1_CLK_HZ/(2*PWM_FREQ_HZ) - 1)

// ADC クロック (APB2/8 ≈ 10.5MHz)
#define ADC_CLK_HZ 		21000000

// 制御周期（= PWM周期に1回呼ぶ想定）
#define CTRL_TS_SEC 	(1.0f / PWM_FREQ_HZ)

// デッドタイム
#define DTG_TICKS  60  // おおよそ ~ 300ns 相当（まずは安全側）


// =====　電圧変換用定数 =====
#define AD_MAX			4095
#define V_REF_AD		0x00013C28
#define V_DIV			0x0000C978

#define V_CENTER		0x0001A666
#define R_SHUNT			0x00000CCC


// ===== 電流制限用定数 =====
#define I_MAX			2500
#define I_MIN			(-(1000))


// ===== ADC チャネル割当 =====
#define ADC_CH_I_U 		2 // 実配線に合わせて調整
#define ADC_CH_I_V 		3
#define ADC_CH_I_W 		4

#define ADC_CH_V_REF	0 // リファレンス電圧
#define ADC_CH_V_BATT 	1 // バッテリー電圧(分圧)

#define ADC_CH_V_CC 	5 // モーター中央電圧(分圧)
#define ADC_CH_V_U 		6 // 相電圧（分圧）
#define ADC_CH_V_V 		7
#define ADC_CH_V_W 		8

#define ADC_SAMPLEING_TIME 	0x04924924


// ===== Human Interface (Throttle) =====
#define THR_ADC_MAX_Q31        Q31_FRAC(1,1)          // 1.0 = フルスケール
#define THR_DEADBAND_Q31       Q31_FRAC(1,50)         // 0.02 ≒ 2%
#define THR_LPF_ALPHA_Q31      Q31_FRAC(1,8)          // LPF係数 α (大→速応答)
#define THR_SLEW_PER_TICK_Q31  Q31_FRAC(1,200)        // 1周期あたり最大変化量


// ===== EXTSEL/JEXTSEL 定数 =====
#define ADC1_EXTSEL_TIM3_TRGO (0b1000) // 例：要RM/ヘッダ確認
#define ADC1_JEXTSEL_TIM1_CC4 (0b0000) // 例：要RM/ヘッダ確認


// ===== モータ/制御パラメータ（完全整数Q31） =====
// 実数値は分数表現で記述（浮動小数は使わない）
#define CONF_RS_Q31					Q31_FRAC(15, 100) // 0.15Ω
#define CONF_LS_Q31					Q31_FRAC(2, 10000) // 0.0002H
#define CONF_OBS_ALPHA_Q31 			Q31_FRAC(1, 5) // 0.2
#define CONF_PLL_KP_Q31 			Q31_FRAC(1, 100) // 0.01
#define CONF_PLL_KI_Q31 			Q31_FRAC(1, 2000) // 0.0005

// 速度・積分制限は「1制御周期あたりのturn量」で指定（= ω[rad/s] * (1/2π) * Ts）
// 例：|ω|<=2000rad/s, Ts=1/20k → |Δθ|≈2000/(2π)/20000 ≈ 0.0159turn
#define CONF_OMEGA_STEP_MAX_Q31		Q31_FRAC(152, 10000) // 0.0152turn
#define CONF_OMEGA_STEP_MIN_Q31 	(-(CONF_OMEGA_STEP_MAX_Q31))
#define CONF_PLL_INT_MIN_Q31		Q31_FRAC(-1, 5) // -0.2
#define CONF_PLL_INT_MAX_Q31 		Q31_FRAC(1, 5) // +0.2
#define IQ_MAX_Q31					Q31_FRAC(2,10)         // 0.2 (必要に応じて調整)

// 速度PI（出力はIq_refに合成）
#define SPEED_KP_Q31           Q31_FRAC(1,50)         // 0.02
#define SPEED_KI_Q31           Q31_FRAC(1,4000)       // 0.00025


// ===== Startup (sensorless) =====
#define ST_ALIGN_TIME_TICKS     400            // ≈20ms @20kHz
#define ST_ALIGN_ID_Q31         Q31_FRAC(1,1) // Id=0.1
#define ST_RAMP_IQ_Q31          Q31_FRAC(1,20) // Iq=0.05 から開始
#define ST_RAMP_DIDQ_TICK_Q31   Q31_FRAC(1,400)// Iqのスルレート(1周期あたり)
#define ST_OMEGA_STEP_INIT_Q31  Q31_FRAC(1,20000) // 1.0 turn/s = 1/20k per tick
#define ST_OMEGA_STEP_MAX_Q31   Q31_FRAC(1,2000)  // 10 turn/s 相当へ上げる例
#define ST_OMEGA_STEP_SLEW_Q31  Q31_FRAC(1,400000)// ωstepスルレート(小さく)

#define ST_HANDOFF_MIN_TICKS    600            // 最低30ms経過
#define ST_HANDOFF_OMEGA_MIN    Q31_FRAC(1,4000)   // PLL|ω|>5 turn/s 相当
#define ST_HANDOFF_EMF_MIN      Q31_FRAC(1,200)    // |e| > 0.005 (目安)
#define ST_BLEND_TICKS          200            // ブレンド期間 ≈10ms
#define ST_TIMEOUT_TICKS        4000           // 200msで諦め


// ===== エンコーダ設定 =====
#define ENC_STEP_Q31     Q31_FRAC(1, 20)   // 1クリックで0.05ずつ増減
#define ENC_MIN_Q31      Q31_FRAC(-1, 1)
#define ENC_MAX_Q31      Q31_FRAC(1, 1)

#endif
