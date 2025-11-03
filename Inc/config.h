/*
 * config.h
 *
 *  Created on: Oct 12, 2025
 *      Author: idune
 */

#ifndef CONFIG_H
#define CONFIG_H


/* クロック/タイミング */
#define SYSCLK_HZ		168000000
#define APB1_HZ			42000000
#define APB2_HZ			84000000

/* TIM1 は APB2 x2 = 168MHz = SYSCLK */
#define TIM1_CLK_HZ		SYSCLK_HZ

/* PWM 21kHz, center-aligned → ARR = TIM1/(2*Fpwm)-1 */
#define PWM_FREQ_HZ		21000
#define TIM1_ARR		(TIM1_CLK_HZ/(2*PWM_FREQ_HZ) - 1)

/* ADC クロック (APB2/8 ≈ 10.5MHz) */
#define ADC_CLK_HZ		21000000

/* デッドタイム */
#define DTG_TICKS		60													/* おおよそ300ns 相当（まずは安全側） */


/* 16bit固定小数点演算用 */
#define Q16_FBITS (16)
#define Q16_FRAC(NUM, DEN)	((q16_t)((((int64_t)(NUM) << Q16_FBITS) + (((int64_t)DEN) / 2)) / (int64_t)(DEN)))


/* ユーザー操作用定数 */
#define ENC_STEP_Q16			Q16_FRAC(1, 20)								/* 1クリックで0.05ずつ増減 */
#define ENC_MIN_Q16				Q16_FRAC(-1, 1)
#define ENC_MAX_Q16				Q16_FRAC(1, 1)

#define THR_ADC_MAX_Q16        Q16_FRAC(1,1)								/* 1.0 = フルスケール */
#define THR_DEADBAND_Q16       Q16_FRAC(1,50)								/* 0.02 ≒ 2% */
#define THR_LPF_ALPHA_Q16      Q16_FRAC(1,8)								/* LPF係数 α (大→速応答) */
#define THR_SLEW_PER_TICK_Q16  Q16_FRAC(1,200)								/* 1周期あたり最大変化量 */


/* 電流制限用定数 */
#define I_MAX				5000											/* 5Aで電流制限 */
#define I_MIN				(-(1000))										/* -1Aで電流制限 */


/* ADC チャネル割当 */
#define ADC_CH_I_U			2												/* U相電流 */
#define ADC_CH_I_V			3												/* V相電流 */
#define ADC_CH_I_W			4												/* W相電流 */

#define ADC_CH_V_REF		0												/* リファレンス電圧 */
#define ADC_CH_V_BATT		1												/* バッテリー電圧(分圧) */

#define ADC_CH_V_CC			5												/* モーター中央電圧(分圧) */
#define ADC_CH_V_U			6												/* U相電圧（分圧） */
#define ADC_CH_V_V			7												/* V相電圧(分圧) */
#define ADC_CH_V_W			8												/* W相電圧(分圧) */

#define ADC_SAMPLEING_TIME 	0x04924924										/* ADCの各チャネルにおけるサンプリング時間を一括で設定する定数 */


/* 電源・ADC 関連 */
#define CONFIG_ADC_RESOLUTION_COUNTS	(4095)								/* 12bit ADC */
#define CONFIG_REF_Q16					Q16_FRAC(1235, 1000)				/* 1.235 V */

/* EXTSEL/JEXTSEL 定数 */
#define ADC1_EXTSEL_TIM3_TRGO			(0b1000)							/* 例：要RM/ヘッダ確認 */
#define ADC1_JEXTSEL_TIM1_CC4			(0b0000)							/* 例：要RM/ヘッダ確認 */

/* シャント・アンプ・オフセット（回路図の実値に合わせて設定）*/
#define CONFIG_RSHUNT_OHM_Q16			Q16_FRAC(5, 100)					/* 0.05 Ω */
#define CONFIG_AMP_GAIN_Q16				Q16_FRAC(3, 1)						/* x3 */
#define CONFIG_VOFFSET_V_Q16			Q16_FRAC(165, 100)					/* 1.65 V */
#define CONF_OBS_ALPHA_Q16				Q16_FRAC(1, 5)						/* 0.2 */

#define CONFIG_DT_S_Q16					Q16_FRAC(1, PWM_FREQ_HZ)			/* 1 / PWM_FREQ_HZ */

/* 電流フルスケール（必要に応じて調整）*/
#define CONFIG_I_MAX_A_Q16				Q16_FRAC(11, 1)						/* 11 A */

/* スルーレートとソフトスタート（初期値）*/
#define CONFIG_SLEW_UP_PER_S_Q16		Q16_FRAC(1, 1)						/* 1.0 / s */
#define CONFIG_SLEW_DN_PER_S_Q16 		Q16_FRAC(3, 1)						/* 3.0 / s */
#define CONFIG_SOFTSTART_RISE_S_Q16		Q16_FRAC(3, 10)						/* 0.3 s */

/* 角速度変換で使用（定数） */
#define CONFIG_TWO_PI_Q16				Q16_FRAC(710,113)					/* ≈ 2π */
#define CONFIG_60_Q16					Q16_FRAC(60, 1)


/* 速度・積分制限 */
#define CONF_OMEGA_STEP_MAX_Q16			Q16_FRAC(152, 10000)				/* 0.0152turn */
#define CONF_OMEGA_STEP_MIN_Q16 		(-(CONF_OMEGA_STEP_MAX_Q16))		/* -0.0152turn */
#define CONF_PLL_INT_MIN_Q16			Q16_FRAC(-1, 5)						/* -0.2 */
#define CONF_PLL_INT_MAX_Q16			Q16_FRAC(1, 5)						/* +0.2 */
#define IQ_MAX_Q16						Q16_FRAC(2, 10)						/* 0.2 (必要に応じて調整) */
#define IQ_MIN_Q16						Q16_FRAC(-2, 10)					/* -0.2 (必要に応じて調整) */


/* PID制御定数 */
#define SPEED_KP_Q16					Q16_FRAC(2, 100)					/* Pゲイン = 0.02 */
#define SPEED_KI_Q16					Q16_FRAC(0, 2000)					/* Iゲイン = 0.00 */
#define SPEED_KD_Q16					Q16_FRAC(0, 8000)					/* Dゲイン = 0.00 */


/* よく使う定数 */
#define Q16_ONE							Q16_FRAC(1, 1)						/* 1.0 */
#define Q16_HALF						(Q16_ONE >> 1)						/* 0.5 */
#define Q16_MAX							((q16_t)0x7FFFFFFF)
#define Q16_MIN							((q16_t)0x80000000)

#define Q16_INV_SQRT3					Q16_FRAC(577350269, 1000000000)		/* 1/sqrt(3) */
#define Q16_SQRT3_OVER_2				Q16_FRAC(866025404, 1000000000)		/* sqrt(3)/2 */
#define Q16_MARGIN_2PCT					Q16_FRAC(1, 50)						/* 0.02 */
#define Q16_INV_TWOPI					Q16_FRAC(159154943, 1000000000)		/* 1/(2π) ≈ 0.159154943 */


/* 初期化用定数 */
#define ST_ALIGN_TIME_TICKS				400									/* ≈20ms @20kHz */
#define ST_ALIGN_ID_Q16					Q16_FRAC(1, 1) 						/* Id=0.1 */
#define ST_RAMP_IQ_Q16					Q16_FRAC(1, 20)						/* Iq=0.05 から開始 */
#define ST_RAMP_DIDQ_TICK_Q16   		Q16_FRAC(1, 400)						/* Iqのスルレート(1周期あたり) */
#define ST_OMEGA_STEP_INIT_Q16			Q16_FRAC(1, 20000)					/* 1.0 turn/s = 1/20k per tick */
#define ST_OMEGA_STEP_MAX_Q16			Q16_FRAC(1, 2000)					/* 10 turn/s 相当へ上げる例 */
#define ST_OMEGA_STEP_SLEW_Q16			Q16_FRAC(1, 400000)					/* ωstepスルレート(小さく) */

#define ST_HANDOFF_MIN_TICKS			600									/* 最低30ms経過 */
#define ST_HANDOFF_OMEGA_MIN			Q16_FRAC(1, 4000)					/* PLL|ω|>5 turn/s 相当 */
#define ST_HANDOFF_EMF_MIN				Q16_FRAC(1, 200)						/* |e| > 0.005 (目安) */
#define ST_BLEND_TICKS					200									/* ブレンド期間 ≈10ms */
#define ST_TIMEOUT_TICKS				4000								/* 200msで諦め */


#endif /* CONFIG_PHYS_Q16_16_DEFINED */
