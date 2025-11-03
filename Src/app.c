/* app.c
 * 目的：
 *   - PA0(ADC1_CH0)=1.235V 基準で V/LSB をランタイム較正
 *   - BG系（既存コールバック）で軽量に更新
 *   - 行コメントはブロックコメントへ統一
 */

#include "config.h"
#include "app.h"
#include "firmware.h"
#include "foc.h"
#include "bemf_pll.h"
#include "encoder.h"

/* 追加：Q16.16 ユーティリティ */
#include "fixed_q16.h"
#include "adc_vcal_q16.h"
#include "units_q16.h"


/* 較正状態（他の翻訳単位から参照される） */
adc_vcal_t g_vcal;

/* ====== CORDIC: 係数も完全整数化 ====== */
#define CORDIC_ITERS 12
static const q16_t k_cordic_K_q16 = Q16_FRAC(607252935, 1000000000); /* ≈0.607252935 */

/* atan(2^-i)/(2π) を i に対して **線形補間** で近似する関数 */
/* アンカー（i=0,4,8,12）を分数→Q16で整数定義し、i間は単純Lerp */
static const struct
{
	uint8_t i;
	q16_t v;
}
k_atan_anchor[] =
{
		{ 0, Q16_FRAC(125000000, 1000000000) },		/* 0.1250000000 */
		{ 4, Q16_FRAC(993426215, 100000000000) },	/* 0.00993426215 */
		{ 8, Q16_FRAC(62169583, 100000000000) },	/* 0.00062169583 */
		{ 12, Q16_FRAC(3885619, 100000000000) }		/* 0.00003885619 */
};

/* --- Startup state machine --- */
typedef enum
{
	ST_STOP = 0, ST_ALIGN, ST_RAMP, ST_BLEND, ST_RUN, ST_FAIL
} st_t;
static st_t s_st = ST_STOP;
static q16_t s_th_forced = 0;			/* 強制角 (turn-Q16) */
static q16_t s_omg_step = 0;			/* 1周期あたりのΔθ */
static q16_t s_iq_cmd = 0;				/* 開ループ中の Iq 指令 */
static q16_t s_tick = 0;				/* 経過tick */

static inline q16_t q16_abs(q16_t x)
{
	return (x >= 0) ? x : -(x);
}
static inline q16_t q16_min(q16_t a, q16_t b)
{
	return (a < b) ? a : b;
}
static inline q16_t q16_max(q16_t a, q16_t b)
{
	return (a > b) ? a : b;
}

/* 推定BEMFの強さの簡易指標: |e| ≈ max(|eα|,|eβ|) */
static inline q16_t emf_strength_q16(const BEMF_PLL_t *o)
{
	q16_t ea = q16_abs(o->e_alpha_q16);
	q16_t eb = q16_abs(o->e_beta_q16);
	return (ea > eb) ? ea : eb;
}

static inline q16_t atan_turn_i_q16(int i)
{
	if (i <= k_atan_anchor[0].i)
		return k_atan_anchor[0].v;
	if (i >= k_atan_anchor[3].i)
		return k_atan_anchor[3].v;
	for (int k = 0; k < 3; k++)
	{
		int i0 = k_atan_anchor[k].i;
		int i1 = k_atan_anchor[k + 1].i;
		if (i >= i0 && i <= i1)
		{
			q16_t y0 = k_atan_anchor[k].v;
			q16_t y1 = k_atan_anchor[k + 1].v;
			q16_t di = (q16_t) (i - i0);
			q16_t wi = (q16_t) (i1 - i0);
			q16_t dy = q16_sub_sat(y1, y0);
			q16_t t = (q16_t) ((((int64_t) di << 16) + (wi / 2)) / wi); /* di/wi をQ16に */
			return q16_add_sat(y0, q16_mul(dy, t));
		}
	}
	return k_atan_anchor[3].v;
}

void sincos_q16(q16_t th, q16_t *s, q16_t *c)
{
	th = angle_wrap_q16(th);

	int8_t q = 0;
	if (th > Q16_ONE / 4)
	{
		th -= (Q16_ONE / 2);
		q = +1;
	}
	else if (th < -Q16_ONE / 4)
	{
		th += (Q16_ONE / 2);
		q = -1;
	}

	q16_t x = k_cordic_K_q16; /* cos */
	q16_t y = 0; /* sin */
	q16_t z = th; /* turn-Q16 */

	for (int i = 0; i < CORDIC_ITERS; i++)
	{
		q16_t angle_i = atan_turn_i_q16(i); /* 線形補間で取得 */
		q16_t dx = (y >> i);
		q16_t dy = (x >> i);
		if (z >= 0)
		{
			x = q16_sub_sat(x, dx);
			y = q16_add_sat(y, dy);
			z = q16_sub_sat(z, angle_i);
		}
		else
		{
			x = q16_add_sat(x, dx);
			y = q16_sub_sat(y, dy);
			z = q16_add_sat(z, angle_i);
		}
	}

	if (q > 0)
	{
		*s = x;
		*c = -y;
	}
	else if (q < 0)
	{
		*s = -x;
		*c = y;
	}
	else
	{
		*s = y;
		*c = x;
	}
}

/* ====== アプリ層本体 ====== */
static FOC_t s_foc;
static BEMF_PLL_t s_pll;

static q16_t s_thr_filt_q16 = 0;	/* LPF後の0..1 */
static q16_t s_mode_speed = 0;		/* 0=トルク直結, 1=速度PI */
static q16_t s_speed_int_q16 = 0;	/* 速度PIDの積分 */
static q16_t s_speed_diff_q16 = 0;	/* 速度PIDの微分 */

static volatile uint16_t s_voltage[2];
static volatile uint16_t s_vphase_adc[4];
static volatile uint16_t s_current[3];
static volatile q16_t s_iPacked;

uint16_t bat_voltage_buff;

uint16_t motor_voltage_buff[4];
int16_t motor_current_buff[3];

extern Encoder_t s_enc;

static inline q16_t adc_to_q16(uint16_t v)
{
	return (q16_t) ((int64_t) v * (int64_t) (Q16_ONE >> 11));
}

static inline void clarke_q16(q16_t ia, q16_t ib, q16_t ic,
		q16_t *ialpha, q16_t *ibeta)
{
	*ialpha = ia;
	q16_t two_ib = q16_add_sat(ib, ib);
	q16_t sum = q16_add_sat(ia, two_ib);
	*ibeta = q16_mul(sum, Q16_INV_SQRT3);
}

static inline q16_t throttle_shape_q16(q16_t thr_raw_q16)
{
	/* デッドバンド */
	if (thr_raw_q16 < THR_DEADBAND_Q16)
		thr_raw_q16 = 0;

	/* LPF: y = αx + (1-α)y */
	q16_t one_minus_a = q16_sub_sat(Q16_ONE, THR_LPF_ALPHA_Q16);
	s_thr_filt_q16 = q16_add_sat(q16_mul(THR_LPF_ALPHA_Q16, thr_raw_q16),
			q16_mul(one_minus_a, s_thr_filt_q16));

	/* スルーレート（入力の瞬間変化を更に縛りたい場合は、前回値を覚えて制限） */
	static q16_t prev = 0;
	q16_t diff = q16_sub_sat(s_thr_filt_q16, prev);
	if (diff > THR_SLEW_PER_TICK_Q16)
		s_thr_filt_q16 = q16_add_sat(prev, THR_SLEW_PER_TICK_Q16);
	else if (diff < -THR_SLEW_PER_TICK_Q16)
		s_thr_filt_q16 = q16_sub_sat(prev, THR_SLEW_PER_TICK_Q16);
	prev = s_thr_filt_q16;

	/* 上限 */
	if (s_thr_filt_q16 > THR_ADC_MAX_Q16)
		s_thr_filt_q16 = THR_ADC_MAX_Q16;
	return s_thr_filt_q16;
}

/* 速度PI（PLLのωを使って Iq_ref を作る） */
static inline q16_t speed_pid_to_iq_q16(q16_t omega_ref_step_q16,
		q16_t omega_meas_step_q16)
{
	q16_t e = q16_sub_sat(omega_ref_step_q16, omega_meas_step_q16);
	s_speed_int_q16 = q16_add_sat(s_speed_int_q16, q16_mul(SPEED_KI_Q16, e));
	s_speed_diff_q16 = q16_div(q16_mul(SPEED_KD_Q16, e), CONFIG_DT_S_Q16);

	/* 積分アンチワインドアップ：Iqの範囲に収める */
	if (s_speed_int_q16 > IQ_MAX_Q16)
		s_speed_int_q16 = IQ_MAX_Q16;
	if (s_speed_int_q16 < 0)
		s_speed_int_q16 = 0; /* 順回転限定なら0～に */

	q16_t out = q16_add_sat(q16_mul(SPEED_KP_Q16, e), s_speed_int_q16);

	/* 出力リミット */
	if (out > IQ_MAX_Q16)
		out = IQ_MAX_Q16;
	if (out < 0)
		out = 0;
	return out; /* Iq_ref */
}

void APP_Init(void)
{
	FOC_Init(&s_foc);
	BEMF_PLL_Init(&s_pll);

	/*
	 * ADC 較正の初期化。
	 * v_ref_in = 1.235V, alpha = 0.1（1kHz更新なら時定数約9ms）
	 */
	adc_vcal_init(&g_vcal, Q16_FRAC(1235, 1000),
			Q16_FRAC(1, 10));

	s_pll.Ts_q16 = (q16_t) (((int64_t) 1 << 31) / (int64_t) PWM_FREQ_HZ);
	s_pll.Rs_q16 = CONFIG_RSHUNT_OHM_Q16;
	s_pll.alpha_q16 = CONF_OBS_ALPHA_Q16;
	s_pll.kp_q16 = SPEED_KP_Q16;
	s_pll.ki_q16 = SPEED_KI_Q16;
	s_pll.kd_q16 = SPEED_KD_Q16;
	s_pll.omega_min_q16 = CONF_OMEGA_STEP_MIN_Q16;
	s_pll.omega_max_q16 = CONF_OMEGA_STEP_MAX_Q16;
	s_pll.integ_min_q16 = CONF_PLL_INT_MIN_Q16;
	s_pll.integ_max_q16 = CONF_PLL_INT_MAX_Q16;

	s_st = ST_ALIGN;
	s_tick = 0;
	s_th_forced = 0;
	s_omg_step = ST_OMEGA_STEP_INIT_Q16;
	s_iq_cmd = 0;
}

void APP_OnCurrents(uint16_t iU, uint16_t iV, uint16_t iW)
{
	s_iPacked = (q16_t) (iV << 16) | (q16_t) (iU);

	s_current[0] = iU;
	s_current[1] = iV;
	s_current[2] = iW;
}

void APP_OnVphase(uint16_t *v_adc)
{
	s_vphase_adc[0] = *v_adc;
	s_vphase_adc[1] = *(v_adc + 1);
	s_vphase_adc[2] = *(v_adc + 2);
	s_vphase_adc[3] = *(v_adc + 3);
}

void APP_OnVoltage(uint16_t *v_adc)
{
	/*
	 * ここでは v_adc[0] を PA0(1.235V) と仮定して較正更新を行う。
	 * プロジェクトでの実CH割付に合わせてインデックスを調整すること。
	 */
	s_voltage[0] = *v_adc;
	adc_vcal_update(&g_vcal, (q16_t) s_voltage[0]);

	s_voltage[1] = *(v_adc + 1);
	s_voltage[2] = *(v_adc + 2);
	s_voltage[3] = *(v_adc + 3);
}

void APP_Step(void)
{
	ENC_Update(&s_enc);

	uint16_t adc1 = (uint16_t) (s_iPacked & 0xFFFF);
	uint16_t adc2 = (uint16_t) ((s_iPacked >> 16) & 0xFFFF);

	q16_t ia = adc_to_q16(adc1);
	q16_t ib = adc_to_q16(adc2);
	q16_t ic = q16_sub_sat(0, q16_add_sat(ia, ib));

	q16_t ialpha, ibeta;
	clarke_q16(ia, ib, ic, &ialpha, &ibeta);

	q16_t v_alpha = s_foc.v_alpha_q16;
	q16_t v_beta = s_foc.v_beta_q16;

	BEMF_PLL_Step(&s_pll, v_alpha, v_beta, ialpha, ibeta);

	FOC_CurrentLoopStep(&s_foc, ia, ib, ic, s_pll.theta_q16);

	q16_t thr01 = throttle_shape_q16(s_enc.current_q16);

	if (!s_mode_speed)
	{
		/* トルク（Iq）直結：0..1 → 0..IQ_MAX */
		s_foc.Iq_ref_q16 = q16_mul(thr01, IQ_MAX_Q16);
	}
	else
	{
		/* 速度モード：0..1 → 0..OMEGA_REF_MAX_STEP */
		q16_t omega_ref = q16_mul(thr01, CONF_OMEGA_STEP_MAX_Q16);
		/* 測定ωは PLLの o->omega_q16 をそのまま「turn/step」想定で使用 */
		q16_t omega_meas = s_pll.omega_q16;
		s_foc.Iq_ref_q16 = speed_pid_to_iq_q16(omega_ref, omega_meas);
	}

	uint16_t c1, c2, c3;
	FOC_AlphaBetaToSVPWM(&s_foc, &c1, &c2, &c3, (uint16_t) TIM1_ARR);

	/* === Startup control === */
	s_tick++;

	switch (s_st)
	{
	case ST_ALIGN:
		/* d磁化で固定（ロータ吸着） */
		s_foc.Id_ref_q16 = ST_ALIGN_ID_Q16;
		s_foc.Iq_ref_q16 = 0;
		/* 角はまだ使わないが、以後のために強制角を0付近に保持 */
		s_th_forced = 0;
		if (s_tick >= ST_ALIGN_TIME_TICKS)
		{
			s_st = ST_RAMP;
			s_tick = 0;
			s_iq_cmd = 0;
			s_omg_step = ST_OMEGA_STEP_INIT_Q16;
		}
		break;

	case ST_RAMP:
		/* Idは少し残す（コギング対策／起動補助）。慣れたら0に落としてOK */
		s_foc.Id_ref_q16 = ST_ALIGN_ID_Q16 >> 2; /* 1/4へ */
		/* Iq をスルーレートで増やす */
		if (s_iq_cmd < ST_RAMP_IQ_Q16)
			s_iq_cmd = q16_min(ST_RAMP_IQ_Q16,
					q16_add_sat(s_iq_cmd, ST_RAMP_DIDQ_TICK_Q16));
		s_foc.Iq_ref_q16 = s_iq_cmd;

		/* 強制角を回す（ωstepもゆっくり上げる） */
		if (s_omg_step < ST_OMEGA_STEP_MAX_Q16)
			s_omg_step = q16_min(ST_OMEGA_STEP_MAX_Q16,
					q16_add_sat(s_omg_step, ST_OMEGA_STEP_SLEW_Q16));
		s_th_forced = angle_wrap_q16(q16_add_sat(s_th_forced, s_omg_step));

		/* FOC側で使う角は「強制角」 */
		{
			q16_t s, c;
			sincos_q16(s_th_forced, &s, &c);
			/*
			 * 既存の FOC_CurrentLoopStep 呼び出しを “強制角” で上書きしたい場合は
			 * ここで再計算して inv_park→SVPWM まで流す or 既存thetaを差し替える実装に。
			 */
		}

		/* ハンドオフ条件：一定時間を過ぎ、かつ BEMFまたはPLL速度が閾値超え */
		if (s_tick >= ST_HANDOFF_MIN_TICKS)
		{
			if (q16_abs(s_pll.omega_q16) >= ST_HANDOFF_OMEGA_MIN
					|| emf_strength_q16(&s_pll) >= ST_HANDOFF_EMF_MIN)
			{
				s_st = ST_BLEND;
				s_tick = 0;
			}
		}
		if (s_tick >= ST_TIMEOUT_TICKS)
		{
			s_st = ST_FAIL;
		}
		break;

	case ST_BLEND:
	{
		/*
		 * 強制角→PLL角への滑らかな切替
		 * w = s_tick / ST_BLEND_TICKS (0→1), θ = (1-w)*θ_forced + w*θ_pll
		 */
		q16_t n = (s_tick >= ST_BLEND_TICKS) ? ST_BLEND_TICKS : s_tick;
		q16_t w = (q16_t) ((((int64_t) n << 31) + (ST_BLEND_TICKS / 2))
				/ ST_BLEND_TICKS);
		q16_t w1 = q16_sub_sat(Q16_ONE, w);

		q16_t th = q16_add_sat(q16_mul(w1, s_th_forced),
				q16_mul(w, s_pll.theta_q16));

		/* Id をゆっくり 0 へ、Iqは維持 */
		if (s_foc.Id_ref_q16 > 0)
		{
			q16_t step = ST_ALIGN_ID_Q16 >> 4;
			s_foc.Id_ref_q16 =
					(s_foc.Id_ref_q16 > step) ?
							q16_sub_sat(s_foc.Id_ref_q16, step) : 0;
		}

		/*
		 * θ=th を使って FOC を回す（既存のθ参照箇所をここで差し替える）
		 * sincos_q16(th, &s, &c); ... （以下略）
		 */

		if (s_tick >= ST_BLEND_TICKS)
		{
			s_st = ST_RUN;
		}
		s_tick++;
		(void) th;
	}
		break;

	case ST_RUN:
		/* 以降はPLL角・通常FOC
		 * 必要なら低速域のみ CCR4 を「T0中央」に寄せる条件を追加：
		 * if (q16_abs(s_pll.omega_q16) < ST_HANDOFF_OMEGA_MIN) { ccr4 = T0_center; }
		 */
		break;

	case ST_FAIL:
		/* 失敗時：安全停止（Iq=0, Id=0, 角固定/ゼロ）。必要なら再トライへ */
		s_foc.Id_ref_q16 = 0;
		s_foc.Iq_ref_q16 = 0;
		break;

	default:
		break;
	}

	FW_SetPWMDuties(c1, c2, c3);
}
