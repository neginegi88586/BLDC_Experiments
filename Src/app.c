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
#include "fixedpoint.h"
#include "encoder.h"
/* 追加：Q16.16 ユーティリティ */
#include "Drivers/BLDC_Lib/Inc/fixed_q16_16.h"
#include "Drivers/BLDC_Lib/Inc/adc_vcal_q16_16.h"
#include "Drivers/BLDC_Lib/Inc/units_q16_16.h"

/* 較正状態（他の翻訳単位から参照される） */
adc_vcal_t g_vcal;


// ====== CORDIC: 係数も完全整数化 ======
#define CORDIC_ITERS 12
static const int32_t k_cordic_K_q31 = Q31_FRAC(607252935, 1000000000); // ≈0.607252935


// atan(2^-i)/(2π) を i に対して **線形補間** で近似する関数
// アンカー（i=0,4,8,12）を分数→Q31で整数定義し、i間は単純Lerp
static const struct
{
	uint8_t i;
	int32_t v;
} k_atan_anchor[] =
{
		{ 0, Q31_FRAC(125000000, 1000000000) }, // 0.1250000000
		{ 4, Q31_FRAC(993426215, 100000000000) }, // 0.00993426215
		{ 8, Q31_FRAC(62169583, 100000000000) }, // 0.00062169583
		{ 12, Q31_FRAC(3885619, 100000000000) } // 0.00003885619
};


// --- Startup state machine ---
typedef enum { ST_STOP=0, ST_ALIGN, ST_RAMP, ST_BLEND, ST_RUN, ST_FAIL } st_t;
static st_t     s_st = ST_STOP;
static int32_t  s_th_forced = 0;            // 強制角 (turn-Q31)
static int32_t  s_omg_step  = 0;            // 1周期あたりのΔθ
static int32_t  s_iq_cmd    = 0;            // 開ループ中の Iq 指令
static uint32_t s_tick      = 0;            // 経過tick

static inline int32_t q31_abs(int32_t x){ return (x>=0)?x:-(x); }
static inline int32_t q31_min(int32_t a,int32_t b){ return (a<b)?a:b; }
static inline int32_t q31_max(int32_t a,int32_t b){ return (a>b)?a:b; }

// 推定BEMFの強さの簡易指標: |e| ≈ max(|eα|,|eβ|)
static inline int32_t emf_strength_q31(const BEMF_PLL_t* o){
    int32_t ea = q31_abs(o->e_alpha_q31);
    int32_t eb = q31_abs(o->e_beta_q31);
    return (ea>eb)?ea:eb;
}


static inline int32_t atan_turn_i_q31(int i)
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
			int32_t y0 = k_atan_anchor[k].v;
			int32_t y1 = k_atan_anchor[k + 1].v;
			int32_t di = (int32_t) (i - i0);
			int32_t wi = (int32_t) (i1 - i0);
			int32_t dy = q31_sub_sat(y1, y0);
			int32_t t = (int32_t) ((((int64_t) di << 31) + (wi / 2)) / wi); // di/wi をQ31に
			return q31_add_sat(y0, q31_mul(dy, t));
		}
	}
	return k_atan_anchor[3].v;
}

void sincos_q31(int32_t th, int32_t *s, int32_t *c)
{
	th = angle_wrap_q31(th);
	if (th > Q31_ONE / 2)
		th -= (Q31_ONE + 1);
	if (th <= -Q31_ONE / 2)
		th += (Q31_ONE + 1);

	int8_t q = 0;
	if (th > Q31_ONE / 4)
	{
		th -= (Q31_ONE / 2);
		q = +1;
	}
	else if (th < -Q31_ONE / 4)
	{
		th += (Q31_ONE / 2);
		q = -1;
	}

	int32_t x = k_cordic_K_q31; // cos
	int32_t y = 0; // sin
	int32_t z = th; // turn-Q31

	for (int i = 0; i < CORDIC_ITERS; i++)
	{
		int32_t angle_i = atan_turn_i_q31(i); // 線形補間で取得
		int32_t dx = (y >> i);
		int32_t dy = (x >> i);
		if (z >= 0)
		{
			x = q31_sub_sat(x, dx);
			y = q31_add_sat(y, dy);
			z = q31_sub_sat(z, angle_i);
		}
		else
		{
			x = q31_add_sat(x, dx);
			y = q31_sub_sat(y, dy);
			z = q31_add_sat(z, angle_i);
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

// ====== アプリ層本体 ======
static FOC_t s_foc;
static BEMF_PLL_t s_pll;

static int32_t s_thr_filt_q31 = 0;     // LPF後の0..1
static int32_t s_mode_speed = 0;       // 0=トルク直結, 1=速度PI
static int32_t s_speed_int_q31 = 0;    // 速度PIの積分

static volatile uint16_t s_voltage[2];
static volatile uint16_t s_vphase_adc[4];
static volatile uint16_t s_current[3];
static volatile uint32_t s_iPacked;

uint16_t bat_voltage_buff;

uint16_t motor_voltage_buff[4];
int16_t motor_current_buff[3];


extern  Encoder_t s_enc;


static inline int32_t adc_to_q31(uint16_t v)
{
	return (int32_t) ((int64_t) v * (int64_t) (Q31_ONE >> 11));
}

static inline void clarke_q31(int32_t ia, int32_t ib, int32_t ic,
		int32_t *ialpha, int32_t *ibeta)
{
	*ialpha = ia;
	int32_t two_ib = q31_add_sat(ib, ib);
	int32_t sum = q31_add_sat(ia, two_ib);
	*ibeta = q31_mul(sum, Q31_INV_SQRT3);
}

static inline int32_t throttle_shape_q31(int32_t thr_raw_q31)
{
    // デッドバンド
    if (thr_raw_q31 < THR_DEADBAND_Q31) thr_raw_q31 = 0;

    // LPF: y = αx + (1-α)y
    int32_t one_minus_a = q31_sub_sat(Q31_ONE, THR_LPF_ALPHA_Q31);
    s_thr_filt_q31 = q31_add_sat(q31_mul(THR_LPF_ALPHA_Q31, thr_raw_q31),
                                 q31_mul(one_minus_a,        s_thr_filt_q31));

    // スルーレート（入力の瞬間変化を更に縛りたい場合は、前回値を覚えて制限）
    static int32_t prev = 0;
    int32_t diff = q31_sub_sat(s_thr_filt_q31, prev);
    if (diff > THR_SLEW_PER_TICK_Q31)      s_thr_filt_q31 = q31_add_sat(prev, THR_SLEW_PER_TICK_Q31);
    else if (diff < -THR_SLEW_PER_TICK_Q31) s_thr_filt_q31 = q31_sub_sat(prev, THR_SLEW_PER_TICK_Q31);
    prev = s_thr_filt_q31;

    // 上限
    if (s_thr_filt_q31 > THR_ADC_MAX_Q31) s_thr_filt_q31 = THR_ADC_MAX_Q31;
    return s_thr_filt_q31;
}

// 速度PI（PLLのωを使って Iq_ref を作る）
static inline int32_t speed_pi_to_iq_q31(int32_t omega_ref_step_q31, int32_t omega_meas_step_q31)
{
    int32_t e = q31_sub_sat(omega_ref_step_q31, omega_meas_step_q31);
    s_speed_int_q31 = q31_add_sat(s_speed_int_q31, q31_mul(SPEED_KI_Q31, e));

    // 積分アンチワインドアップ：Iqの範囲に収める
    if (s_speed_int_q31 > IQ_MAX_Q31) s_speed_int_q31 = IQ_MAX_Q31;
    if (s_speed_int_q31 < 0)          s_speed_int_q31 = 0; // 順回転限定なら0～に

    int32_t out = q31_add_sat(q31_mul(SPEED_KP_Q31, e), s_speed_int_q31);

    // 出力リミット
    if (out > IQ_MAX_Q31) out = IQ_MAX_Q31;
    if (out < 0)          out = 0;
    return out; // Iq_ref
}


void APP_Init(void)
{
        FOC_Init(&s_foc);
        BEMF_PLL_Init(&s_pll);

       /* ブロックコメント：
        * ADC 較正の初期化。
        * v_ref_in = 1.235V, alpha = 0.1（1kHz更新なら時定数約9ms）
        */
       adc_vcal_init(&g_vcal, q16_16_from_frac(1235, 1000), q16_16_from_frac(1, 10));

        s_pll.Ts_q31 = (int32_t) (((int64_t) 1 << 31) / (int64_t) PWM_FREQ_HZ);
	s_pll.Rs_q31 = CONF_RS_Q31;
	s_pll.Ls_q31 = CONF_LS_Q31;
	s_pll.alpha_q31 = CONF_OBS_ALPHA_Q31;
	s_pll.kp_q31 = CONF_PLL_KP_Q31;
	s_pll.ki_q31 = CONF_PLL_KI_Q31;
	s_pll.omega_min_q31 = CONF_OMEGA_STEP_MIN_Q31;
	s_pll.omega_max_q31 = CONF_OMEGA_STEP_MAX_Q31;
	s_pll.integ_min_q31 = CONF_PLL_INT_MIN_Q31;
	s_pll.integ_max_q31 = CONF_PLL_INT_MAX_Q31;

	s_st = ST_ALIGN;
	s_tick = 0;
	s_th_forced = 0;
	s_omg_step  = ST_OMEGA_STEP_INIT_Q31;
	s_iq_cmd    = 0;
}

void APP_OnCurrents(uint16_t iU, uint16_t iV, uint16_t iW)
{
	s_iPacked = (uint32_t)(iV << 16) | (uint32_t)(iU);

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
       /* ブロックコメント：
        * ここでは v_adc[0] を PA0(1.235V) と仮定して較正更新を行う。
        * プロジェクトでの実CH割付に合わせてインデックスを調整すること。
        */
       s_voltage[0] = *v_adc;
       adc_vcal_update(&g_vcal, (int32_t)s_voltage[0]);

       s_voltage[1] = *(v_adc + 1);
       s_voltage[2] = *(v_adc + 2);
       s_voltage[3] = *(v_adc + 3);
}

void APP_Step(void)
{
	uint8_t ocp_flag = 0;
	APP_VoltageConv();
	APP_CurrentConv();

	for(size_t i = 0; i < 3; i++)
	{
		if(motor_current_buff[i] > I_MAX)
		{
			ocp_flag = 1;
		}
	}

	if(ocp_flag != 0)
	{
		FW_SetPWMDuties(0, 0, 0);
		APP_Init();
	}
	else
	{
		ENC_Update(&s_enc);

		uint16_t adc1 = (uint16_t)(s_iPacked & 0xFFFF);
		uint16_t adc2 = (uint16_t)((s_iPacked >> 16) & 0xFFFF);

		int32_t ia = adc_to_q31(adc1);
		int32_t ib = adc_to_q31(adc2);
		int32_t ic = q31_sub_sat(0, q31_add_sat(ia, ib));

		int32_t ialpha, ibeta;
		clarke_q31(ia, ib, ic, &ialpha, &ibeta);

		int32_t v_alpha = s_foc.v_alpha_q31;
		int32_t v_beta = s_foc.v_beta_q31;

		BEMF_PLL_Step(&s_pll, v_alpha, v_beta, ialpha, ibeta);

		FOC_CurrentLoopStep(&s_foc, ia, ib, ic, s_pll.theta_q31);

	    int32_t thr01 = throttle_shape_q31(s_enc.current_q31);

	    if (!s_mode_speed)
	    {
	        // トルク（Iq）直結：0..1 → 0..IQ_MAX
	        s_foc.Iq_ref_q31 = q31_mul(thr01, IQ_MAX_Q31);
	    }
	    else
	    {
	        // 速度モード：0..1 → 0..OMEGA_REF_MAX_STEP
	        int32_t omega_ref = q31_mul(thr01, CONF_OMEGA_STEP_MAX_Q31);
	        // 測定ωは PLLの o->omega_q31 をそのまま「turn/step」想定で使用
	        int32_t omega_meas = s_pll.omega_q31;
	        s_foc.Iq_ref_q31 = speed_pi_to_iq_q31(omega_ref, omega_meas);
	    }

		uint16_t c1, c2, c3;
		FOC_AlphaBetaToSVPWM(&s_foc, &c1, &c2, &c3, (uint16_t) TIM1_ARR);

		// === Startup control ===
		s_tick++;

		switch (s_st)
		{
		case ST_ALIGN:
		    // d磁化で固定（ロータ吸着）
		    s_foc.Id_ref_q31 = ST_ALIGN_ID_Q31;
		    s_foc.Iq_ref_q31 = 0;
		    // 角はまだ使わないが、以後のために強制角を0付近に保持
		    s_th_forced = 0;
		    if (s_tick >= ST_ALIGN_TIME_TICKS) {
		        s_st   = ST_RAMP;
		        s_tick = 0;
		        s_iq_cmd   = 0;
		        s_omg_step = ST_OMEGA_STEP_INIT_Q31;
		    }
		    break;

		case ST_RAMP:
		    // Idは少し残す（コギング対策／起動補助）。慣れたら0に落としてOK
		    s_foc.Id_ref_q31 = ST_ALIGN_ID_Q31 >> 2; // 1/4へ
		    // Iq をスルーレートで増やす
		    if (s_iq_cmd < ST_RAMP_IQ_Q31)
		        s_iq_cmd = q31_min(ST_RAMP_IQ_Q31, q31_add_sat(s_iq_cmd, ST_RAMP_DIDQ_TICK_Q31));
		    s_foc.Iq_ref_q31 = s_iq_cmd;

		    // 強制角を回す（ωstepもゆっくり上げる）
		    if (s_omg_step < ST_OMEGA_STEP_MAX_Q31)
		        s_omg_step = q31_min(ST_OMEGA_STEP_MAX_Q31, q31_add_sat(s_omg_step, ST_OMEGA_STEP_SLEW_Q31));
		    s_th_forced = angle_wrap_q31(q31_add_sat(s_th_forced, s_omg_step));

		    // FOC側で使う角は「強制角」
		    {
		        int32_t s,c; sincos_q31(s_th_forced,&s,&c);
		        // 既存の FOC_CurrentLoopStep 呼び出しを “強制角” で上書きしたい場合は
		        // ここで再計算して inv_park→SVPWM まで流す or 既存thetaを差し替える実装に。
		    }

		    // ハンドオフ条件：一定時間を過ぎ、かつ BEMFまたはPLL速度が閾値超え
		    if (s_tick >= ST_HANDOFF_MIN_TICKS) {
		        if (q31_abs(s_pll.omega_q31) >= ST_HANDOFF_OMEGA_MIN ||
		            emf_strength_q31(&s_pll)  >= ST_HANDOFF_EMF_MIN) {
		            s_st   = ST_BLEND;
		            s_tick = 0;
		        }
		    }
		    if (s_tick >= ST_TIMEOUT_TICKS) { s_st = ST_FAIL; }
		    break;

		case ST_BLEND: {
		    // 強制角→PLL角への滑らかな切替
		    // w = s_tick / ST_BLEND_TICKS (0→1), θ = (1-w)*θ_forced + w*θ_pll
		    uint32_t n = (s_tick >= ST_BLEND_TICKS)? ST_BLEND_TICKS : s_tick;
		    int32_t w  = (int32_t)((((int64_t)n << 31) + (ST_BLEND_TICKS/2)) / ST_BLEND_TICKS);
		    int32_t w1 = q31_sub_sat(Q31_ONE, w);

		    int32_t th = q31_add_sat(q31_mul(w1, s_th_forced), q31_mul(w, s_pll.theta_q31));

		    // Id をゆっくり 0 へ、Iqは維持
		    if (s_foc.Id_ref_q31 > 0) {
		        int32_t step = ST_ALIGN_ID_Q31 >> 4;
		        s_foc.Id_ref_q31 = (s_foc.Id_ref_q31 > step) ? q31_sub_sat(s_foc.Id_ref_q31, step) : 0;
		    }

		    // θ=th を使って FOC を回す（既存のθ参照箇所をここで差し替える）
		    // sincos_q31(th, &s, &c); ... （以下略）

		    if (s_tick >= ST_BLEND_TICKS) { s_st = ST_RUN; }
		    s_tick++;
		    } break;

		case ST_RUN:
		    // 以降はPLL角・通常FOC
		    // 必要なら低速域のみ CCR4 を「T0中央」に寄せる条件を追加：
		    // if (q31_abs(s_pll.omega_q31) < ST_HANDOFF_OMEGA_MIN) { ccr4 = T0_center; }
		    break;

		case ST_FAIL:
		    // 失敗時：安全停止（Iq=0, Id=0, 角固定/ゼロ）。必要なら再トライへ
		    s_foc.Id_ref_q31 = 0;
		    s_foc.Iq_ref_q31 = 0;
		    break;

		default: break;
		}

		FW_SetPWMDuties(c1, c2, c3);
	}
}

void APP_VoltageConv(void)
{
	uint32_t v_phase[4];
	uint32_t v_ref = (uint32_t)((((uint64_t)AD_MAX / (uint64_t)(s_voltage[0] << 16)) * (uint64_t)V_REF_AD) >> 16);
	uint32_t v_battery = (uint32_t)((((((uint64_t)s_voltage[1] << 16 / (uint64_t)AD_MAX) * (uint64_t)v_ref) >> 16) * (uint64_t)V_DIV) >> 16);

	for(size_t i = 0; i < 4; i++)
	{
		v_phase[i] = (uint32_t)((((((uint64_t)s_vphase_adc[i] << 16 / (uint64_t)AD_MAX) * (uint64_t)v_ref) >> 16) * (uint64_t)V_DIV) >> 16);
		motor_voltage_buff[i] = (uint16_t)((v_phase[i] * 1000) >> 16);
	}

	bat_voltage_buff = (uint16_t)((v_battery * 1000) >> 16);
}

void APP_CurrentConv(void)
{
	int32_t i_phase;
	int32_t vi_phase;
	int32_t v_ref = (int32_t)((((int64_t)AD_MAX / (int64_t)(s_voltage[0] << 16)) * (int64_t)V_REF_AD) >> 16);

	for(size_t i = 0; i < 3; i++)
	{
		vi_phase =  (int32_t)((((int64_t)(s_current[i] << 16) / (int64_t)AD_MAX) * (int64_t)v_ref) >> 16);
		i_phase = (int32_t)((((((int64_t)(vi_phase - V_CENTER)) * 1000) / 11) << 16) / (int64_t)(R_SHUNT));
		motor_current_buff[i] = (int16_t)(i_phase >> 16);
	}
}
