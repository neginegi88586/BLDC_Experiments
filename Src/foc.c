/* foc.c
 * 目的：
 *   FOC の電流ループ部を Q16.16/SI単位PID に置換（外形はそのままq16）。
 *   - 入出力は既存q16のまま（-1..+1）で互換性を維持。
 *   - 内部で Id/Iq を[A]に変換し、PID(D-on-meas)＋ソフトスタート＋スルーレートを適用。
 * 注意：
 *   係数（I_MAX_A など）は実機に合わせて調整。
 */

#include "foc.h"
#include "config.h"
#include "firmware.h"
#include "app.h"
/* 追加：Q16.16 制御ユーティリティ */
#include "fixed_q16.h"
#include "pid_q16.h"
#include "slew_q16.h"
#include "softstart_q16.h"


static inline void park_q16(q16_t ialpha, q16_t ibeta, q16_t sin_t,
		q16_t cos_t, q16_t *id, q16_t *iq)
{
	q16_t d1 = q16_mul(cos_t, ialpha);
	q16_t d2 = q16_mul(sin_t, ibeta);
	*id = q16_add_sat(d1, d2);

	q16_t q1 = q16_mul(-sin_t, ialpha);
	q16_t q2 = q16_mul(cos_t, ibeta);
	*iq = q16_add_sat(q1, q2);
}

static inline void inv_park_q16(q16_t vd, q16_t vq, q16_t sin_t,
		q16_t cos_t, q16_t *valpha, q16_t *vbeta)
{
	q16_t a1 = q16_mul(cos_t, vd);
	q16_t a2 = q16_mul(-sin_t, vq);
	*valpha = q16_add_sat(a1, a2);

	q16_t b1 = q16_mul(sin_t, vd);
	q16_t b2 = q16_mul(cos_t, vq);
	*vbeta = q16_add_sat(b1, b2);
}

// --- SVM セクタ算出（αβ→abcの符号で判定, 1..6） ---
static inline uint8_t svm_sector_from_alphabeta_q16(q16_t v_alpha,
		q16_t v_beta)
{
	q16_t va = v_alpha;
	q16_t vb = q16_add_sat(q16_mul(-(Q16_HALF), v_alpha),
			q16_mul(Q16_SQRT3_OVER_2, v_beta));
	q16_t vc = q16_sub_sat(q16_mul(-(Q16_HALF), v_alpha),
			q16_mul(Q16_SQRT3_OVER_2, v_beta));

	uint8_t a = (va > 0);
	uint8_t b = (vb > 0);
	uint8_t c = (vc > 0);
	uint8_t code = (a << 2) | (b << 1) | c;

	switch (code)
	{
	case 0b110:
		return 1;
	case 0b101:
		return 2;
	case 0b011:
		return 3;
	case 0b010:
		return 4;
	case 0b001:
		return 5;
	case 0b100:
		return 6;
	case 0b111:
	case 0b000:
	default:
		return 1;
	}
}

// --- T0,T1,T2 と相デューティ生成（半キャリア正規化, q16） ---
static inline void svpwm_compute_T012_q16(q16_t v_alpha, q16_t v_beta,
		uint8_t sector, q16_t *T0_q16, q16_t *T1_q16, q16_t *T2_q16,
		q16_t *Da_q16, q16_t *Db_q16, q16_t *Dc_q16)
{
	// 逆Clarke（相対値でOK）
	q16_t va = v_alpha;
	q16_t vb = q16_add_sat(q16_mul(-(Q16_HALF), v_alpha),
			q16_mul(Q16_SQRT3_OVER_2, v_beta));
	q16_t vc = q16_sub_sat(q16_mul(-(Q16_HALF), v_alpha),
			q16_mul(Q16_SQRT3_OVER_2, v_beta));

	// 最小値で平行移動 → [0, ...]
	q16_t min_ab = (va < vb) ? va : vb;
	q16_t min_all = (min_ab < vc) ? min_ab : vc;
	q16_t da = q16_sub_sat(va, min_all);
	q16_t db = q16_sub_sat(vb, min_all);
	q16_t dc = q16_sub_sat(vc, min_all);

	q16_t sum = q16_add_sat(q16_add_sat(da, db), dc); // = T1+T2
	if (sum > Q16_ONE)
		sum = Q16_ONE; // クリップ

	q16_t offset = q16_mul(q16_sub_sat(Q16_ONE, sum), Q16_HALF);

	q16_t Da = q16_add_sat(da, offset);
	q16_t Db = q16_add_sat(db, offset);
	q16_t Dc = q16_add_sat(dc, offset);

	// 並び替えで T1/T2 を取得
	q16_t Dmax = Da, Dmid = Db, Dmin = Dc;
	if (Dmax < Dmid)
	{
		q16_t t = Dmax;
		Dmax = Dmid;
		Dmid = t;
	}
	if (Dmid < Dmin)
	{
		q16_t t = Dmid;
		Dmid = Dmin;
		Dmin = t;
	}
	if (Dmax < Dmid)
	{
		q16_t t = Dmax;
		Dmax = Dmid;
		Dmid = t;
	}

	q16_t T1 = q16_sub_sat(Dmid, Dmin);
	q16_t T2 = q16_sub_sat(Dmax, Dmid);
	q16_t T0 = q16_sub_sat(Q16_ONE, q16_add_sat(T1, T2));

	if (T0_q16)
		*T0_q16 = T0;
	if (T1_q16)
		*T1_q16 = T1;
	if (T2_q16)
		*T2_q16 = T2;
	if (Da_q16)
		*Da_q16 = Da;
	if (Db_q16)
		*Db_q16 = Db;
	if (Dc_q16)
		*Dc_q16 = Dc;
}

void FOC_Init(FOC_t *foc)
{
	foc->Id_ref_q16 = 0;
	foc->Iq_ref_q16 = 0;

	foc->Kp_d_q16 = Q16_FRAC(1, 10);
	foc->Ki_d_q16 = 0;
	foc->Kp_q_q16 = Q16_FRAC(1, 10);
	foc->Ki_q_q16 = 0;

	foc->Id_i_q16 = 0;
	foc->Iq_i_q16 = 0;

	foc->Vbus_q16 = Q16_FRAC(12, 1);
	foc->v_alpha_q16 = 0;
	foc->v_beta_q16 = 0;
}

void FOC_CurrentLoopStep(FOC_t *foc, q16_t i_a_q16, q16_t i_b_q16,
		q16_t i_c_q16, q16_t theta_q16)
{
	q16_t vd;
	q16_t vq;
	q16_t i_alpha = i_a_q16;
	q16_t two_ib = q16_add_sat(i_b_q16, i_b_q16);
	q16_t sum = q16_add_sat(i_a_q16, two_ib);
	q16_t i_beta = q16_mul(sum, Q16_INV_SQRT3);

	q16_t s, c;
	sincos_q16(theta_q16, &s, &c);

	// Park
	q16_t d1 = q16_mul(c, i_alpha);
	q16_t d2 = q16_mul(s, i_beta);
	q16_t id = q16_add_sat(d1, d2);

	q16_t q1 = q16_mul(-s, i_alpha);
	q16_t q2 = q16_mul(c, i_beta);
	q16_t iq = q16_add_sat(q1, q2);

       /* ブロックコメント：
        * PID 制御（Q16.16, SI単位）
        * - q16 の Id/Iq [-1..1] を 実電流[A] に変換（I_MAX_A を係数として使用）
        * - PID(D-on-meas) で Vd/Vq の正規化指令[-1..1]を生成
        * - ソフトスタート係数とスルーレート制限を適用
        */
       {
               static const q16_t I_MAX_A_Q16 = (q16_t)(10 << 16); /* フルスケール電流[A]（例：10A）*/
               static const q16_t SLEW_UP_Q16 = (1 << 16);           /* 1.0 / s */
               static const q16_t SLEW_DN_Q16 = (3 << 16);           /* 3.0 / s */
               static const q16_t DT_Q16      = (q16_t)((1<<16) / PWM_FREQ_HZ); /* 制御周期[s] */

               static pid_q16_t pid_d, pid_q;
               static softstart_t  ss;
               static int          inited = 0;
               static q16_t      u_d_prev = 0, u_q_prev = 0;

               if (!inited)
               {
                       pid_q16_init(&pid_d);
                       pid_q16_init(&pid_q);
                       pid_d.kp = (3<<16); pid_d.ki = (40<<16); pid_d.kd = 0;
                       pid_q.kp = (3<<16); pid_q.ki = (40<<16); pid_q.kd = 0;
                       pid_d.out_min = -(1<<16); pid_d.out_max = (1<<16);
                       pid_q.out_min = -(1<<16); pid_q.out_max = (1<<16);
                       softstart_init(&ss, (q16_t)(0.3 * 65536)); /* 0.3s */
                       softstart_enable(&ss, 1);
                       inited = 1;
               }

               /* Id/Iq 実測と参照を Q16.16[A] へ変換 */
               q16_t id_A_q16     = (q16_t)(( (int64_t)id >> 15) * I_MAX_A_Q16 >> 16);
               q16_t iq_A_q16     = (q16_t)(( (int64_t)iq >> 15) * I_MAX_A_Q16 >> 16);
               q16_t Id_ref_A_q16 = (q16_t)(( (int64_t)foc->Id_ref_q16 >> 15) * I_MAX_A_Q16 >> 16);
               q16_t Iq_ref_A_q16 = (q16_t)(( (int64_t)foc->Iq_ref_q16 >> 15) * I_MAX_A_Q16 >> 16);

               /* ソフトスタートで Iq_ref を段階的に上げる */
               q16_t ss_gain = softstart_step(&ss, DT_Q16);
               Iq_ref_A_q16 = (q16_t)(( (int64_t)Iq_ref_A_q16 * ss_gain) >> 16);

               /* PID 実行（出力は正規化電圧[-1..1]の Q16.16）*/
               q16_t u_d_q16 = pid_q16_step(&pid_d, Id_ref_A_q16, id_A_q16, DT_Q16);
               q16_t u_q_q16 = pid_q16_step(&pid_q, Iq_ref_A_q16, iq_A_q16, DT_Q16);

               /* スルーレート制限 */
               u_d_q16 = q16_slew_step(u_d_prev, u_d_q16, SLEW_UP_Q16, SLEW_DN_Q16, DT_Q16);
               u_q_q16 = q16_slew_step(u_q_prev, u_q_q16, SLEW_UP_Q16, SLEW_DN_Q16, DT_Q16);
               u_d_prev = u_d_q16; u_q_prev = u_q_q16;

               /* q16 へ戻す（-1..1） */
               vd = (q16_t)(( (int64_t)u_d_q16 ) << 15);
               vq = (q16_t)(( (int64_t)u_q_q16 ) << 15);
               /* 逆Parkへ渡すためのローカル上書き */
               /* 以下の逆Park計算はこの vd/vq を使用 */
               /* 注意：既存の Ki/Kp を使わないため foc->Id_i_q16 等は以降未使用 */

               /* 逆Park用にスコープ外で参照される変数名と整合させる */
               /* vd, vq ローカルをこの後の逆Park計算に使用 */
               /* ↓（そのまま下の逆Parkへ連続） */
               /* fallthrough */
       }

	// 逆Park
	q16_t a1 = q16_mul(c, vd);
	q16_t a2 = q16_mul(-s, vq);
	q16_t v_alpha = q16_add_sat(a1, a2);

	q16_t b1 = q16_mul(s, vd);
	q16_t b2 = q16_mul(c, vq);
	q16_t v_beta = q16_add_sat(b1, b2);

	foc->v_alpha_q16 = v_alpha;
	foc->v_beta_q16 = v_beta;

	// （任意）ここでセクタを参照できる
	volatile uint8_t sector = svm_sector_from_alphabeta_q16(v_alpha, v_beta);
	(void) sector;
}

void FOC_AlphaBetaToSVPWM(FOC_t *foc, uint16_t *ccr1, uint16_t *ccr2,
		uint16_t *ccr3, uint16_t arr)
{
	// vαβ から T0,T1,T2 と相デューティを生成（半キャリア正規化）
	uint8_t sector = svm_sector_from_alphabeta_q16(foc->v_alpha_q16,
			foc->v_beta_q16);

	q16_t T0, T1, T2, Da, Db, Dc; // q16
	svpwm_compute_T012_q16(foc->v_alpha_q16, foc->v_beta_q16, sector, &T0, &T1,
			&T2, &Da, &Db, &Dc);

	// デューティを [0..ARR] へマップ
	uint16_t d1 = (uint16_t) ((((int64_t) Da) * arr) >> 16);
	uint16_t d2 = (uint16_t) ((((int64_t) Db) * arr) >> 16);
	uint16_t d3 = (uint16_t) ((((int64_t) Dc) * arr) >> 16);

	*ccr1 = d1;
	*ccr2 = d2;
	*ccr3 = d3;

	// --- CCR4（サンプルマーカ）自動更新 ---
	// 半キャリア: T0/2, T1, T2, T0/2 → 長いアクティブベクトル中央に配置
	q16_t Tmid_center = 0;
	if (T1 >= T2)
	{
		Tmid_center = q16_add_sat(q16_mul(T0, Q16_HALF), q16_mul(T1, Q16_HALF));
	}
	else
	{
		q16_t t0h = q16_mul(T0, Q16_HALF);
		q16_t t2h = q16_mul(T2, Q16_HALF);
		Tmid_center = q16_add_sat(q16_add_sat(t0h, T1), t2h);
	}

	// セーフティマージン（例：2%）
	q16_t margin = Q16_MARGIN_2PCT;
	// 必要なら Tmid_center にマージン調整を加える（ここでは中心不変）

	uint16_t c4 = (uint16_t) ((((int64_t) Tmid_center) * arr) >> 31);
	FW_SetSampleMarker(c4);

	(void)margin;
}
