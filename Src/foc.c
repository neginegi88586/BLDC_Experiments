/*
 * foc.c
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#include "foc.h"
#include "fixedpoint.h"
#include "config.h"
#include "firmware.h"


static inline void park_q31(int32_t ialpha, int32_t ibeta, int32_t sin_t,
		int32_t cos_t, int32_t *id, int32_t *iq)
{
	int32_t d1 = q31_mul(cos_t, ialpha);
	int32_t d2 = q31_mul(sin_t, ibeta);
	*id = q31_add_sat(d1, d2);

	int32_t q1 = q31_mul(-sin_t, ialpha);
	int32_t q2 = q31_mul(cos_t, ibeta);
	*iq = q31_add_sat(q1, q2);
}

static inline void inv_park_q31(int32_t vd, int32_t vq, int32_t sin_t,
		int32_t cos_t, int32_t *valpha, int32_t *vbeta)
{
	int32_t a1 = q31_mul(cos_t, vd);
	int32_t a2 = q31_mul(-sin_t, vq);
	*valpha = q31_add_sat(a1, a2);

	int32_t b1 = q31_mul(sin_t, vd);
	int32_t b2 = q31_mul(cos_t, vq);
	*vbeta = q31_add_sat(b1, b2);
}

// --- SVM セクタ算出（αβ→abcの符号で判定, 1..6） ---
static inline uint8_t svm_sector_from_alphabeta_q31(int32_t v_alpha,
		int32_t v_beta)
{
	int32_t va = v_alpha;
	int32_t vb = q31_add_sat(q31_mul(-(Q31_HALF), v_alpha),
			q31_mul(Q31_SQRT3_OVER_2, v_beta));
	int32_t vc = q31_sub_sat(q31_mul(-(Q31_HALF), v_alpha),
			q31_mul(Q31_SQRT3_OVER_2, v_beta));

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

// --- T0,T1,T2 と相デューティ生成（半キャリア正規化, Q31） ---
static inline void svpwm_compute_T012_q31(int32_t v_alpha, int32_t v_beta,
		uint8_t sector, int32_t *T0_q31, int32_t *T1_q31, int32_t *T2_q31,
		int32_t *Da_q31, int32_t *Db_q31, int32_t *Dc_q31)
{
	// 逆Clarke（相対値でOK）
	int32_t va = v_alpha;
	int32_t vb = q31_add_sat(q31_mul(-(Q31_HALF), v_alpha),
			q31_mul(Q31_SQRT3_OVER_2, v_beta));
	int32_t vc = q31_sub_sat(q31_mul(-(Q31_HALF), v_alpha),
			q31_mul(Q31_SQRT3_OVER_2, v_beta));

	// 最小値で平行移動 → [0, ...]
	int32_t min_ab = (va < vb) ? va : vb;
	int32_t min_all = (min_ab < vc) ? min_ab : vc;
	int32_t da = q31_sub_sat(va, min_all);
	int32_t db = q31_sub_sat(vb, min_all);
	int32_t dc = q31_sub_sat(vc, min_all);

	int32_t sum = q31_add_sat(q31_add_sat(da, db), dc); // = T1+T2
	if (sum > Q31_ONE)
		sum = Q31_ONE; // クリップ

	int32_t offset = q31_mul(q31_sub_sat(Q31_ONE, sum), Q31_HALF);

	int32_t Da = q31_add_sat(da, offset);
	int32_t Db = q31_add_sat(db, offset);
	int32_t Dc = q31_add_sat(dc, offset);

	// 並び替えで T1/T2 を取得
	int32_t Dmax = Da, Dmid = Db, Dmin = Dc;
	if (Dmax < Dmid)
	{
		int32_t t = Dmax;
		Dmax = Dmid;
		Dmid = t;
	}
	if (Dmid < Dmin)
	{
		int32_t t = Dmid;
		Dmid = Dmin;
		Dmin = t;
	}
	if (Dmax < Dmid)
	{
		int32_t t = Dmax;
		Dmax = Dmid;
		Dmid = t;
	}

	int32_t T1 = q31_sub_sat(Dmid, Dmin);
	int32_t T2 = q31_sub_sat(Dmax, Dmid);
	int32_t T0 = q31_sub_sat(Q31_ONE, q31_add_sat(T1, T2));

	if (T0_q31)
		*T0_q31 = T0;
	if (T1_q31)
		*T1_q31 = T1;
	if (T2_q31)
		*T2_q31 = T2;
	if (Da_q31)
		*Da_q31 = Da;
	if (Db_q31)
		*Db_q31 = Db;
	if (Dc_q31)
		*Dc_q31 = Dc;
}

void FOC_Init(FOC_t *foc)
{
	foc->Id_ref_q31 = Q31_FROM_FLOAT(0.0f);
	foc->Iq_ref_q31 = Q31_FROM_FLOAT(0.0f);

	foc->Kp_d_q31 = Q31_FROM_FLOAT(0.1f);
	foc->Ki_d_q31 = Q31_FROM_FLOAT(0.01f);
	foc->Kp_q_q31 = Q31_FROM_FLOAT(0.1f);
	foc->Ki_q_q31 = Q31_FROM_FLOAT(0.01f);

	foc->Id_i_q31 = 0;
	foc->Iq_i_q31 = 0;

	foc->Vbus_q31 = Q31_FROM_FLOAT(24.0f);
	foc->v_alpha_q31 = 0;
	foc->v_beta_q31 = 0;
}

void FOC_CurrentLoopStep(FOC_t *foc, int32_t i_a_q31, int32_t i_b_q31,
		int32_t i_c_q31, int32_t theta_q31)
{
	int32_t i_alpha = i_a_q31;
	int32_t two_ib = q31_add_sat(i_b_q31, i_b_q31);
	int32_t sum = q31_add_sat(i_a_q31, two_ib);
	int32_t i_beta = q31_mul(sum, Q31_INV_SQRT3);

	int32_t s, c;
	sincos_q31(theta_q31, &s, &c);

	// Park
	int32_t d1 = q31_mul(c, i_alpha);
	int32_t d2 = q31_mul(s, i_beta);
	int32_t id = q31_add_sat(d1, d2);

	int32_t q1 = q31_mul(-s, i_alpha);
	int32_t q2 = q31_mul(c, i_beta);
	int32_t iq = q31_add_sat(q1, q2);

	// PI
	int32_t ed = q31_sub_sat(foc->Id_ref_q31, id);
	int32_t eq = q31_sub_sat(foc->Iq_ref_q31, iq);

	foc->Id_i_q31 = q31_add_sat(foc->Id_i_q31, q31_mul(foc->Ki_d_q31, ed));
	foc->Iq_i_q31 = q31_add_sat(foc->Iq_i_q31, q31_mul(foc->Ki_q_q31, eq));

	int32_t vd = q31_add_sat(q31_mul(foc->Kp_d_q31, ed), foc->Id_i_q31);
	int32_t vq = q31_add_sat(q31_mul(foc->Kp_q_q31, eq), foc->Iq_i_q31);

	// 逆Park
	int32_t a1 = q31_mul(c, vd);
	int32_t a2 = q31_mul(-s, vq);
	int32_t v_alpha = q31_add_sat(a1, a2);

	int32_t b1 = q31_mul(s, vd);
	int32_t b2 = q31_mul(c, vq);
	int32_t v_beta = q31_add_sat(b1, b2);

	foc->v_alpha_q31 = v_alpha;
	foc->v_beta_q31 = v_beta;

	// （任意）ここでセクタを参照できる
	volatile uint8_t sector = svm_sector_from_alphabeta_q31(v_alpha, v_beta);
	(void) sector;
}

void FOC_AlphaBetaToSVPWM(FOC_t *foc, uint16_t *ccr1, uint16_t *ccr2,
		uint16_t *ccr3, uint16_t arr)
{
	// vαβ から T0,T1,T2 と相デューティを生成（半キャリア正規化）
	uint8_t sector = svm_sector_from_alphabeta_q31(foc->v_alpha_q31,
			foc->v_beta_q31);

	int32_t T0, T1, T2, Da, Db, Dc; // Q31
	svpwm_compute_T012_q31(foc->v_alpha_q31, foc->v_beta_q31, sector, &T0, &T1,
			&T2, &Da, &Db, &Dc);

	// デューティを [0..ARR] へマップ
	uint16_t d1 = (uint16_t) ((((int64_t) Da) * arr) >> 31);
	uint16_t d2 = (uint16_t) ((((int64_t) Db) * arr) >> 31);
	uint16_t d3 = (uint16_t) ((((int64_t) Dc) * arr) >> 31);

	*ccr1 = d1;
	*ccr2 = d2;
	*ccr3 = d3;

	// --- CCR4（サンプルマーカ）自動更新 ---
	// 半キャリア: T0/2, T1, T2, T0/2 → 長いアクティブベクトル中央に配置
	int32_t Tmid_center = 0;
	if (T1 >= T2)
	{
		Tmid_center = q31_add_sat(q31_mul(T0, Q31_HALF), q31_mul(T1, Q31_HALF));
	}
	else
	{
		int32_t t0h = q31_mul(T0, Q31_HALF);
		int32_t t2h = q31_mul(T2, Q31_HALF);
		Tmid_center = q31_add_sat(q31_add_sat(t0h, T1), t2h);
	}

	// セーフティマージン（例：2%）
	int32_t margin = Q31_MARGIN_2PCT;
	// 必要なら Tmid_center にマージン調整を加える（ここでは中心不変）

	uint16_t c4 = (uint16_t) ((((int64_t) Tmid_center) * arr) >> 31);
	FW_SetSampleMarker(c4);

	(void)margin;
}
