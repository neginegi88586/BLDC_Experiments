/* firmware.c
 * 目的：
 *   TIM1 出力をアクティブロー（Pchハイサイド＋Nchローサイドとも 0=ON）に統一。
 *   アイドル状態も OFF となるよう OIS を設定。
 * 注意：
 *   既存のCCR計算はそのままで、極性のみハード側で反転。
 */

#include "config.h"
#include "firmware.h"
#include "encoder.h"
#include "app.h"
#include <stm32f4xx.h>


#define ADC_INJBUF_LEN 4

#define ADC_BUF_LEN 5
static volatile uint16_t s_AdcBuf[ADC_BUF_LEN];

volatile uint8_t count_flag = 0;
Encoder_t s_enc;

void FW_InitClocksAndGPIO(void)
{
	FW_InitClock();
	FW_InitGPIO();

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_ADC1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM7EN;
}

void FW_InitClock(void)
{
	RCC->CR |= RCC_CR_HSEON;
	while((RCC->CR) & RCC_CR_HSERDY)
	{
		/* 何もしない */
	}

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC;

	RCC->PLLCFGR &= (~(63 << RCC_PLLCFGR_PLLM_Pos));
	RCC->PLLCFGR |= (12 << RCC_PLLCFGR_PLLM_Pos);

	RCC->PLLCFGR &= (~(511 << RCC_PLLCFGR_PLLN_Pos));
	RCC->PLLCFGR |= (168 << RCC_PLLCFGR_PLLN_Pos);

	RCC->PLLCFGR &= (3 << RCC_PLLCFGR_PLLP_Pos);
	RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLP_Pos);

	RCC->PLLCFGR &= (~(15 << RCC_PLLCFGR_PLLQ_Pos));
	RCC->PLLCFGR |= (7 << RCC_PLLCFGR_PLLQ_Pos);

	RCC->CFGR |= (5 << RCC_CFGR_PPRE1_Pos);	/* 4分周 */
	RCC->CFGR |= (4 << RCC_CFGR_PPRE2_Pos);	/* 2分周 */

	RCC->CR |= RCC_CR_PLLON;
	while((RCC->CR) & RCC_CR_PLLRDY)
	{
		/* 何もしない */
	}

	uint32_t tmp = (RCC->CFGR);
	tmp &= (~(RCC_CFGR_SW_Msk));
	tmp |= (RCC_CFGR_SW_PLL);
	RCC->CFGR = tmp;
	while(((RCC->CFGR) & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
	{
		/* 何もしない */
	}

	SystemCoreClockUpdate();
}

void FW_InitGPIO(void)
{
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
			RCC_AHB1ENR_GPIODEN);

	GPIOA->MODER = 0x282Affff;
	GPIOA->OSPEEDR = 0x0C3F0000;
	GPIOA->AFR[0] = 0x00000000;
	GPIOA->AFR[1] = 0x00000111;

	GPIOB->MODER = 0xA800A003;
	GPIOB->OSPEEDR = 0xFC003000;
	GPIOB->AFR[0] = 0x77000000;
	GPIOB->AFR[1] = 0x11100000;

	GPIOD->MODER = 0x00000010;
	GPIOD->OSPEEDR = 0x00000000;
}

void FW_TIM1_InitPWM(void)
{
	TIM1->PSC = 0;
	TIM1->ARR = TIM1_ARR;

	TIM1->CR1 &= ~TIM_CR1_DIR;
	TIM1->CR1 &= ~TIM_CR1_CMS;
	TIM1->CR1 |= (1 << TIM_CR1_CMS_Pos); /* center-aligned mode 1 */
	TIM1->CR1 |= TIM_CR1_ARPE;

	/* 出力比較: CH1..3 = PWM1 + preload */
	TIM1->CCMR1 = 0;
	TIM1->CCMR1 |= (6<<TIM_CCMR1_OC1M_Pos)|TIM_CCMR1_OC1PE;
	TIM1->CCMR1 |= (6<<TIM_CCMR1_OC2M_Pos)|TIM_CCMR1_OC2PE;
	TIM1->CCMR2 = 0;
	TIM1->CCMR2 |= (6<<TIM_CCMR2_OC3M_Pos)|TIM_CCMR2_OC3PE;

	/* CH4 は内部トリガ用 (OC4REF)。ピン出力は使わない＝CC4Eは後で0のまま。 */
	/* ここでは強制アクティブ + preload で基準パルスを作る例 */
	TIM1->CCMR2 |= (7<<TIM_CCMR2_OC4M_Pos)|TIM_CCMR2_OC4PE;

	/* デューティ初期値（50%で開始推奨） */
	TIM1->CCR1 = TIM1_ARR/2;
	TIM1->CCR2 = TIM1_ARR/2;
	TIM1->CCR3 = TIM1_ARR/2;

	/* --- CCER: メイン＋コンプリメンタリを両方有効化 --- */
	/* 極性はまず非反転（H=ON）で開始。必要なら後述の「極性」参照。 */
	TIM1->CCER = 0;
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;  // CH1/CH1N
	TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE;  // CH2/CH2N
	TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NE;  // CH3/CH3N
	/* CH4ピンは無効のまま（OC4REFは内部利用） */
	/* 極性：アクティブロー（論理0=ON）に設定する */
	TIM1->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC1NP
			|  TIM_CCER_CC2P | TIM_CCER_CC2NP
			|  TIM_CCER_CC3P | TIM_CCER_CC3NP);
	/* 停止時のアイドル出力も LOW=OFF へ統一（OISx=0/OISxN=0） */
	TIM1->CR2 &= ~(TIM_CR2_OIS1|TIM_CR2_OIS1N|TIM_CR2_OIS2|TIM_CR2_OIS2N|TIM_CR2_OIS3|TIM_CR2_OIS3N);

	/* --- TRGO: OC4REF を外部へ（TIM3ブリッジ・ADC用） --- */
	TIM1->CR2 &= ~TIM_CR2_MMS;
	TIM1->CR2 |=  (7<<TIM_CR2_MMS_Pos);  /* TRGO = OC4REF */

	TIM1->BDTR = 0;
	TIM1->BDTR |= (DTG_TICKS << TIM_BDTR_DTG_Pos);
	TIM1->BDTR |= TIM_BDTR_OSSR | TIM_BDTR_OSSI;   /* ★強く推奨（停止/ブレークでOISレベルを適用）*/

	TIM1->BDTR |= TIM_BDTR_MOE;
}

void FW_TIM2_Init(void)
{
	TIM2->PSC = 8 - 1;
	TIM2->ARR = 1000 - 1;

	TIM2->DIER = 0x00000001;

	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);
}

void FW_TIM3_InitBridge(void)
{
	/* TIM1: TRGO=OC4REF（firmware内のInjected初期化で設定）*/
	TIM3->PSC = 0;
	TIM3->ARR = 1;

	TIM3->SMCR &= ~(TIM_SMCR_TS | TIM_SMCR_SMS);
	TIM3->SMCR |= (0 << TIM_SMCR_TS_Pos); /*TS = ITR0 (多くのF4で TIM1)*/
	TIM3->SMCR |= (6 << TIM_SMCR_SMS_Pos); /*Trigger mode: TRGI↑でUG*/

	TIM3->CR2 &= ~TIM_CR2_MMS;
	TIM3->CR2 |= (2 << TIM_CR2_MMS_Pos); /* TRGO=Update*/

	TIM3->EGR |= TIM_EGR_UG;
}

void FW_TIM7_Init(void)
{
	ENC_Init(&s_enc, ENC_STEP_Q16, ENC_MIN_Q16, ENC_MAX_Q16);

	TIM7->PSC = 420 - 1;
	TIM7->ARR = 1000 - 1;

	TIM7->DIER = 0x00000001;

	NVIC_SetPriority(TIM7_IRQn, 0);
	NVIC_EnableIRQ(TIM7_IRQn);
}

void FW_ADC1_Init(void)
{
	ADC1->SMPR1 = 0;
	ADC1->SMPR2 = ADC_SAMPLEING_TIME;

	FW_ADC12_InitDualRegular_TIM3_TRGO();
	FW_ADC1_InitInjected_TIM1_CC4();
	FW_DMA_InitForADC();
}

void FW_ADC12_InitDualRegular_TIM3_TRGO(void)
{
	ADC1->CR1 = 0;
	ADC1->CR2 = 0;
	ADC1->SQR1 = 0;
	ADC1->SQR3 = (ADC_CH_V_REF << 0);
	ADC1->SQR3 = (ADC_CH_V_BATT << 5);
	ADC1->SQR3 = (ADC_CH_I_U << 10);
	ADC1->SQR3 = (ADC_CH_I_V << 15);
	ADC1->SQR3 = (ADC_CH_I_W << 20);

	ADC1->CR2 &= ~(ADC_CR2_EXTSEL | ADC_CR2_EXTEN);
	ADC1->CR2 |= (ADC1_EXTSEL_TIM3_TRGO << ADC_CR2_EXTSEL_Pos);
	ADC1->CR2 |= (1 << ADC_CR2_EXTEN_Pos); /* Rising */

	ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;

	ADC1->CR2 |= ADC_CR2_ADON;
}

void FW_ADC1_InitInjected_TIM1_CC4(void)
{
	/* CC4: 内部OC4REFのみ生成 */
	TIM1->CCMR2 &= ~TIM_CCMR2_OC4M;
	TIM1->CCMR2 |= (7 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

	/* TIM1 TRGO = OC4REF（TIM3ブリッジにも有効） */
	TIM1->CR2 &= ~TIM_CR2_MMS;
	TIM1->CR2 |= (7 << TIM_CR2_MMS_Pos);

	/* Injected設定 */
	ADC1->JSQR = 0;
	ADC1->JSQR |= (3 << 20);
	ADC1->JSQR |= (ADC_CH_V_CC << 0);
	ADC1->JSQR |= (ADC_CH_V_U << 5);
	ADC1->JSQR |= (ADC_CH_V_V << 10);
	ADC1->JSQR |= (ADC_CH_V_W << 15);

	ADC1->CR2 &= ~(ADC_CR2_JEXTSEL | ADC_CR2_JEXTEN);
	ADC1->CR2 |= (ADC1_JEXTSEL_TIM1_CC4 << ADC_CR2_JEXTSEL_Pos);
	ADC1->CR2 |= (1 << ADC_CR2_JEXTEN_Pos); /* Rising */

	ADC1->CR1 |= ADC_CR1_JEOCIE;

	NVIC_SetPriority(ADC_IRQn, 0);
	NVIC_EnableIRQ(ADC_IRQn);
}

void FW_DMA_InitForADC(void)
{
	DMA2_Stream0->CR = 0;
	while (DMA2_Stream0->CR & DMA_SxCR_EN)
	{
	}

	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t) s_AdcBuf;
	DMA2_Stream0->NDTR = ADC_BUF_LEN;

	DMA2_Stream0->CR = (0 << DMA_SxCR_CHSEL_Pos) |
	DMA_SxCR_PL_1 |
	DMA_SxCR_MSIZE_0 | /* 16bit */
			DMA_SxCR_PSIZE_0 | /* 16bit */
			DMA_SxCR_MINC |
			DMA_SxCR_CIRC |
			DMA_SxCR_TCIE |
			(0 << DMA_SxCR_DIR_Pos);

	DMA2_Stream0->FCR = 0;

	NVIC_SetPriority(DMA2_Stream0_IRQn, 0);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	DMA2_Stream0->CR |= DMA_SxCR_EN;
}

void FW_StartAll(void)
{
	TIM1->EGR |= TIM_EGR_UG;
	TIM1->CR1 |= TIM_CR1_CEN;

	TIM2->CR1 |= TIM_CR1_CEN;

	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_CEN;

	TIM7->CR1 |= TIM_CR1_CEN;

	__enable_irq();
}

void FW_SetPWMDuties(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3)
{
	TIM1->CCR1 = ccr1;
	TIM1->CCR2 = ccr2;
	TIM1->CCR3 = ccr3;
}

void FW_SetSampleMarker(uint16_t ccr4)
{
	TIM1->CCR4 = ccr4;
}


/* ===== 割り込み ===== */
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void)
{
	ADC1->SR &= ~ADC_SR_EOC;
	ADC1->SR &= ~ADC_SR_STRT;

	uint16_t buff[2] = {0, 0};

	DMA2->LIFCR = DMA_LIFCR_CTCIF0;

	buff[0] = s_AdcBuf[0];
	buff[1] = s_AdcBuf[1];

	APP_OnVoltage(&buff[0]);
	APP_OnCurrents(s_AdcBuf[2], s_AdcBuf[3], s_AdcBuf[4]);
}

void ADC_IRQHandler(void);
void ADC_IRQHandler(void)
{
	if (ADC1->SR & ADC_SR_JEOC)
	{
		uint16_t v[ADC_INJBUF_LEN] = {(uint16_t)ADC1->JDR1, (uint16_t)ADC1->JDR2, (uint16_t)ADC1->JDR3, (uint16_t)ADC1->JDR4};

		APP_OnVphase(v);

		ADC1->SR &= ~ADC_SR_JEOC;
		ADC1->SR &= ~ADC_SR_JSTRT;
	}
}

void TIM2_IRQHandler(void);
void TIM2_IRQHandler(void)
{
	TIM2->SR = 0x00000000;

	count_flag = 1;
}

void TIM7_IRQHandler(void);
void TIM7_IRQHandler(void) /* 1〜2ms周期で呼ばれるタイマ割り込み */
{
	TIM7->SR = 0x00000000;

    ENC_Scan(&s_enc, ((uint8_t)((GPIOB->IDR & ((uint16_t)0x300)) >> 8)));
}

void HardFault_Handler(void);
void HardFault_Handler(void)
{
	while(1)
	{

	}
}

void MemManage_Handler(void);
void MemManage_Handler(void)
{
	while(1)
	{

	}
}

void BusFault_Handler(void);
void BusFault_Handler(void)
{
	while(1)
	{

	}
}

void UsageFault_Handler(void);
void UsageFault_Handler(void)
{
	while(1)
	{

	}
}
