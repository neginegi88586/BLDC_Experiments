/*
 * firmware.h
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#ifndef FIRMWARE_H
#define FIRMWARE_H

#include <stdint.h>
#include <stddef.h>


// ===== 周辺初期化 =====
void FW_InitClocksAndGPIO(void);
void FW_InitClock(void);
void FW_InitGPIO(void);
void FW_TIM1_InitPWM(void);
void FW_TIM2_Init(void);
void FW_TIM3_InitBridge(void);
void FW_TIM7_Init(void);
void FW_ADC1_Init(void);
void FW_ADC12_InitDualRegular_TIM3_TRGO(void);
void FW_ADC1_InitInjected_TIM1_CC4(void);
void FW_DMA_InitForADC(void);
void FW_StartAll(void);


// ===== PWM デューティ更新（0..ARR）=====
void FW_SetPWMDuties(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3);


// ===== サンプルタイミング（位相マーカ）=====
void FW_SetSampleMarker(uint16_t ccr4);


// ===== コールバック（アプリ層が実装）=====
void APP_OnCurrents(uint16_t iU, uint16_t iV, uint16_t iW);
void APP_OnVphase(uint16_t *v_adc); // Injectedサンプル
void APP_OnVoltage(uint16_t *v_adc);

#endif
