#ifndef QEI_H
#define QEI_H

#include "stm32f3xx_hal.h"

typedef enum {
	QEI_X2_ENCODING = 0, QEI_X4_ENCODING
} QEI_Encoding;

typedef struct {
	GPIO_TypeDef *chanA_port;
	uint16_t chanA_pin;
	GPIO_TypeDef *chanB_port;
	uint16_t chanB_pin;
	int pulses;
	int revolutions;
	int pulses_per_rev;
	QEI_Encoding encoding;
	int curr_state;
	int prev_state;
} QEI_HandleTypeDef;

// 初期化
void QEI_Init(QEI_HandleTypeDef *hqei, GPIO_TypeDef *chanA_port,
		uint16_t chanA_pin, GPIO_TypeDef *chanB_port, uint16_t chanB_pin,
		int pulses_per_rev, QEI_Encoding encoding);

// リセット
void QEI_Reset(QEI_HandleTypeDef *hqei);

// パルス数取得
int QEI_GetPulses(QEI_HandleTypeDef *hqei);

// 回転数取得
int QEI_GetRevolutions(QEI_HandleTypeDef *hqei);

// 状態取得
int QEI_GetCurrentState(QEI_HandleTypeDef *hqei);

// 割り込みハンドラ
void QEI_Encode(QEI_HandleTypeDef *hqei);

#endif // QEI_H
