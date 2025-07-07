#include "qei.h"

// 状態定数
#define QEI_INVALID 0x3
#define QEI_PREV_MASK 0x1
#define QEI_CURR_MASK 0x2

void QEI_Init(QEI_HandleTypeDef *hqei, GPIO_TypeDef *chanA_port,
		uint16_t chanA_pin, GPIO_TypeDef *chanB_port, uint16_t chanB_pin,
		int pulses_per_rev, QEI_Encoding encoding) {
	hqei->chanA_port = chanA_port;
	hqei->chanA_pin = chanA_pin;
	hqei->chanB_port = chanB_port;
	hqei->chanB_pin = chanB_pin;
	hqei->pulses = 0;
	hqei->revolutions = 0;
	hqei->pulses_per_rev = pulses_per_rev;
	hqei->encoding = encoding;

	// 現在の状態を算出
	int chanA = HAL_GPIO_ReadPin(chanA_port, chanA_pin);
	int chanB = HAL_GPIO_ReadPin(chanB_port, chanB_pin);

	hqei->curr_state = (chanA << 1) | chanB;
	hqei->prev_state = hqei->curr_state;
}

void QEI_Reset(QEI_HandleTypeDef *hqei) {
	hqei->pulses = 0;
	hqei->revolutions = 0;
}

int QEI_GetPulses(QEI_HandleTypeDef *hqei) {
	return hqei->pulses;
}

int QEI_GetRevolutions(QEI_HandleTypeDef *hqei) {
	return hqei->revolutions;
}

int QEI_GetCurrentState(QEI_HandleTypeDef *hqei) {
	return hqei->curr_state;
}

// レジスタ直読専用関数
static inline int QEI_ReadChan(GPIO_TypeDef *port, uint16_t pin) {
	return ((port->IDR & pin) ? 1 : 0);
}

void QEI_Encode(QEI_HandleTypeDef *hqei) {
	int change = 0;
	int chanA = QEI_ReadChan(hqei->chanA_port, hqei->chanA_pin);
	int chanB = QEI_ReadChan(hqei->chanB_port, hqei->chanB_pin);

	hqei->curr_state = (chanA << 1) | chanB;

	if (hqei->encoding == QEI_X2_ENCODING) {
		if ((hqei->prev_state == 0x3 && hqei->curr_state == 0x0)
				|| (hqei->prev_state == 0x0 && hqei->curr_state == 0x3)) {
			hqei->pulses++;
		} else if ((hqei->prev_state == 0x2 && hqei->curr_state == 0x1)
				|| (hqei->prev_state == 0x1 && hqei->curr_state == 0x2)) {
			hqei->pulses--;
		}
	} else if (hqei->encoding == QEI_X4_ENCODING) {
		if (((hqei->curr_state ^ hqei->prev_state) != QEI_INVALID)
				&& (hqei->curr_state != hqei->prev_state)) {
			change = (hqei->prev_state & QEI_PREV_MASK)
					^ ((hqei->curr_state & QEI_CURR_MASK) >> 1);
			if (change == 0)
				change = -1;
			hqei->pulses -= change;
		}
	}

	// 1回転判定
	if (hqei->pulses_per_rev > 0) {
		if (hqei->pulses >= hqei->pulses_per_rev) {
			hqei->revolutions++;
			hqei->pulses -= hqei->pulses_per_rev;
		} else if (hqei->pulses <= -hqei->pulses_per_rev) {
			hqei->revolutions--;
			hqei->pulses += hqei->pulses_per_rev;
		}
	}
	hqei->prev_state = hqei->curr_state;
}
