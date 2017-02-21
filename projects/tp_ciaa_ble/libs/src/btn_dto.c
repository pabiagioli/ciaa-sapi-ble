#include "btn_dto.h"

btn_dto botones[] = {
		{ /*.state = BTN_OFF, */ .funcion_btn = funcion_btn1 },
		{ /*.state = BTN_OFF, */ .funcion_btn = funcion_btn2 },
		{ /*.state = BTN_OFF, */ .funcion_btn = funcion_btn3 },
		{ /*.state = BTN_OFF, */ .funcion_btn = funcion_btn4 }
};

uint8_t funcion_btn1 (uint8_t current_duty) {
  return 0x00;
}

uint8_t funcion_btn2 (uint8_t current_duty) {
  return 0xFF;
}

uint8_t funcion_btn3 (uint8_t current_duty) {
  return current_duty + 0x10;
}

uint8_t funcion_btn4 (uint8_t current_duty) {
  return current_duty - 0x10;
}