/*
 * app.h
 *
 *  Created on: Oct 13, 2025
 *      Author: idune
 */

#ifndef APP_H
#define APP_H

#include "fixed_q16.h"

void sincos_q16(q16_t th, q16_t *s, q16_t *c);
void APP_Init(void);
void APP_Step(void);

#endif
