
/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include "ets_sys.h"
#ifndef __hw_timer_H__
#define __hw_timer_H__
//TIMER PREDIVED MODE
typedef enum {
    DIVDED_BY_1 = 0,        //timer clock
    DIVDED_BY_16 = 4,    //divided by 16
    DIVDED_BY_256 = 8,    //divided by 256
} time_predived_mode;

typedef enum {            //timer interrupt mode
    TM_LEVEL_INT = 1,    // level interrupt
    TM_EDGE_INT   = 0,    //edge interrupt
} time_int_mode;

typedef enum {
    FRC1_SOURCE = 0,
    NMI_SOURCE = 1,
} frc1_timer_source_type;


void  hw_timer_init(frc1_timer_source_type source_type, uint8_t req);

void  hw_timer_set_func(void (* user_hw_timer_cb_set)(void)) ;
void  hw_timer_arm(uint32_t val) ;

#endif
