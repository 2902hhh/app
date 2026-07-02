/*
 * Copyright (c) 2020 HiSilicon (Shanghai) Technologies CO., LIMITED.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <unistd.h>

#include "ohos_init.h"
#include "cmsis_os2.h"

#include <hi_types_base.h>
#include <hi_early_debug.h>

#include <hi_pwm.h>


#include <hi_io.h>

int flag = 0;
int dir = 1;

#define LED_PORT 9

hi_void pwm_0_demo(hi_void)
{

    hi_pwm_start(HI_PWM_PORT_PWM0, flag, 1500); /* duty: 750 freq:1500 */
}


hi_void app_demo_pwm(hi_void)
{
    printf("start test pwm");

    pwm_0_demo();


    printf("please use an oscilloscope to check the output waveform!");
}

void *PWM_Task(const char*arg)
{
    arg = arg;


    hi_io_set_func(LED_PORT,HI_IO_FUNC_GPIO_9_PWM0_OUT);
    hi_pwm_init(HI_PWM_PORT_PWM0);
    hi_pwm_set_clock(PWM_CLK_160M);
    while(1)
    {   
        if(flag>=1500)
        {
            dir = -1;
        }
        if(flag<=0)
        {
            dir = 1;
        }
        flag = flag + dir*5;
        app_demo_pwm();
        //usleep(5000000);

    }
}

void pwm_demo(void)
{
    osThreadAttr_t attr;

    attr.name = "PWM_Task";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 2048;
    attr.priority = 26;

    if (osThreadNew((osThreadFunc_t)PWM_Task, NULL, &attr) == NULL) {
        printf("[PWM_Task] Falied to create PWM_Task!\n");
    }

}

SYS_RUN(pwm_demo);