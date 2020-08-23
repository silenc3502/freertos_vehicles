/** @file HL_sys_main.c 
*   @brief Application main file
*   @date 07-July-2017
*   @version 04.07.00
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2016 Texas Instruments Incorporated - www.ti.com  
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "HL_sys_common.h"

/* USER CODE BEGIN (1) */
#include "string.h"
#include "stdio.h"

#include "HL_sci.h"
#include "HL_etpwm.h"
#include "HL_ecap.h"
#include "HL_can.h"
#include "HL_system.h"

#include "FreeRTOS.h"
#include "os_task.h"
#include "os_semphr.h"

#define UART        sciREG1
#define AHRS        sciREG3

#define NEUTRAL         (1500)
#define THRO_BASE       (1350)
#define THRO_INC        (30)
#define REAL_UPPERBASE  (1500)
#define REAL_DOWNBASE   (1420)
#define REAL_INC        (5)

SemaphoreHandle_t sem;

TaskHandle_t dsp_can_fd;
TaskHandle_t bldc_fd;
TaskHandle_t steer_fd;
TaskHandle_t uart_fd;
TaskHandle_t ahrs_fd;
TaskHandle_t control_fd;

int throttle, steer, control;
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
void wait(uint32 delay)
{
    int i;

    for(i = 0; i < delay; i++)
        ;
}

void sci_display(sciBASE_t *sci, uint8 *text, uint32 len)
{
    while(len--)
    {
        while((UART->FLR & 0x04) == 4)
            ;
        sciSendByte(UART, *text++);
    }
}

void disp_set(char *str)
{
    sci_display(sciREG1, (uint8 *)str, strlen(str));

    wait(100000);
}

void init_peripheral(void)
{
    sciInit();

    ecapInit();
    ecapStartCounter(ecapREG1);
    ecapStartCounter(ecapREG2);
    ecapStartCounter(ecapREG3);
    ecapStartCounter(ecapREG4);
    ecapStartCounter(ecapREG5);
    ecapStartCounter(ecapREG6);
    ecapEnableCapture(ecapREG1);
    ecapEnableCapture(ecapREG2);
    ecapEnableCapture(ecapREG3);
    ecapEnableCapture(ecapREG4);
    ecapEnableCapture(ecapREG5);
    ecapEnableCapture(ecapREG6);

    etpwmInit();
    etpwmStartTBCLK();

    canInit();
    canEnableErrorNotification(canREG1);

    wait(100000);
}

void ecap_throttle(void)    // channel 3
{
    unsigned int cap[2];

    cap[0] = ecapGetCAP1(ecapREG3);
    cap[1] = ecapGetCAP2(ecapREG3);
    throttle = (cap[1] - cap[0]) / VCLK3_FREQ;
}

void ecap_steer(void)       // channel 2
{
   unsigned int cap[2];

   cap[0] = ecapGetCAP1(ecapREG2);
   cap[1] = ecapGetCAP2(ecapREG2);
   steer = (cap[1] - cap[0]) / VCLK3_FREQ;
}

void ecap_control(void)     // channel 7
{
    unsigned int cap[2];

    cap[0] = ecapGetCAP1(ecapREG6);
    cap[1] = ecapGetCAP2(ecapREG6);
    control = (cap[1] - cap[0]) / VCLK3_FREQ;
}

void uart_task(void *pvPrameters)
{
    //int gear;
    //int real;
    char buf[64] = "";

    for(;;)
    {
        ecap_throttle();
        ecap_steer();
        ecap_control();

        if(xSemaphoreTake(sem, (TickType_t) 0x01) == pdTRUE)
        {
#if 0
            gear = ((throttle - THRO_BASE) / THRO_INC) + 1;

            if(throttle <= 1320 || throttle > 1650)
            {
                real = NEUTRAL;
            }
            else if(gear > 6)
            {
                real = REAL_UPPERBASE + gear * REAL_INC;
            }
            else if(gear + 1 < 6)
            {
                real = REAL_DOWNBASE + (gear + 1) * REAL_INC;
            }
            else if(1530 <= throttle && throttle < 1560)
            {
                real = 1530;
            }
            else if(1470 < throttle && throttle < 1530)
            {
                real = NEUTRAL;
            }
            else if(1440 < throttle && throttle <= 1470)
            {
                real = 1470;
            }
#endif

            sprintf(buf, "bldc motor = %d ms\n\r\0", throttle);
            sci_display(UART, (uint8 *)buf, strlen(buf));

#if 0
            sprintf(buf, "gear = %d ms\n\r\0", gear);
            sci_display(UART, (uint8 *)buf, strlen(buf));

            sprintf(buf, "real = %d ms\n\r\0", real);
            sci_display(UART, (uint8 *)buf, strlen(buf));
#endif

            //sprintf(buf, "steer = %d ms\n\r\0", steer);
            sprintf(buf, "steer = %d ms\n\r\0", steer & ~(32 - 1));
            sci_display(UART, (uint8 *)buf, strlen(buf));

            sprintf(buf, "control = %d ms\n\r\n\r\0", control);
            sci_display(UART, (uint8 *)buf, strlen(buf));

            xSemaphoreGive(sem);
            //vTaskDelay(30);
            //vTaskDelay(250);
            vTaskDelay(510);
        }
    }
}

void bldc_task(void *pvParameters)
{
    int gear = 0;

    for(;;)
    {
        if(xSemaphoreTake(sem, (TickType_t) 0x01) == pdTRUE)
        {
            ecap_throttle();

            /* motor 스펙이 매우 높아 2000 을 모두 먹이면 너무 빠르다.
             * scale 팩터를 적절하게 입혀서 그 사이의 레벨값으로 속도를 조정한다.
             * gear 레벨을 11 개 정도로 설정하도록 한다.
             * gear 레벨은 1 ~ 11 까지고 6 번이 중립 기어다.
             * throttle 의 원본값은 1000 ~ 2000 이므로 0 ~ 1000 을 100 으로 나누면 0 ~ 10 이 된다.
             * gear 1 ~ 5 는 후진이며 7 ~ 11 은 전진에 해당한다.
             * min = 1350, max = 1650 로 gear 값 별로 실제 throttle 은 아래와 같다.
             * 1 = 1430, 2 = 1440, 3 = 1450, 4 = 1460, 5 = 1470
             * 7 = 1530, 8 = 1540, 9 = 1550, 10 = 1560, 11 = 1570
             * pwm 값은 상황에 따라 적절히 유도리 있게 변경하여 사용하도록 한다. */
#if 0
            cur = throttle;

            if(cur < 1460 || cur > 1540)
            {
                etpwmREG1->CMPA = 1500;
            }
            else
                etpwmREG1->CMPA = cur;
#else
            gear = ((throttle - THRO_BASE) / THRO_INC) + 1;

            if(throttle <= 1320 || throttle > 1650)
            {
                etpwmREG1->CMPA = NEUTRAL;
            }
            else if(gear > 6)
            {
                etpwmREG1->CMPA = REAL_UPPERBASE + gear * REAL_INC;
            }
            else if(gear + 1 < 6)
            {
                etpwmREG1->CMPA = REAL_DOWNBASE + (gear + 1) * REAL_INC;
            }
            else if(1530 <= throttle && throttle < 1560)
            {
                etpwmREG1->CMPA = 1530;
            }
            else if(1470 < throttle && throttle < 1530)
            {
                etpwmREG1->CMPA = NEUTRAL;
            }
            else if(1440 < throttle && throttle <= 1470)
            {
                etpwmREG1->CMPA = 1470;
            }
#endif

            xSemaphoreGive(sem);
            vTaskDelay(30);
        }
    }
}

void steering_task(void *pvParameters)
{
    for(;;)
    {
        if(xSemaphoreTake(sem, (TickType_t) 0x01) == pdTRUE)
        {
            ecap_steer();

#if 0
            if(steer >= 900 && steer < 1200)
            {
                etpwmREG2->CMPA = steer;
            }
            else if(steer < 900)
            {
                etpwmREG2->CMPA = 900;
            }
            else if(steer >= 1500 && steer <= 1600)
            {
                etpwmREG2->CMPA = 1500;
            }
            else if(steer > 1300 && steer <= 1800)
            {
                etpwmREG2->CMPA = steer;
            }
            else if(steer > 1800 && steer <= 2000)
            {
                etpwmREG2->CMPA = 1800;
            }
            else
            {
                etpwmREG2->CMPA = 1250;
            }
#endif

            /* servo의 좌측은 중립이 1900 정도여야함
               servo의 우측은 중립이 1100 정도여야함
               1500 to 1900, 1500 to 1100
               range 1000 ~ 2000 - 문제는 1500 인 중립상태가 중립에 있지 않다는것!

               steer = 1500, left = 1900, right = 1100
               steer = 1900(우회전), left = 1900, right = 1900
               steer = 1100(좌회전), left = 1100, right = 1100
               steer = 1700(우회전), left = 1900, right = 1700
               steer = 1300(좌회전), left = 1300, right = 1100

               bit operator를 활용해서 고속으로 32 배수 단위로 정렬한다.
               */
            steer &= ~(32 - 1);
            if(1472 <= steer && steer <= 1536)
            {
                //etpwmREG2->CMPA = 1100;
                etpwmREG2->CMPA = 1900;
                etpwmREG3->CMPA = 1100;
            }
            else if(steer > 1536)
            {
                etpwmREG2->CMPA = 1900;
                etpwmREG3->CMPA = steer;
                //etpwmREG2->CMPA = steer;
                //etpwmREG3->CMPA = 1100;
            }
            else if(steer < 1472)
            {
                etpwmREG2->CMPA = steer;
                etpwmREG3->CMPA = 1100;
                //etpwmREG2->CMPA = 1900;
                //etpwmREG3->CMPA = steer;
            }

            xSemaphoreGive(sem);
            vTaskDelay(30);
        }
    }
}
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    init_peripheral();

    disp_set("Init Peripheral Success!\n\r\0");

    sem = xSemaphoreCreateBinary();
    xSemaphoreGive(sem);

#if 0
    if(xTaskCreate(can_task, "can", configMINIMAL_STACK_SIZE * 2,
                   NULL, 5, &dsp_can_fd) != pdPASS)
    {
        for(;;)
            ;
    }
#endif

#if 1
    if(xTaskCreate(bldc_task, "bldc", configMINIMAL_STACK_SIZE * 2,
                   NULL, 5, &bldc_fd) != pdPASS)
    {
        for(;;)
            ;
    }
#endif

#if 1
    if(xTaskCreate(steering_task, "steering", configMINIMAL_STACK_SIZE,
                   NULL, 5, &steer_fd) != pdPASS)
    {
        for(;;)
            ;
    }
#endif

#if 1
    if(xTaskCreate(uart_task, "uart", configMINIMAL_STACK_SIZE * 2,
                   NULL, 5, &uart_fd) != pdPASS)
    {
        for(;;)
            ;
    }
#endif

#if 0
    if(xTaskCreate(ahrs_task, "ahrs", configMINIMAL_STACK_SIZE * 2,
                   NULL, 5, &ahrs_fd) != pdPASS)
    {
        for(;;)
            ;
    }
#endif


#if 0
    if(xTaskCreate(control_task, "control", configMINIMAL_STACK_SIZE,
                   NULL, 5, &control_fd) != pdPASS)
    {
        for(;;)
            ;
    }
#endif

#if 0
    if(xTaskCreate(light_task, "light", configMINIMAL_STACK_SIZE,
                   NULL, 5, &light_fd) != pdPASS)
    {
        for(;;)
            ;
    }
#endif

    vTaskStartScheduler();
/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
/* USER CODE END */
