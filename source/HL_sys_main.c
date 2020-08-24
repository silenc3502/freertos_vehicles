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

#include <string.h>

#define UART        sciREG1
#define AHRS        sciREG3

#define NEUTRAL         (1500)
#define THRO_BASE       (1350)
#define THRO_INC        (30)
#define REAL_UPPERBASE  (1500)
#define REAL_DOWNBASE   (1420)
#define REAL_INC        (5)

#define D_COUNT         (8)
#define D_SIZE          (8)

#define AUTO_LEFT       (1)
#define AUTO_RIGHT      (2)
#define AUTO_FORWARD    (3)
#define AUTO_BACKWARD   (4)
#define AUTO_STOP       (5)
#define COLLISION       (6)

#define LEFT_LIGHT      (7)
#define RIGHT_LIGHT     (8)
#define FRONT_LIGHT     (9)
#define BREAK_LIGHT     (10)

#define FM_RADIO        (11)

#define MAN_SERVO       (12)
#define MAN_BLDC        (13)

#define OBSTACLE        (15)
#define LIDAR           (16)

#define TRANSITION      (33)
#define ALL_STOP        (44)

uint32 cnt = 0;
uint32 error = 0;
uint32 tx_done = 0;

uint8_t tx_data[D_COUNT] = {1, 2, 3, 4, 4, 3, 2, 1};
uint8_t rx_data[D_COUNT] = {0};

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

#if 0
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
#endif

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

void can_decision_task(void *pvParameters)
{
    for(;;)
    {
        if(xSemaphoreTake(sem, (TickType_t) 0x01) == pdTRUE)
        {
            switch(rx_data[0])
            {
                case AUTO_LEFT:
                    disp_set("Auto Left Turn Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case AUTO_RIGHT:
                    disp_set("Auto Right Turn Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case AUTO_FORWARD:
                    disp_set("Auto Forward Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case AUTO_BACKWARD:
                    disp_set("Auto Backward Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case AUTO_STOP:
                    disp_set("Auto Stop Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case COLLISION:
                    disp_set("Collision Warning Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case LEFT_LIGHT:
                    disp_set("Left Light Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case RIGHT_LIGHT:
                    disp_set("Right Light Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case FRONT_LIGHT:
                    disp_set("Front Light Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case BREAK_LIGHT:
                    disp_set("Break Light Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case FM_RADIO:
                    disp_set("FM Radio Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case MAN_SERVO:
                    disp_set("Manual Servo Control Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case MAN_BLDC:
                    disp_set("Manual BLDC Control Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case OBSTACLE:
                    disp_set("Obstacle Detection Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case LIDAR:
                    disp_set("Lidar Summary Data Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case TRANSITION:
                    disp_set("Change Auto / Manual Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;

                case ALL_STOP:
                    disp_set("All Stop Instruction\n\r\0");
                    xSemaphoreGive(sem);
                    vTaskDelay(250);
                    break;
            }


            xSemaphoreGive(sem);
            //vTaskDelay(30);
            //vTaskDelay(250);
            vTaskDelay(30);
        }
    }
}

void uart_task(void *pvParameters)
{
    //int gear;
    //int real;
    char buf[64] = "";

    for(;;)
    {
        if(xSemaphoreTake(sem, (TickType_t) 0x01) == pdTRUE)
        {
            ecap_throttle();
            ecap_steer();
            ecap_control();
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
            // ecap_throttle();

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
            // ecap_steer();

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

#if 1
    if(xTaskCreate(can_decision_task, "can_decision", configMINIMAL_STACK_SIZE * 2,
                   NULL, 5, &dsp_can_fd) != pdPASS)
    {
        for(;;)
            ;
    }
#endif

#if 0
    if(xTaskCreate(bldc_task, "bldc", configMINIMAL_STACK_SIZE * 2,
                   NULL, 5, &bldc_fd) != pdPASS)
    {
        for(;;)
            ;
    }
#endif

#if 0
    if(xTaskCreate(steering_task, "steering", configMINIMAL_STACK_SIZE,
                   NULL, 5, &steer_fd) != pdPASS)
    {
        for(;;)
            ;
    }
#endif

#if 0
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
void canMessageNotification(canBASE_t *node, uint32_t messageBox)
{
    if(canIsRxMessageArrived(canREG1, canMESSAGE_BOX2))
        canGetData(node, messageBox, rx_data);
}
/* USER CODE END */
