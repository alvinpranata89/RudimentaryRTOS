#ifndef __OS_KERNEL2__
#define __OS_KERNEL2__
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"

void StackInit(int i);
void KernelInit(void);
uint8_t AddThreads(void(*task0)(void), void(*task1)(void), void(*task2)(void));
void KernelLaunch(uint32_t quanta);
__attribute__((naked)) void SysTick_Handler(void);
void osLaunch(void);
#endif
