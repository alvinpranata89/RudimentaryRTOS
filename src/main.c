#include <oskernel.h>
#include "uart.h"

#define QUANTA		10

typedef uint32_t TaskProfiler;

TaskProfiler Task0_Profiler,Task1_Profiler,Task2_Profiler;

void motor_run(void);
void motor_stop(void);
void valve_open(void);
void valve_close(void);


void task0(void)
{

	while(1)
	{
		Task0_Profiler++;
		motor_run();
	}
}

void task1(void)
{

	while(1)
	{
		Task1_Profiler++;
		valve_close();
	}
}

void task2(void)
{

	while(1)
	{
		Task2_Profiler++;
		valve_open();
	}
}

int main(void)
{
//	osKernelInit();
//	osKernelAddThreads(&task0, &task1, &task2);
//	osKernelLaunch(QUANTA);
	uart2_tx_init();
	KernelInit();
	AddThreads(&task0, &task1, &task2);
	KernelLaunch(QUANTA);
//	while(1)
//		{
//			Task1_Profiler++;
//			motor_stop();
//		}

}

void motor_run(void)
{
	printf("Motor is starting..\n\r");
}

void motor_stop(void)
{
	printf("Motor is stopping..\n\r");
}

void valve_open(void)
{
	printf("Valve is opening..\n\r");
}

void valve_close(void)
{
	printf("Valve is closing..\n\r");
}


