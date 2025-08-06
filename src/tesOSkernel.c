/*
 * tesOSkernel.c
 *
 *  Created on: Aug 4, 2025
 *      Author: Alvin Pranata
 */
#include <oskernel.h>
#define NUM_OF_THREADS			3
#define STACKSIZE				500
#define BUS_FREQ				16000000

#define CTRL_ENABLE				(1U<<0)
#define CTRL_TICKINT 			(1U<<1)
#define CTRL_CLKSRC 			(1U<<2)
#define CTRL_COUNTFLAG			(1U<<16)

typedef struct tcb{
	int32_t *stackPt;
	struct tcb *nextPt;
}tcbType;

tcbType tcbs[NUM_OF_THREADS];
tcbType *currentPt;
uint32_t MILLIS_PRESCALER;

int32_t TCBSTACK[NUM_OF_THREADS][STACKSIZE];

void osLaunch(void);

void StackInit(int i) // this function is called to initialize the stack allocation on memory as a temporary "snapshot" of the CPU's register
{
	tcbs[i].stackPt=&TCBSTACK[i][STACKSIZE-16]; //setting the TCB's stack pointer to the copied SP register on memory from CPU

	TCBSTACK[i][STACKSIZE-1] |= (1U<<24);       //setting the thumb mode operation at bit 24 of copied CPU's PSR register (bit 1)
	TCBSTACK[i][STACKSIZE-3] = 0xAAAAAAAA;		//---------------------------------------
	TCBSTACK[i][STACKSIZE-4] = 0xAAAAAAAA;		//
	TCBSTACK[i][STACKSIZE-5] = 0xAAAAAAAA;		//
	TCBSTACK[i][STACKSIZE-6] = 0xAAAAAAAA;		//
	TCBSTACK[i][STACKSIZE-7] = 0xAAAAAAAA;		//
	TCBSTACK[i][STACKSIZE-8] = 0xAAAAAAAA;		//  Mimicking the CPU's register order on memory, values = dummy only
	TCBSTACK[i][STACKSIZE-9] = 0xAAAAAAAA;		//
	TCBSTACK[i][STACKSIZE-10] = 0xAAAAAAAA;		//
	TCBSTACK[i][STACKSIZE-11] = 0xAAAAAAAA;		//
	TCBSTACK[i][STACKSIZE-12] = 0xAAAAAAAA;		//
	TCBSTACK[i][STACKSIZE-13] = 0xAAAAAAAA;		//
	TCBSTACK[i][STACKSIZE-14] = 0xAAAAAAAA;		//
	TCBSTACK[i][STACKSIZE-15] = 0xAAAAAAAA;		//
	TCBSTACK[i][STACKSIZE-16] = 0xAAAAAAAA;		//---------------------------------------
}

void KernelInit(void){
	MILLIS_PRESCALER = (BUS_FREQ/1000);
}

uint8_t AddThreads(void(*task0)(void), void(*task1)(void), void(*task2)(void))
{
//	disable global interrupt
	__disable_irq();
	tcbs[0].nextPt=&tcbs[1]; //--------------------------------
	tcbs[1].nextPt=&tcbs[2]; // creating a circular thread flow
	tcbs[2].nextPt=&tcbs[0]; //--------------------------------

	StackInit(0); // initializing each thread0's stack allocation on memory
	TCBSTACK[0][STACKSIZE-2] = (int32_t)task0; // pointing the PC to the address of task0

	StackInit(1); // initializing each thread1's stack allocation on memory
	TCBSTACK[1][STACKSIZE-2] = (int32_t)task1; // pointing the PC to the address of task1

	StackInit(2);// initializing each thread2's stack allocation on memory
	TCBSTACK[2][STACKSIZE-2] = (int32_t)task2; // pointing the PC to the address of task2

	currentPt=&tcbs[0];

	//enable global interrupt
	__enable_irq();
	return 1;

}

void KernelLaunch(uint32_t quanta) //using systick as heartbeat
{
	SysTick->CTRL =0;
	SysTick->LOAD = (quanta * MILLIS_PRESCALER);
	SysTick->VAL =0;
	NVIC_SetPriority(SysTick_IRQn,15);					//setting the Systick to the lowest priority so it is always prioritized
	SysTick->CTRL = CTRL_CLKSRC | CTRL_ENABLE | CTRL_TICKINT; 	//setting the systick register to use interrupt and processor clock, disabling the counter
	osLaunch(); 												//kickstart the first thread execution
}

__attribute__((naked)) void SysTick_Handler(void)
{//if systick time runs out, this function will be called, executing the necessary steps to switch thread

	__asm("cpsid i"); //disable global interrupt
	__asm("push {r4-r11}");//push r4-r11 manually to memory

	__asm("ldr r0,=currentPt");//-------------------------------------------
	__asm("ldr r1,[r0]");      //  currentPt = stack pointer value from CPU
	__asm("str sp,[r1]");      //-------------------------------------------

	__asm("ldr r1,[r1,#4]");//-------------------------------------------------------------------------
	__asm("str r1,[r0]");   //   switching to next TCB,
	__asm("ldr sp,[r1]");   //   loading CPU's stack pointer with the next TCB's address from memory
					        //-------------------------------------------------------------------------
	__asm("pop {r4-r11}");  // popping back the previously pushed r4-r11
	__asm("cpsie i");       // enable back global interrupt
	__asm("bx lr");         // branch to the next flow of code in main.c
}

void osLaunch(void)
{//this function is the very first executor of the thread
		__asm("ldr r0,=currentPt"); //---------------------------------------------------------
		__asm("ldr r2,[r0]");       // loading CPU's stack pointer with currentPt from memory
		__asm("ldr sp,[r2]");       //---------------------------------------------------------

		__asm("pop {r4-r12}");		//-------------------------------------------------------------------
		__asm("pop {r0-r3}");		// popping the necessary registers as if previously context switched

		__asm("add sp,sp,#4");      //skipping the pushed LR register because it is always using the updated one from the CPU
		__asm("pop {lr}");			//using the real LR value from CPU
		__asm("add sp,sp,#4");		//skipping PSR register

		__asm("cpsie i");			//enable global interrupt
		__asm("bx lr");				// branch to the next flow of code in main.c
}

