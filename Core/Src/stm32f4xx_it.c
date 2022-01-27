
#include "main.h"
#include "stm32f4xx_it.h"
#include "socket.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;


volatile uint32_t tick = 0;



extern osSemaphoreId_t Sem_tcp_reciv_sem;
extern osThreadId_t ModBusTaskHandle;


volatile long isr_test = 0;

void NMI_Handler(void)
{

  while (1)
  {
  }
}


void HardFault_Handler(void)
{

  while (1)
  {

  }
}


void MemManage_Handler(void)
{

  while (1)
  {

  }
}


void BusFault_Handler(void)
{

  while (1)
  {

  }
}


void UsageFault_Handler(void)
{

  while (1)
  {

  }
}


void DebugMon_Handler(void)
{

}


void TIM1_UP_TIM10_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/* Ќастроено прерывание дл€ всех сокетов
по событию RECIVE */

void EXTI9_5_IRQHandler(void)
{
  EXTI->PR = W5500_INT_Pin;
  if(osOK == osSemaphoreRelease(Sem_tcp_reciv_sem)); //wake up the server
   isr_test++;                  //DEBUG!!!
}



void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
  
}



