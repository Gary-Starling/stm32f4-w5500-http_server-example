/* freeRTOS v10.3.1 */
/* CMSIS v2.0 */
/* Discovery kit with STM32F407VG MCU + W5500*/


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "socket.h"

#include <stdio.h>
#include <string.h>                           




#define DEBUG                      1
#define DATA_BUF_SIZE		2048            //input buff size
#define MAX_SOCK                   8        //max sock


#define IP_NOT_SET                 (net_param.ip[0] != net_param_read.ip[0] ||\
net_param.ip[1] != net_param_read.ip[1] ||\
  net_param.ip[2] != net_param_read.ip[2] ||\
    net_param.ip[3] != net_param_read.ip[3])

#define MASK_NOT_SET            (net_param.sn[0] != net_param_read.sn[0] ||\
net_param.sn[1] != net_param_read.sn[1] ||\
  net_param.sn[2] != net_param_read.sn[2] ||\
    net_param.sn[3] != net_param_read.sn[3])

#define GW_NOT_SET              (net_param.gw[0] != net_param_read.gw[0] ||\
net_param.gw[1] != net_param_read.gw[1] ||\
  net_param.gw[2] != net_param_read.gw[2] ||\
    net_param.gw[3] != net_param_read.gw[3])

/************************
* tcp process states list   
*************************/    

/*
enum Tcp_state {
  TCP_IDLE,
  TCP_REQ_INPROC,
  TCP_REQ_DONE,
  TCP_RES_INPROC,
  TCP_RES_DONE,
};
*/


#define TCP_IDLE                        0           /* IDLE, Waiting for data received (TCP established) */
#define TCP_REQ_INPROC  		1           /* Received request from client */
#define TCP_REQ_DONE    		2           /* The end of request parse */
#define TCP_RES_INPROC  		3           /* Sending the response to client (in progress) */
#define TCP_RES_DONE    		4           /* The end of response send */

#define RECV_IR_MSK                   0x04






wiz_PhyConf phy_param = {
  .by = PHY_CONFBY_SW,       // PHY_CONFBY_SW
  .mode = PHY_MODE_AUTONEGO, // PHY_MODE_AUTONEGO
  .speed = PHY_DUPLEX_FULL,  // PHY_SPEED_100
  .duplex = PHY_DUPLEX_FULL  // PHY_DUPLEX_FULL
};

wiz_NetInfo net_param = { 
  .mac = {0x02, 0x04, 0x08, 0x0A, 0x1A, 0x2A},       ///< Source Mac Address
  .ip =  {192, 168, 1, 50},                          ///< Source IP Address
  .sn =  {255, 255, 255, 0},                         ///< Subnet Mask 
  .gw =  {192, 168, 1, 1},                           ///< Gateway IP Address
  .dns = {0, 0, 0, 0},                               ///< DNS server IP Address
  .dhcp = NETINFO_STATIC                             ///< 1 - Static, 2 - DHCP 
};

int8_t link_stat = -1;

wiz_NetInfo net_param_read;

uint8_t input_buff[DATA_BUF_SIZE] = {0};
char out_buff[DATA_BUF_SIZE] = {0};

/* test page*/
char str_len[20] = {0}; 
char str_cnt[20] = {0}; 
char head[] = "HTTP/1.1 200 OK\r\n"
"Content-Type: text/html; charset=windows-1251\r\n"
"Content-Length:";

char page_start[] = "<!DOCTYPE html><html>"
"<body>"
"<br><br><br><center><b>HTTP W5500 + STM32 HTTP</b></center><br>";

char page_value[] =
"Response counter = ";

char page_end[] =
"</body>"
"</html>";
/* test page */

uint8_t sock_status[8] = { TCP_IDLE };   
int32_t recive_len = 0;            
uint32_t size = 0;
uint32_t sock_err_cnt = 0;


void w5500_sel(void);
void w5500_unsel(void);
void w5500_read_buff(uint8_t* buff, uint16_t len);
void w5500_write_buff(uint8_t* buff, uint16_t len);
uint8_t w5500_read_byte(void);
void w5500_write_byte(uint8_t byte);
void w5500_init(void);
void w5500_enter_critical(void);
void w5500_exit_critical(void);
uint8_t tcp_ip_ser_run(uint8_t sock_number);
void w5500_full_reset(void);



void SystemClock_Config(void);

/* externs */
extern volatile long isr_test; 
/* externs */


#if ( DEBUG == 1)
/* debug values */
static size_t free_size = 0;
volatile unsigned long ulIdleCycleCount = 0;
/* debug values */
#endif

typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;


/* Definitions for tcpCountingSem */
osSemaphoreId_t Sem_tcp_reciv_sem;
osStaticSemaphoreDef_t myCountingSem01ControlBlock;
const osSemaphoreAttr_t myCountingSem01_attributes = {
  .name = "tcpCountingSem",
  .cb_mem = &myCountingSem01ControlBlock,
  .cb_size = sizeof(myCountingSem01ControlBlock),
};



/* Definitions for ModBusTask */
osThreadId_t ModBusTaskHandle;
uint32_t ModBusTaskBuffer[ 1024 ];
osStaticThreadDef_t ModBusTaskControlBlock;
const osThreadAttr_t ModBusTask_attributes = {
  .name = "ModBusTask",
  .cb_mem = &ModBusTaskControlBlock,
  .cb_size = sizeof(ModBusTaskControlBlock),
  .stack_mem = &ModBusTaskBuffer[0],
  .stack_size = sizeof(ModBusTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};

/* Definitions for Lcd_Bottons_tas */
osThreadId_t Lcd_Bottons_tasHandle;
uint32_t Lcd_Bottons_tasBuffer[ 256 ];
osStaticThreadDef_t Lcd_Bottons_tasControlBlock;
const osThreadAttr_t Lcd_Bottons_tas_attributes = {
  .name = "Lcd_Bottons_tas",
  .cb_mem = &Lcd_Bottons_tasControlBlock,
  .cb_size = sizeof(Lcd_Bottons_tasControlBlock),
  .stack_mem = &Lcd_Bottons_tasBuffer[0],
  .stack_size = sizeof(Lcd_Bottons_tasBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Logic_task */
osThreadId_t Logic_taskHandle;
uint32_t Logic_taskBuffer[ 256 ];
osStaticThreadDef_t Logic_taskControlBlock;
const osThreadAttr_t Logic_task_attributes = {
  .name = "Logic_task",
  .cb_mem = &Logic_taskControlBlock,
  .cb_size = sizeof(Logic_taskControlBlock),
  .stack_mem = &Logic_taskBuffer[0],
  .stack_size = sizeof(Logic_taskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Calcul_task */
osThreadId_t Calcul_taskHandle;
uint32_t Calcul_taskBuffer[ 256 ];
osStaticThreadDef_t Calcul_taskControlBlock;
const osThreadAttr_t Calcul_task_attributes = {
  .name = "Calcul_task",
  .cb_mem = &Calcul_taskControlBlock,
  .cb_size = sizeof(Calcul_taskControlBlock),
  .stack_mem = &Calcul_taskBuffer[0],
  .stack_size = sizeof(Calcul_taskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};



void modbus_task(void *argument);
void lcd_bottons_task(void *argument);
void logic_task(void *argument);
void calcul_task(void *argument);

void MX_FREERTOS_Init(void); 


/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook (void);
void vApplicationIdleHook(void);



void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  while(1);
}


void vApplicationMallocFailedHook (void)
{
  while(1);
}


void vApplicationIdleHook( void )
{
  /* Увеличить переменную-счетчик на 1 */
  ulIdleCycleCount++;
}

void MX_FREERTOS_Init(void) 
{
  
  /* add mutexes, ... */
  /* add semaphores, ... */
  Sem_tcp_reciv_sem = osSemaphoreNew(10000, 0, &myCountingSem01_attributes);
  /* start timers, add new ones, ... */
  /* add queues, ... */
  
  /* Create the thread(s) */
  /* creation of ModBusTask */
  ModBusTaskHandle = osThreadNew(modbus_task, NULL, &ModBusTask_attributes);
  
  /* creation of Lcd_Bottons_tas */
  Lcd_Bottons_tasHandle = osThreadNew(lcd_bottons_task, NULL, &Lcd_Bottons_tas_attributes);
  
  /* creation of Logic_task */
  Logic_taskHandle = osThreadNew(logic_task, NULL, &Logic_task_attributes);
  
  /* creation of Calcul_task */
  Calcul_taskHandle = osThreadNew(calcul_task, NULL, &Calcul_task_attributes);
  
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
  
  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... osEventFlagsNew*/
  /* USER CODE END RTOS_EVENTS */
  
}


void modbus_task(void *argument)
{
  int res = 0;
  static uint8_t sock_st[MAX_SOCK] = { 0 };
  
  MX_SPI1_Init();
  /* init w5500*/
  w5500_init();   
  wizphy_reset();
  wizphy_setphyconf(&phy_param);
  wizphy_getphystat(&phy_param);
  wizchip_getnetinfo(&net_param_read);
  link_stat = wizphy_getphylink();   

  if(IP_NOT_SET || MASK_NOT_SET || GW_NOT_SET)
  {
    while(IP_NOT_SET || MASK_NOT_SET || GW_NOT_SET) 
      w5500_full_reset(); 
  }
  
  for(;;)
  {
    
    
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin); //DEBUG
    link_stat = wizphy_getphylink();
    wizchip_getnetinfo(&net_param_read);
    
    //if 
    if(IP_NOT_SET || MASK_NOT_SET || GW_NOT_SET)
    {
      while(IP_NOT_SET || MASK_NOT_SET || GW_NOT_SET)
      {
        w5500_full_reset(); 
        osDelay(100);
      }
    }
    
    if(link_stat) //ethernet is attached
    {
      for(uint8_t sock = 0; sock < 8; sock++)
      {
        sock_st[sock] = tcp_ip_ser_run(sock);
        if(sock_st[sock] == SOCK_LISTEN)
          res++;
      }
      /* all sock is listening  */
      if( res == MAX_SOCK ) 
      { 
        res = 0; //
        if(0 == osSemaphoreGetCount(Sem_tcp_reciv_sem)) 
        {
          for(uint8_t sock = 0; sock < 8; sock++)
          {
            if(getSn_IR(sock) & Sn_IR_RECV) 
              res++; 
          }
          if(!res) //if we don't have recv 
           osSemaphoreAcquire(Sem_tcp_reciv_sem, portMAX_DELAY);//wait untill exti will not occur
          res = 0;
        }
        else  //some interrupts did't handle
        {
          while (0 != osSemaphoreGetCount(Sem_tcp_reciv_sem))  //clear all
            osSemaphoreAcquire(Sem_tcp_reciv_sem, 0);   
        }
      }
      else
      {
        res = 0;
        osThreadYield();
      }
    }
    else //Ethernet is not attach
      osThreadYield();
    
  }
}


void lcd_bottons_task(void *argument)
{
  for(;;)
  {
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD3_Pin);
    osDelay(500);
  }
}


void logic_task(void *argument)
{
  
  
  for(;;)
  {
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD5_Pin);
    osDelay(100);
  }
  
}


void calcul_task(void *argument)
{
  for(;;)
  {
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD6_Pin);
    osDelay(1000);
  }
}



void w5500_sel(void) 
{
  HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);
}

void w5500_unsel(void) 
{
  HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_SET);
}

void w5500_read_buff(uint8_t* buff, uint16_t len) 
{
  HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
}

void w5500_write_buff(uint8_t* buff, uint16_t len) 
{
  HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
}

uint8_t w5500_read_byte(void) 
{
  uint8_t byte;
  
  w5500_read_buff(&byte, sizeof(byte));
  return byte;
}

void w5500_write_byte(uint8_t byte)
{
  w5500_write_buff(&byte, sizeof(byte));
}


void w5500_enter_critical(void)
{
  taskENTER_CRITICAL(); 
}

void w5500_exit_critical(void)
{
  taskEXIT_CRITICAL();	
}

void w5500_init(void)
{
  /* w5500 input output buff size 2кБит */
  uint8_t rx_tx_buff_size[8] = {2, 2, 2, 2, 2, 2, 2, 2};
  
  /* deinit - init w5500 */
  HAL_GPIO_WritePin(W5500_RESET_GPIO_Port, W5500_RESET_Pin, GPIO_PIN_RESET);
  osDelay(1);         // >= 500uSec
  HAL_GPIO_WritePin(W5500_RESET_GPIO_Port, W5500_RESET_Pin, GPIO_PIN_SET);
  osDelay(15);        // >= 10ms
  
  /* reg callbacks wizchip_conf.c/.h */ 
  reg_wizchip_cris_cbfunc(w5500_enter_critical, w5500_exit_critical);   //Для критических секций
  reg_wizchip_cs_cbfunc(w5500_sel, w5500_unsel);                        //CS для w5500
  reg_wizchip_spi_cbfunc(w5500_read_byte, w5500_write_byte);            //чтение/запись одного байта
  reg_wizchip_spiburst_cbfunc(w5500_read_buff, w5500_write_buff);       //чтение/запись буфера
  wizchip_init(rx_tx_buff_size, rx_tx_buff_size);                       //
  wizchip_setnetinfo(&net_param);
  ctlnetwork(CN_SET_NETINFO, (void*) &net_param);                       //Установка параметров сети
  
  /*init inerrupts */
  for(uint8_t sock_n = 0;  sock_n < 8; sock_n++)
    setSn_IMR(sock_n, RECV_IR_MSK);
  /* ebavle interrups*/
  setSIMR(0xff);  
}



uint8_t tcp_ip_ser_run(uint8_t sock_number)
{
  
  uint8_t time_out_cnt = 0;
  static uint32_t resp_cnt = 0;         //Cnt output
  uint8_t sock_sn[MAX_SOCK] = { 0 };
  
  sock_sn[sock_number] = getSn_SR(sock_number);
  
  switch(sock_sn[sock_number])
  {
    //==============================================================================    
  case SOCK_ESTABLISHED:
    
    // Interrupt clear
    if(getSn_IR(sock_number) & Sn_IR_RECV) //sock n interrupt?
    {
      setSn_IR(sock_number, Sn_IR_RECV);        // clear Sn_IR_RECV
      osSemaphoreAcquire(Sem_tcp_reciv_sem, 0); //clear cnt sem
    }
    
    if(getSn_IR(sock_number) & Sn_IR_CON)
      setSn_IR(sock_number, Sn_IR_CON);
    
    
    switch(sock_status[sock_number])  // HTTP Process states
    {
      
    case TCP_IDLE :
      if ((recive_len = getSn_RX_RSR(sock_number)) > 0)
      {
        if (recive_len > DATA_BUF_SIZE) recive_len = DATA_BUF_SIZE;
        recive_len = recv(sock_number, input_buff, recive_len);   
        
        *(input_buff + recive_len) = '\0';
        
        /**/
        while(getSn_TX_FSR(sock_number) != (getSn_TxMAX(sock_number)))
        {
          time_out_cnt++;
          osDelay(10);
          if(time_out_cnt >= 3)
          {
            time_out_cnt = 0;
            break;
          }
        }
        time_out_cnt = 0;
        sock_status[sock_number] = TCP_RES_INPROC;     
      }
      break;
      
    case TCP_REQ_INPROC:
      break;
      
      /* Len page is very important!!!*/
    case TCP_RES_INPROC :
      memset(out_buff, '\0', sizeof(out_buff));
      resp_cnt++;
      sprintf(str_cnt, "%d", resp_cnt);      
      strcpy(out_buff, head);            //Вставим хедер
      /* copy len into */  
      sprintf(str_len, "%d", strlen(page_start) + strlen(page_value) + \
        strlen(page_end) + strlen(str_cnt));              
      strcat(out_buff, str_len);             
      strcat(out_buff, "\r\n\r\n");          
      strcat(out_buff, page_start);         
      strcat(out_buff, page_value);
      strcat(out_buff, str_cnt);
      strcat(out_buff, page_end);            //
      size = strlen(out_buff);               //
      send(sock_number, (uint8_t *)(out_buff), size);
      sock_status[sock_number] = TCP_RES_DONE;
      break;
      
    case TCP_RES_DONE :
      memset(input_buff, '\0', sizeof(input_buff)); //clear buffers
      memset(out_buff, '\0', sizeof(out_buff));
      size = 0;
      sock_err_cnt = 0;
      setSn_CR(sock_number,Sn_CR_DISCON);
      while(getSn_CR(sock_number));                      //wait utill sock is closing
      sock_status[sock_number] = TCP_IDLE;
      break;
      
    default :
      break;
    }
    break;
    //==========================================================================    
  case SOCK_TIME_WAIT:
    break;
    //==========================================================================    
  case SOCK_CLOSE_WAIT:
    disconnect(sock_number);
    break;
    //==========================================================================
  case SOCK_CLOSED:
    if(socket(sock_number, Sn_MR_TCP, 80, 0x00) == SOCKERR_SOCKINIT)
    {
      sock_err_cnt++;
      osDelay(10);
      if(sock_err_cnt >= 30)                            //hard error
      {
        sock_err_cnt = 0; 
        /* clear all semaphore otherwise we wont't stop after */
        while (0 != osSemaphoreGetCount(Sem_tcp_reciv_sem)) 
          osSemaphoreAcquire(Sem_tcp_reciv_sem, 0);        
        MX_SPI1_Init();                 

      }
    }
    break;
    //===========================================================================  
  case SOCK_INIT:
    if(listen(sock_number)!= SOCK_OK)
    {
      disconnect(sock_number);
      close(sock_number);
      sock_status[sock_number] = SOCK_CLOSED;
    }
    break;
    //==============================================================================    
  case SOCK_LISTEN:
    return sock_sn[sock_number];
    break;
    //==============================================================================
  default:
    break;
    
  } // end of switch
  
  return sock_sn[sock_number];
}

void w5500_full_reset(void)
{
  MX_SPI1_Init();
  w5500_init();
  wizphy_reset();
  wizphy_setphyconf(&phy_param);
  wizphy_getphystat(&phy_param);
  wizchip_getnetinfo(&net_param_read);
  link_stat = wizphy_getphylink();   
}

