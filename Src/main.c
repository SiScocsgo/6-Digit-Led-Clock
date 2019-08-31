/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gps.h" // gps
#include "math.h" // floor
#include "stdint.h" //ints
#include "stdio.h"
#include "stdlib.h"
#include "string.h" // strings
#include "stdbool.h" // bool
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_FULL_LL_DRIVER
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
#define dDelay 100

const uint8_t numTable[] = // 7 segment display data
{
0b00111111, // 0
0b00000110, // 1
0b01011011, // 2
0b01001111, // 3
0b01100110, // 4
0b01101101, // 5
0b01111101, // 6
0b00000111, // 7
0b01111111, // 8
0b01101111  // 9
  };
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void SystemClock_Config(void);
static void LL_Init(void);
static void TIM2_Init(void);
static void TIM4_Init(void);
static void MX_GPIO_Init(void);

void usart_init(void);
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);
void usart_send_string(const char* str);
void usart_send_line(const char* str);
void delay(uint32_t ticks);
void delayU(uint32_t ticks);
uint32_t millis();
uint32_t micros();
void dMulti();
void refreshDisplay(void);

extern void gpsUpdate();
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t dTick = 0, myTick = 0, sysTick = 0;
static uint32_t bTime, dTime;
uint32_t time;
static uint16_t timeZone;
static uint8_t usart_rx_dma_buffer[64];
static uint8_t d[7];
static uint8_t h, m, s, bPress;
uint8_t colEn;
uint8_t gpschar;
/* USER CODE END 0 */
uint16_t temp;
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  MX_GPIO_Init();
  /* USER CODE BEGIN Init */
  TIM2_Init();
  TIM4_Init();
  LL_Init();
  /* USER CODE END Init */
  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  usart_init();
  usart_send_string("USART Initialized\r\n");
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  timeZone = (uint16_t)(*(uint16_t*)0x0800D000);
  colEn = (uint16_t)(*(uint16_t*)0x0800C000);
  while (1) {
    if(millis() - dTime > 50)  {
    refreshDisplay();
    char str[20];
    char str4[12];
    sprintf(str, "%d:%d:%d", h,m,s);
    sprintf(str4, "%ld", time);
    usart_send_string(str);
    usart_send_string("   ");
    usart_send_string(str4);
    usart_send_line("");
    dTime = millis();
    }

    if(HAL_GPIO_ReadPin(GPIOB, But_Pin) == 0)  {
      if(bPress == 0)  {
        bTime = millis();
        bPress = 1;
      }
      if(bPress == 1 && millis() - bTime > 50)  {
        bPress = 2;
      }      
    }else if(HAL_GPIO_ReadPin(GPIOB, But_Pin) == 1 && bPress == 2)  {
      if(millis() - bTime > 1000)  {
        colEn = (uint16_t)(*(uint16_t*)0x0800C000);
        colEn = ~colEn;
        HAL_FLASH_Unlock();
        FLASH_PageErase(0x0800C000);
        CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x0800C000, colEn);
        CLEAR_BIT (FLASH->CR, (FLASH_CR_PG));
        HAL_FLASH_Lock();
      } else {
        timeZone = (uint16_t)(*(uint16_t*)0x0800D000);
        timeZone++;
        if(timeZone >= 24)  {
          timeZone = 0;
        }
        HAL_FLASH_Unlock();
        FLASH_PageErase(0x0800D000);
        CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x0800D000, timeZone);
        CLEAR_BIT (FLASH->CR, (FLASH_CR_PG));
        HAL_FLASH_Lock();
      }
      bPress = 0;
    }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
 
static void MX_GPIO_Init(void)  {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // GPIO Ports Clock Enable
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
  // __HAL_RCC_GPIOD_CLK_ENABLE();
  // __HAL_RCC_GPIOA_CLK_ENABLE();
  // __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, A_Pin|B_Pin|C_Pin|D_Pin|E_Pin|F_Pin|G_Pin|Col_Pin, GPIO_PIN_RESET); // Configure GPIO pin Output Level
  GPIO_InitStruct.Pin = A_Pin|B_Pin|C_Pin|D_Pin|E_Pin|F_Pin|G_Pin|Col_Pin; // Configure GPIO pins : A_Pin B_Pin C_Pin D_Pin E_Pin F_Pin G_Pin Col_Pin
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, Char_1_Pin|Char_2_Pin|Char_3_Pin|Char_4_Pin|Char_5_Pin|Char_6_Pin, GPIO_PIN_RESET); //Configure GPIO pin Output Level
  GPIO_InitStruct.Pin = Char_1_Pin|Char_2_Pin|Char_3_Pin|Char_4_Pin|Char_5_Pin|Char_6_Pin; // Configure GPIO pins : Char_1_Pin Char_2_Pin Char_3_Pin Char_4_Pin Char_5_Pin Char_6_Pin
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = But_Pin; // Configure Button Pin
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/* USER CODE BEGIN 4 */

void  usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5); // Calculate current position in buffer
    if (pos != old_pos) { // Check change in received data
        if (pos > old_pos) { // Current position is over previous one. "linear" mode. Process data directly by subtracting "pointers"
          for(int i = 0; i < pos-old_pos; i++) {
            gpschar = usart_rx_dma_buffer[old_pos + i];
            gpsUpdate();
          } 
          // usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else { // We are in "overflow" mode. We are in "overflow" mode. First process data to the end of buffer
          for(int i = 0; i < ARRAY_LEN(usart_rx_dma_buffer) - old_pos; i++) {
            gpschar = usart_rx_dma_buffer[old_pos + i];
            gpsUpdate();
          } 
          // usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
          if (pos) { // Check and continue with beginning of buffer
            for(int i = 0; i < pos; i++) {
              gpschar = usart_rx_dma_buffer[0];
              gpsUpdate();
            } 
            // usart_process_data(&usart_rx_dma_buffer[0], pos);
          }
        }
    }
    old_pos = pos; // Save current position as old
    
    if (old_pos == ARRAY_LEN(usart_rx_dma_buffer)) { // Check and manually update if we reached end of buffer
        old_pos = 0;
    }
}

void usart_process_data(const void* data, size_t len)  {
    const uint8_t* d = data;
    while (len--) {
        LL_USART_TransmitData8(USART1, *d++);
        while (!LL_USART_IsActiveFlag_TXE(USART1));
    }
    while (!LL_USART_IsActiveFlag_TC(USART1));
}

void usart_send_string(const char* str) {
    usart_process_data(str, strlen(str));
}

void usart_send_line(const char* str) {
    const char endl[2] = "\r\n";
    // const char endl[2] = "\n";
    usart_process_data(str, strlen(str));
    usart_process_data(endl, strlen(endl));
}

void delay(uint32_t ticks)  {
  TIM4->CR1 |= TIM_CR1_CEN;
  myTick = 0;
  while(myTick < (ticks * 1000))  {
  }
  TIM4->CR1 &= ~TIM_CR1_CEN;
}

void delayU(uint32_t ticks)  {
  TIM4->CR1 |= TIM_CR1_CEN;
  myTick = 0;
  while(myTick < ticks) {
  }
  TIM4->CR1 &= ~TIM_CR1_CEN;
}

uint32_t millis() {
  return sysTick / 1000;
}

uint32_t micros() {
  return sysTick;
}

void refreshDisplay(void) {
  h = ((time / 1000000) + timeZone) % 24;
  m = (time / 10000) % 100;
  s = (time / 100) % 100;
  d[0] = h / 10;
  d[1] = h % 10;
  d[2] = m / 10;
  d[3] = m % 10;
  d[4] = s / 10;
  d[5] = s % 10;
  d[6] = (d[5] % 2)<<7 & colEn;
}

void dMulti()  {
  static uint8_t dChar;
  switch(dChar)  {
    case 0:
    HAL_GPIO_WritePin(GPIOA, ~numTable[d[dChar]] | ~d[6], 0);
    HAL_GPIO_WritePin(GPIOB, Char_2_Pin|Char_3_Pin|Char_4_Pin|Char_5_Pin|Char_6_Pin, 0);
    HAL_GPIO_WritePin(GPIOA, numTable[d[dChar]] | d[6], 1);
    HAL_GPIO_WritePin(GPIOB, Char_1_Pin, 1);
    break;
    case 1:
    HAL_GPIO_WritePin(GPIOA, ~numTable[d[dChar]] | ~d[6], 0);
    HAL_GPIO_WritePin(GPIOB, Char_1_Pin|Char_3_Pin|Char_4_Pin|Char_5_Pin|Char_6_Pin, 0);
    HAL_GPIO_WritePin(GPIOA, numTable[d[dChar]] | d[6], 1);
    HAL_GPIO_WritePin(GPIOB, Char_2_Pin, 1);
    break;
    case 2:
    HAL_GPIO_WritePin(GPIOA, ~numTable[d[dChar]] | ~d[6], 0);
    HAL_GPIO_WritePin(GPIOB, Char_1_Pin|Char_2_Pin|Char_4_Pin|Char_5_Pin|Char_6_Pin, 0);
    HAL_GPIO_WritePin(GPIOA, numTable[d[dChar]] | d[6], 1);
    HAL_GPIO_WritePin(GPIOB, Char_3_Pin, 1);
    break;
    case 3:
    HAL_GPIO_WritePin(GPIOA, ~numTable[d[dChar]] | ~d[6], 0);
    HAL_GPIO_WritePin(GPIOB, Char_1_Pin|Char_2_Pin|Char_3_Pin|Char_5_Pin|Char_6_Pin, 0);
    HAL_GPIO_WritePin(GPIOA, numTable[d[dChar]] | d[6], 1);
    HAL_GPIO_WritePin(GPIOB, Char_4_Pin, 1);
    break;
    case 4:
    HAL_GPIO_WritePin(GPIOA, ~numTable[d[dChar]] | ~d[6], 0);
    HAL_GPIO_WritePin(GPIOB, Char_1_Pin|Char_2_Pin|Char_3_Pin|Char_4_Pin|Char_6_Pin, 0);
    HAL_GPIO_WritePin(GPIOA, numTable[d[dChar]] | d[6], 1);
    HAL_GPIO_WritePin(GPIOB, Char_5_Pin, 1);
    break;
    case 5:
    HAL_GPIO_WritePin(GPIOA, ~numTable[d[dChar]] | ~d[6], 0);
    HAL_GPIO_WritePin(GPIOB, Char_1_Pin|Char_2_Pin|Char_3_Pin|Char_4_Pin|Char_5_Pin, 0);
    HAL_GPIO_WritePin(GPIOA, numTable[d[dChar]] | d[6], 1);
    HAL_GPIO_WritePin(GPIOB, Char_6_Pin, 1);
    break;
  }
  if(dChar > 5) {
    dChar = 0;
  } else  {
    dChar++;
  }
}

void usart_init(void) {
    LL_USART_InitTypeDef USART_InitStruct;
    // UART_InitTypeDef USART_InitStruct;
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    // GPIO_InitTypeDef GPIO_InitStruct;
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1); // Peripheral clock enable
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_9;                  // USART1 GPIO Configuration
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;        // PA9   ------> USART1_TX
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;      // PA10  ------> USART1_RX
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY); // USART1 DMA Init
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&USART1->DR); // USART1_RX Init
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)usart_rx_dma_buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, ARRAY_LEN(usart_rx_dma_buffer));
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_5); // Enable HT & TC interrupts
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
    NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0)); // DMA1_Channel5_IRQn interrupt configuration
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    USART_InitStruct.BaudRate = 115200; // USART configuration
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(USART1);
    LL_USART_EnableDMAReq_RX(USART1);
    LL_USART_EnableIT_IDLE(USART1);
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 1)); // USART interrupt
    NVIC_EnableIRQ(USART1_IRQn);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5); // Enable USART and DMA
    LL_USART_Enable(USART1);
}

void DMA1_Channel5_IRQHandler(void) {
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_5) && LL_DMA_IsActiveFlag_HT5(DMA1)) { // Check half-transfer complete interrupt
        LL_DMA_ClearFlag_HT5(DMA1);             /* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_5) && LL_DMA_IsActiveFlag_TC5(DMA1)) { // Check transfer-complete interrupt
        LL_DMA_ClearFlag_TC5(DMA1);             /* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }
    /* Implement other events when needed */
}

void USART1_IRQHandler(void)  { // Check for IDLE line interrupt
    if (LL_USART_IsEnabledIT_IDLE(USART1) && LL_USART_IsActiveFlag_IDLE(USART1)) {
        LL_USART_ClearFlag_IDLE(USART1);        /* Clear IDLE line flag */
        usart_rx_check();                       /* Check for data to process */
    }
    /* Implement other events when needed */
}

void TIM2_IRQHandler(void)  {
  dTick++;
  sysTick++;
  if(dTick > dDelay)  {
    dMulti();
    dTick = 0;
  }
  TIM2->SR &= ~TIM_SR_UIF;
}

void TIM4_IRQHandler(void)  {
  myTick++;
  TIM4->SR &= ~TIM_SR_UIF;
}

static void  LL_Init(void) {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0)); /* System interrupt init*/
    NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
}

static void TIM2_Init(void)  {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable TIM3 clock gating
  
  TIM2->PSC = 0;
  TIM2->ARR = 72; //sysclock is 72mhz, giving us 1mhz freq
  TIM2->CR1 |= TIM_CR1_URS; // only counter over/underflow generates an interrupt
  TIM2->DIER |= TIM_DIER_UIE; // UIE: update interrupt enable
  TIM2->EGR |= TIM_EGR_UG; // UG: update generation
  // TIM2->CR1 &= ~TIM_CR1_CEN;
  TIM2->CR1 |= TIM_CR1_CEN; // counter timer
  NVIC_EnableIRQ(TIM2_IRQn); // declare interrupt
  NVIC_SetPriority(TIM2_IRQn, 5);
}

static void TIM4_Init(void)  {
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //enable TIM4 clock gating
  TIM4->PSC = 0;
  TIM4->ARR = 72; //sysclock is 72mhz, giving us 1mhz freq
  TIM4->CR1 |= TIM_CR1_URS; // only counter over/underflow generates an interrupt
  TIM4->DIER |= TIM_DIER_UIE; // UIE: update interrupt enable
  TIM4->EGR |= TIM_EGR_UG; // UG: update generation
  // TIM4->CR1 |= TIM_CR1_CEN; // counter timer
  NVIC_EnableIRQ(TIM4_IRQn); // declare interrupt
  NVIC_SetPriority(TIM4_IRQn, 0);
}

void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);  /* Configure flash latency */
    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
        while (1) {}
    }
    LL_RCC_HSE_Enable();  /* Configure HSE */
    while (LL_RCC_HSE_IsReady() != 1) {}
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);  /* Configure PLL */
    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1) {}
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);  /* Set system clock */
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}
    LL_Init1msTick(72000000);  /* Configure systick */
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SetSystemCoreClock(72000000);
    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE END 4 */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
