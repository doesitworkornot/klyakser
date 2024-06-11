/**
  ******************************************************************************
  * @file    BSP/Src/main.c 
  * @author  MCD Application Team
  * @brief   This example code shows how to use the STM32429I-Discovery BSP Drivers
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stlogo.h"
#include "doodler.h"
#include "platform.h"



/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup BSP
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
int pos = 0;
int to_add = 0;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/



/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


void jump_stage(doodler_s* doodle) {
	for (int y = 0; y < JUMP_HEIGHT; y++){
		doodle->y++;
	}
}

//int collision(platforms* plts, doodler_s doodle){
//	for (int i = 0; i < NUM_PLATFORMS; i++){
//		if (-plts[i].y + doodle.y + 60 > 0 &&
//		   0 <= doodle.x - plts[i].x && doodle.x - plts[i].x <= 80) return 1;
//	}
//	return 0;
//}

int collision(platforms* plts, doodler_s doodle){
	for (int i = 0; i < NUM_PLATFORMS; i++){
		int a = doodle.x - (plts[i].x - 60);
		int b = doodle.y - (plts[i].y - 60);
		if ((a >= 0 && a <= 140) && (b >= 0 && b <= 80)) return 1;
	}
	return 0;
}

#define RCC_AHB1ENR_GPIOF_EN  ((uint32_t)0x00000020) // Бит для включения тактирования GPIOF
#define RCC_AHB1ENR_GPIOG_EN  ((uint32_t)0x00000040) // Бит для включения тактирования GPIOG

// Функция для инициализации GPIO без HAL
void GPIO_Init(void) {
    // Включение тактирования портов GPIOF и GPIOG
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOF_EN | RCC_AHB1ENR_GPIOG_EN;

    // Ожидание пока тактирование активируется (холостая операция)
    volatile uint32_t delay = RCC->AHB1ENR;

    // Настройка пинов PF7 и PF9 как входов с подтягивающими резисторами
    GPIOF->MODER &= ~((3 << (2 * 7)) | (3 << (2 * 9))); // Режим входа для PF7 и PF9
    GPIOF->PUPDR &= ~((3 << (2 * 7)) | (3 << (2 * 9))); // Очистить подтягивающие резисторы
    GPIOF->PUPDR |= (1 << (2 * 7)) | (1 << (2 * 9));    // Включить подтягивающие резисторы на PF7 и PF9

    // Настройка пинов PG13 и PG14 как выходов для светодиодов
    GPIOG->MODER &= ~((3 << (2 * 13)) | (3 << (2 * 14))); // Очистить режимы
    GPIOG->MODER |= (1 << (2 * 13)) | (1 << (2 * 14));    // Режим выхода для PG13 и PG14
    GPIOG->OTYPER &= ~((1 << 13) | (1 << 14)); // Режим push-pull для PG13 и PG14
    GPIOG->OSPEEDR &= ~((3 << (2 * 13)) | (3 << (2 * 14))); // Низкая скорость для PG13 и PG14
    GPIOG->OSPEEDR |= (1 << (2 * 13)) | (1 << (2 * 14));    // Установить низкую скорость
}

uint8_t IsButtonPressed(GPIO_TypeDef *GPIOx, uint16_t pinNumber) {
    return !(GPIOx->IDR & (1 << pinNumber)); // Кнопка нажата, если значение 0
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{ 
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  GPIO_Init();
  /* Configure LED3 and LED4 */
//  BSP_LED_Init(LED3);
//  BSP_LED_Init(LED4);
  
  /* Configure the system clock to 180 MHz */

  doodler_s doodle = {0, 240, doodle_char};
  platforms plts[NUM_PLATFORMS];

  plts[0].enabled = 1;
  plts[0].x = 0;
  plts[0].y = 300;

  plts[1].enabled = 1;
  plts[1].x = 80;
  plts[1].y = 100;





  
  /* Configure USER Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  /*##-1- Initialize the LCD #################################################*/
  /* Initialize the LCD */
  BSP_LCD_Init();
  
  /* Initialize the LCD Layers */
  BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER);
  
  SystemClock_Config();

  int jump_stage = 0, per = 0;

  while (1)
  {

	if (per == 90000){
		to_add = pos / 10000;
		pos = 0;
		doodle.x += to_add;
		if (collision(plts, doodle) && jump_stage == 0){
			jump_stage = JUMP_HEIGHT;
		}

		if (jump_stage > 0){
			jump_stage--;
			doodle.y -= 5;
		}

		else {
			doodle.y += 5;
		}

		Display_field(plts, doodle);
		per = 0;
	}
	if (IsButtonPressed(GPIOF, 7) == 0) {
		GPIOG->ODR |= (1 << 13);
		pos--;
	} else {
		GPIOG->ODR &= ~(1 << 13);
	}


	if (IsButtonPressed(GPIOF, 9) == 0) {
		GPIOG->ODR |= (1 << 14);
		pos++;
	} else {
		GPIOG->ODR &= ~(1 << 14);
	}

	per++;
//    if(BSP_PB_GetState(BUTTON_KEY) == RESET)
//    {
//      while (BSP_PB_GetState(BUTTON_KEY) == RESET);
//
//      BSP_examples[DemoIndex++].DemoFunc();
//
//      if(DemoIndex >= COUNT_OF_EXAMPLE(BSP_examples))
//      {
//#ifdef EE_M24LR64
//        /* Increment number of loops which be used by EEPROM example */
//        NbLoop++;
//#endif /* EE_M24LR64 */
//        DemoIndex = 0;
//      }
//      Display_DemoDescription();
//    }
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
    
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/**
  * @brief  Display main demo messages
  * @param  None
  * @retval None
  */

void move_down(platforms* plts, int hm){
	for (int i = 0; i < NUM_PLATFORMS; i++){
		if (plts[i].enabled){
			plts[i].y += hm;
			if (plts[i].y > 300)
				plts[i].y = 0;
		}
	}
}



void Display_field(platforms* plts, doodler_s doodle){
  BSP_LCD_SelectLayer(1);


  /* Clear the LCD */
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_DrawBitmap(0, 0, (uint8_t *)platform);

  for(int i = 0; i < NUM_PLATFORMS; i++){

	  if (plts[i].enabled){
		  BSP_LCD_DrawBitmap(plts[i].x, plts[i].y, (uint8_t *)platform);
	  }
  }

  BSP_LCD_DrawBitmap(doodle.x, doodle.y, doodle.img);
}


/**
  * @brief  Check for user input
  * @param  None
  * @retval Input state (1 : active / 0 : Inactive)
  */
uint8_t CheckForUserInput(void)
{
  if(BSP_PB_GetState(BUTTON_KEY) == RESET)
  {
    while (BSP_PB_GetState(BUTTON_KEY) == RESET);
    return 1;
  }
  return 0;
}

/**
  * @brief  Toggle LEDs
  * @param  None
  * @retval None
  */
void Toggle_Leds(void)
{
  static uint8_t ticks = 0;
  
  if(ticks++ > 100)
  {
    BSP_LED_Toggle(LED3);
    BSP_LED_Toggle(LED4);
    ticks = 0;
  }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 if (GPIO_Pin == KEY_BUTTON_PIN)
 {
	 int x = 0;
//   ubKeyPressed = SET;
 }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */
