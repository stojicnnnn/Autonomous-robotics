/* USER COD00E BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "biblioteka/tajmer.h"
#include "biblioteka/pid.h"
#include "biblioteka/step.h"
#include "biblioteka/pwm.h"
#include "biblioteka/motor.h"
#include "biblioteka/odometrija.h"
#include "biblioteka/kretanje.h"
#include "biblioteka/uart.h"
#include "biblioteka/hvataljke.h"
#include "biblioteka/senzori.h"
#include <math.h>
float pocetno;
float flagic=0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int j=0;
float test_theta=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  
       	/* MCU Configuration--------------------------------------------------------*/  	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */                                                                                                                                                                                                     HAL_Init();

  /* USER CODE BEGIN Init */
  tajmer_init();
  enc_init();
  motor_init();
  pid_init();
  init_PWM();
  uart_init();
 init_senzor();


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

 // lift_dizanje_vrh(sistemsko_vreme);
  /* USER CODE END 2 */
 // lift_dizanje_vrh(sistemsko_vreme);
  //lift_spustanje(sistemsko_vreme);
  		//AX12AsetEndless(2, 0);

  		//while(sistemsko_vreme<=1000);
  		//AX12Amove(2,1023);

  /* Infinite loop */


  /* USER CODE BEGIN WHILE */

while(0){
	motor2_set_PWM(1000);  //TESTIRATI
	  	motor1_set_PWM(1000);



}
	  while (1) {
		  if(j==0){ //-(60*M_PI)/180
	      pid_init();
	      racunanje_brzine(0, 0);

	      j=1;
		  }
		  if(j==1){ //-(60*M_PI)/180

			   //napred(-100,0,M_PI,600,4,1);
			  // Move forward
			      napred(200, 0, 0, 300, 4, 0);  // Move forward to (200, 0)


		  	      j=2;}
		  	    if(j==2){ //-(60*M_PI)/180

		  	    			   //napred(-100,0,M_PI,600,4,1)
		  	    	    // Move backward
		  	    		  	      j=2;


	  }



	  }






  while (0)
  {
    /* USER CODE END WHILE */


  //if((!(GPIOC->IDR & (1<<11))) & (flag_100s==0)){   // uslov za pocetak

	//set_test(0);



	//KOD ZA HOMOLOGCIJU
/*if(faza1==0){				//ZUTA BOJA KOD PANELA skraceno
pid_init();
racunanje_brzine(0,0);

}
else if(faza1==1){
rucica_napolje(sistemsko_vreme);
 napred(600,0,0,300,3,0);
 faza1=2;
}
else if(faza1==2){
 rucica_unutra(sistemsko_vreme);
 napred(0,0,M_PI,300,3,1);
 faza1=3;
}
else if(faza1==3){
  faza1=4;
}*/


	/*if(faza1==0){
				pid_init();
					 racunanje_brzine(0,0);
			 	 }
			    else if(faza1==1){

				  napred(700,200,0,600,4,0);
				  faza1=2;
			    }
				  else if(faza1==2){
					//  napred(0,0,M_PI,600,4,1);
			 	  faza1=3;
				  }
				  else if(faza1==3){
					//  hvataljka1_otvori(sistemsko_vreme);
					//  hvataljka2_otvori(sistemsko_vreme);
					// napred(483,-530,0,400,4,0);
			 		//faza1=4;								// x-93 y+100
				  }*/





 /*	 }
else{
	 racunanje_brzine(0,0);
	 set_test(1);
}*/
  }
 /* else{
  	set_test(1);
  	motor2_set_PWM(0);  //TESTIRATI
  	motor1_set_PWM(0);


  	// racunanje_brzine(0,0);
  }*/
  }

    /* USER CODE BEGIN 3 */


  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */

}

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
