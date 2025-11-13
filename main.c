/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "i2c_lcd.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Macchina a stati per menu
typedef enum {
	MENU_NAV = 0,   // Navigazione principale: selezione Batteria/Spazzola
	SUB_BATT,       // Sottomenu Batteria
	SUB_SPAZ        // Sottomenu Spazzola
} MenuState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENC_DIV          4     // passi encoder per scatto logico
#define BTN_PORT         GPIOA
#define BTN_PIN          GPIO_PIN_10
#define BTN_DEBOUNCE_MS  500 //era 120
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint32_t voltage = 0;
uint64_t light_value = 0;
uint16_t adc_value = 0;
uint32_t rawCounter = 0;
uint32_t Counter = 0;
uint32_t lastCounter = 0;
uint8_t counter_butt = 0;
uint32_t rawcounterVel = 0;
uint32_t counterVel = 0;

MenuState stato_menu = MENU_NAV;

// Per tasto (edge + debounce)
uint8_t prev_btn = GPIO_PIN_SET;      // con pull-up: rilasciato=SET
uint32_t last_btn_tick = 0;

int32_t encoder_last = 0;

uint32_t push = 0;
uint32_t push_prev = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint8_t push_pressed(void) {
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    if (push_prev >= 3000 && push < 3000 && (now - last_tick > 50)) {
        last_tick = now;
        push_prev = push;
        return 1;
    }
    push_prev = push;
    return 0;
}



static inline uint8_t button_falling_edge(void) {
	uint8_t now = HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN);
	uint32_t t = HAL_GetTick();

	// falling edge: HIGH -> LOW con debounce
	if (now == GPIO_PIN_RESET && prev_btn == GPIO_PIN_SET) {
		if (t - last_btn_tick > BTN_DEBOUNCE_MS) {
			last_btn_tick = t;
			prev_btn = now;
			return 1;
		}
	}
	// aggiorna stato precedente
	if (now != prev_btn) {
		// se risale, aggiorno comunque con debounce semplice
		if (t - last_btn_tick > 10) {
			prev_btn = now;
			last_btn_tick = t;
		}
	}
	return 0;
}

static inline void lcd_show_menu_nav(uint32_t sel) {
	if (sel == 0) {
		lcd_put_cursor(0, 0);
		lcd_send_string(">Batteria        ");
		lcd_put_cursor(1, 0);
		lcd_send_string(" Spazzola        ");
	} else {
		lcd_put_cursor(0, 0);
		lcd_send_string(" Batteria        ");
		lcd_put_cursor(1, 0);
		lcd_send_string(">Spazzola        ");
	}
}
/* USER CODE END 0 */

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	lcd_init();

	__HAL_TIM_SET_COUNTER(&htim3, 0);
	Counter = 0;
	counterVel = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		HAL_ADC_Start(&hadc1);

		//partitore
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); //il processore resta fermo fino al completamento della conversione
		adc_value = HAL_ADC_GetValue(&hadc1);


		/*float Vref = 3.3;
		 float Vadc = (voltage / 4095) * Vref; //tensione reale es. 3.3V

		 float Vbatt = Vadc * (40000.0) / 10000.0; //tensione reale es. 16.8V*/

		//int percentuale = 0;
		float voltage = ((float) adc_value / 4095.0f) * 3.3f; // tensione ADC
		voltage *= (40000.0) / 10000.0; // correzione partitore

		float x0 = 12.0f;   // punto medio della curva per LiPo 4S
		float k = 5.0f;     // pendenza della curva
		float sigmoid = 1.0f / (1.0f + expf(-k * (voltage - x0)));
		float percentuale = sigmoid * 100.0f;

		//fotoresistore
		/*HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		 light_value = HAL_ADC_GetValue(&hadc1);*/

		HAL_ADC_Stop(&hadc1);

		switch (stato_menu) {

		case MENU_NAV: {
			rawCounter = __HAL_TIM_GET_COUNTER(&htim3);
			Counter = (rawCounter / 4) % 2;

			lcd_show_menu_nav(Counter);

			// Tasto: entra nel sottomenu selezionato
			if (button_falling_edge()) {
				if (Counter == 0) {
					stato_menu = SUB_BATT;
					// quando entro in batteria, non uso l'encoder -> azzero per sicurezza
					__HAL_TIM_SET_COUNTER(&htim3, 0);
				} else {
					stato_menu = SUB_SPAZ;
					// allineo l'encoder alla velocità corrente (step logici * ENC_DIV)
					__HAL_TIM_SET_COUNTER(&htim3, counterVel * ENC_DIV);
				}
			}
		}
			break;

		case SUB_BATT: {
			char buffer[16];
			sprintf(buffer, "Rimanente: %.1f%%  ", percentuale);

			lcd_put_cursor(0, 0);
			lcd_send_string(buffer);
			lcd_put_cursor(1, 0);
			lcd_send_string("          ");
			if (percentuale <= 50.0) {
				lcd_put_cursor(1, 0);
				lcd_send_string("Ricaricare    ");
				HAL_Delay(100);
			}

			// Tasto: torna al menu principale
			if (button_falling_edge()) {
				stato_menu = MENU_NAV;
				// allineo l'encoder alla selezione attuale (Counter rimane quello di prima)
				__HAL_TIM_SET_COUNTER(&htim3, Counter * ENC_DIV);
			}
		}

			break;

		case SUB_SPAZ: {
			rawcounterVel = __HAL_TIM_GET_COUNTER(&htim3);
			int32_t delta = rawcounterVel - encoder_last;

			if (delta >= ENC_DIV && counterVel < 10) {
				counterVel++;
				encoder_last = rawcounterVel;
			} else if (delta <= -ENC_DIV && counterVel > 0) {
				counterVel--;
				encoder_last = rawcounterVel;
			}

			char buffer_vel[16];
			sprintf(buffer_vel, "Velocita': %d         ", counterVel);

			lcd_put_cursor(0, 0);
			lcd_send_string(buffer_vel);
			lcd_put_cursor(1, 0);
			lcd_send_string("         ");

			if (button_falling_edge()) {
				stato_menu = MENU_NAV;
				__HAL_TIM_SET_COUNTER(&htim3, Counter * ENC_DIV);
			}
		}
			break;

		}

		//motori, controllati da selettore velocità
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, counterVel*25.5); //ho cambiato il Counter Period (ARR) da CubeMx


		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, counterVel*25.5); //ho cambiato il Counter Period (ARR) da CubeMx

		/* if (light_value>3800){
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		 }*/

	}
  /* USER CODE END 3 */
}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
