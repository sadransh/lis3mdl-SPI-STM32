
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "lis3mdl.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#define NumOfSensors 1 //setup number of Sensors you want to use
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
HAL_StatusTypeDef LIS3MDL_ConnectAndConfig(LIS3MDL* Sensors,
		SPI_HandleTypeDef* spi, GPIO_TypeDef* gpio, uint16_t _pin,
		uint8_t performance, uint8_t mode, uint8_t scale, int8_t enableTemp);
//semi hosting
int _write(int fd, char *ptr, int len) {
	CDC_Transmit_FS((uint8_t*) ptr, len);
	return len;
}
/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();
	MX_USB_DEVICE_Init();

	/* USER CODE BEGIN 2 */
	HAL_StatusTypeDef status = HAL_BUSY;
	uint8_t res = 0x00;
	LIS3MDL Sensor[NumOfSensors];
	LIS3MDL_ConnectAndConfig(&Sensor[0],&hspi3,SPI3CS8_GPIO_Port,SPI3CS8_Pin,LIS3MDL_PERFORMANCE_ULTRA_HIGH,LIS3MDL_MODE_CONTINUOUS,LIS3MDL_SCALE_16_GAUSS,1);

//Config other sensors if you

//	(LIS3MDL_readStatus(&Sensor[0], &res) == 0) ?
//			printf("LIS3MDL Status :  %d\r\n", res) :
//			printf("failed to get status\r\n");

	int16_t val[3] = { 0 };

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		LIS3MDL_readTemperature(&Sensor[0], &res);
		LIS3MDL_readAxis(&Sensor[0], 0, &val[0]);
		LIS3MDL_readAxis(&Sensor[0], 1, &val[1]);
		LIS3MDL_readAxis(&Sensor[0], 2, &val[2]);

		printf("%d   %d    %d    %d \n", res, val[0], val[1], val[2]);
		HAL_Delay(100);
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {
}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef LIS3MDL_ConnectAndConfig(LIS3MDL* Sensors,
		SPI_HandleTypeDef* spi, GPIO_TypeDef* gpio, uint16_t _pin,
		uint8_t performance, uint8_t mode, uint8_t scale, int8_t enableTemp) {
	HAL_StatusTypeDef status = HAL_ERROR;
	while (HAL_OK
			!= LIS3MDL_setup(&Sensors[0], &hspi3, SPI3CS8_GPIO_Port,
					SPI3CS8_Pin)) {
		printf("can't init LIS3MDL\r\n");
		HAL_Delay(500);
	}
	printf("Connected to LIS3MDL...\r\n");
	if (enableTemp)
		status = LIS3MDL_enableTemperature(&Sensors[0]);
//	(status == HAL_ERROR) ? return HAL_ERROR :status = HAL_ERROR ;
	//(LIS3MDL_readTemperature(&res) ==0) ? printf("LIS3MDL temp : %d\r\n",res) : printf("failed to get status\n") ;
	status = LIS3MDL_setPerformance(&Sensors[0],
			LIS3MDL_PERFORMANCE_ULTRA_HIGH);

	status = LIS3MDL_setMode(&Sensors[0], LIS3MDL_MODE_CONTINUOUS);
	status = LIS3MDL_setScale(&Sensors[0], LIS3MDL_SCALE_16_GAUSS);
	return HAL_OK;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

