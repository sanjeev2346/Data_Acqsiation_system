/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>


FATFS fs;
FIL fil;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
void ADC_Select_CH0(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
void ADC_Select_CH1(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
void ADC_Select_CH2(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
void ADC_Select_CH3(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t ADC_Value[4],
				 VBAT_Out_Raw_Voltage,VBAT_Out_Voltage,VBAT_Out_Raw_current,VBAT_Out_current,VBAT_current,V_Raw_speed,V_speed,
				 rpm,rphr,speed,normal,echo,sports,side_stand,charging,DAS,vechicle_on,brake,left_ind, right_ind,
				 low_beam, high_beam,alert,latitude,longitude;
int days,mont,years,hours,mins,secs,ADC_Valu0,ADC_Valu1,ADC_Valu2,ADC_Valu4;

uint8_t batteryPercentage;

float VRT,VR,RT,ln,TX,contemp;
float b = 3977, rto = 1000, vcc = 5, R = 1000, T0 = 15+273.15;
//VBAT_Out_Voltage,VBAT_current,speed,contemp,normal,echo,sports,side_stand,charging,vechicle_on, DAS,brake,left_ind,right_ind,low_beam,high_beam
char VBAT_Out_Voltage_string[50];
char VBAT_current_string[50];
char speed_string[50];
char contemp_string[50];
char normal_string[50];
char echo_string[50];
char sports_string[50];
char side_stand_string[50];
char charging_string[50];
char vechicle_on_string[50];
char DAS_string[50];
char brake_string[50];
char left_ind_string[50];
char right_ind_string[50];
char low_beam_string[50];
char high_beam_string[50];
char soc_string[50];
char dod_string[50];
char batteryPercentage_string[50];
char location_string[50];
char time_string[50];


uint8_t TxData[8],TxData1[8],TxData2[8],TxData3[8],TxData4[8],TxData5[8],TxData6[8],TxData7[8],TxData8[8],TxData9[8],TxData10[8],TxData11[8],TxData12[8],TxData13[8],TxData14[8],TxData15[8],TxData16[8],TxData17[8],TxData18[8],TxData19[8],TxData20[8],TxData21[8],TxData22[8],TxData23[8],TxData24[8],TxData25[8],TxData26[8],TxData27[8],TxData28[8],TxData29[8],TxData30[8],TxData31[8];

#define VOLTAGE_MAX 84       // Maximum battery voltage (fully charged)
#define VOLTAGE_mins 54       // minsimum battery voltage (considered empty)


// Define the lookup table for voltage and SOC
#define TABLE_SIZE 11 // Number of entries in the table
const uint16_t voltage_table[TABLE_SIZE] = {84, 81, 78, 75, 72, 69, 66, 63, 60, 57, 54};
const uint8_t soc_table[TABLE_SIZE] = {100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0};

// Lookup table for voltage, current, and DoD
const uint16_t current_table[TABLE_SIZE] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50};
const uint8_t dod_table[TABLE_SIZE] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};


char GGA[100];
char RMC[100];

//GPSSTRUCT gpsData;

int flagGGA = 0, flagRMC = 0;
char time [50];
char location [50];
char NS;
char EW;


int VCCTimeout = 5000; // GGA or RMC will not be received if the VCC is not sufficient

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Function to look up SOC based on voltage
uint8_t lookupSOC(uint16_t voltage) {
    // Search the voltage table for the closest match
    for (int i = 0; i < TABLE_SIZE; i++) {
        if (voltage >= voltage_table[i]) {
            // Linear interpolation between two nearest points
            if (i == TABLE_SIZE - 1 || voltage <= voltage_table[i + 1]) {
                // Calculate SOC using linear interpolation
                uint8_t soc = soc_table[i] + (voltage - voltage_table[i]) * (soc_table[i + 1] - soc_table[i]) / (voltage_table[i + 1] - voltage_table[i]);
                return soc;
            }
        }
    }
    // Default return value if voltage is out of range
    return 0;
}

// Function to look up DoD based on voltage and current
uint8_t lookupDoD(uint16_t voltage, uint16_t current) {
    // Search the voltage and current table for the closest match
    for (int i = 0; i < TABLE_SIZE; i++) {
        if (voltage >= voltage_table[i] && current >= current_table[i]) {
            // Linear interpolation between two nearest points
            if (i == TABLE_SIZE - 1 || (voltage <= voltage_table[i + 1] && current <= current_table[i + 1])) {
                // Calculate DoD using linear interpolation
                uint8_t dod = dod_table[i] + (voltage - voltage_table[i]) * (dod_table[i + 1] - dod_table[i]) / (voltage_table[i + 1] - voltage_table[i]);
                return dod;
            }
        }
    }
    // Default return value if voltage or current is out of range
    return 0;
}

uint8_t calculateBatteryPercentage(float voltage)
{
    // Calculate battery percentage
    float percentage = ((voltage - VOLTAGE_mins) / (VOLTAGE_MAX - VOLTAGE_mins)) * 100;

    // Ensure percentage is within valid range (0 to 100)
    if (percentage < 0)
    {
        percentage = 0;
    }
    else if (percentage > 100)
    {
        percentage = 100;
    }

    return (uint8_t)percentage;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

				  //char msg[10];
				  char uart_buf[100];
				  	int uart_buf_len;
				  	//const int resistorValue = 10000;




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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  f_mount(&fs, "", 0);
    	    f_open(&fil, "DAS.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
    	  	f_lseek(&fil, fil.fsize);
    	  	f_puts("Logging Vechicle Data...\n", &fil);
    	  	f_close(&fil);

    	 //Ringbuf_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  ADC_Select_CH0();
	 	 	  	 	  HAL_ADC_Start(&hadc1);
	 	 	  	 	  HAL_ADC_PollForConversion(&hadc1, 1000);
	 	 	  	 	 ADC_Valu0 = HAL_ADC_GetValue(&hadc1);

	 	 	        VBAT_Out_Raw_Voltage		=		(((ADC_Valu0)*3.3)/4095);
	 	 	         VBAT_Out_Voltage			=		(((VBAT_Out_Raw_Voltage * (68+2.2))/2.2));
	 	 	         HAL_ADC_Stop(&hadc1);

	 	 	         ADC_Select_CH1();
	 	 	         	  HAL_ADC_Start(&hadc1);
	 	 	         	  HAL_ADC_PollForConversion(&hadc1, 1000);
	 	 	         	ADC_Valu1 = HAL_ADC_GetValue(&hadc1);

	 	 	         VBAT_Out_Raw_current		=		(((ADC_Valu1)*3.3)/4095);
	 	 	  	     VBAT_Out_current			=		(((VBAT_Out_Raw_Voltage * (68+2.2))/2.2));
	 	 	  	     VBAT_current          =      VBAT_Out_current/0.75;
	 	 	  	     HAL_ADC_Stop(&hadc1);

	 	 	  	     ADC_Select_CH2();
	 	 	  	     	  HAL_ADC_Start(&hadc1);
	 	 	  	     	  HAL_ADC_PollForConversion(&hadc1, 1000);
	 	 	  	     	 ADC_Valu2 = HAL_ADC_GetValue(&hadc1);

	 	 	         V_Raw_speed		=		(((ADC_Valu2)*3.3)/4095);
	 	 	  		 V_speed			=		(((V_Raw_speed* (51+4.7))/4.7)); //VBAT_Out_Voltage,VBAT_current,speed,contemp
	 	 	  		 rpm = (V_speed/39)*652;
	 	 	  	     rphr = rpm*60;
	 	 	  	     speed =0.001356*rphr;
	 	 	  	     HAL_ADC_Stop(&hadc1);

	 	 	  	     ADC_Select_CH3();
	 	 	  	    	  HAL_ADC_Start(&hadc1);
	 	 	  	    	  HAL_ADC_PollForConversion(&hadc1, 1000);
	 	 	  	    	  ADC_Valu4 = HAL_ADC_GetValue(&hadc1);
	 	 	  	     VRT = (((ADC_Valu4)*3.3)/4095);      //Conversion to voltage
	 	 	  		 VR = vcc - VRT;
	 	 	     	 RT = VRT / (VR / R);               //Resistance of RT

	 	 	  		 ln = log(RT / rto);
	 	 	  		 TX = (1 / ((ln / b) + (1 / T0))); //Temperature from thermistor

	 	 	  		 contemp = TX - 273.15;
	 	 	  		HAL_ADC_Stop(&hadc1);


	  	  	 	  	    uint16_t voltage = VBAT_Out_Voltage;
	  	  	 	  	    uint16_t current =  VBAT_current;
	  	  	 	  	    uint8_t dod = lookupDoD(voltage, current);
	  	  	 	  	    uint8_t soc = lookupSOC(voltage);
	  	  	 	  	 // Calculate battery percentage
	  	  	 	  	    batteryPercentage = calculateBatteryPercentage(VBAT_Out_Voltage);

	  	  	 	  	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);




	  	  	 	  			  	 	  		   	 		if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_3))  { //Check if button pressed

	  	  	 	  			  	 	  		   	 			 	  			 normal = 1; //vechicle normal mode


	  	  	 	  			  	 	  		   	 		}
	  	  	 	  			  	 	  		   	 		else{

	  	  	 	  			  	 	  		   	                              normal = 0;

	  	  	 	  			  	 	  		   	 		}

	  	  	 	  			  	 	  		   	 	if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15))  { //Check if button pressed

	  	  	 	  			  	 	  		   	 			 	  		  echo = 1; //vechicle echo mode


	  	  	 	  			  	 	  		   	 	}
	  	  	 	  			  	 	  		   	 	else{

	  	  	 	  			  	 	  		   	 			 	  	  	 echo = 0;//VBAT_Out_Voltage,VBAT_current,speed,contemp,normal,echo,sports,

	  	  	 	  			  	 	  		   	 	}
	  	  	 	  			  	 	  		   	 	if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_12))  { //Check if button pressed

	  	  	 	  			  	 	  		   	 			  	  		  sports = 1; //vechicle sports mode


	  	  	 	  			  	 	  		   	 	}
	  	  	 	  			  	 	  		   	 	else{

	  	  	 	  			  	 	  		   	 			  	  		  sports = 0;

	  	  	 	  			  	 	  		   	 	}
	  	  	 	  			  	 	  		   	 	if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_13))  { //Check if button pressed

	  	  	 	  			  	 	  		   	 				  		    side_stand = 1; //side stand status check


	  	  	 	  			  	 	  		   	 	}
	  	  	 	  			  	 	  		   	 	else{

	  	  	 	  			  	 	  		   	 				  		   side_stand = 0;
	  	  	 	  			  	 	  		   	 	}
	  	  	 	  			  	 	  		   	 	if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_14))  { //Check if button pressed

	  	  	 	  			  	 	  		   	 			  	  		  	charging = 1; //charging status check


	  	  	 	  			  	 	  		   	 	}
	  	  	 	  			  	 	  		   	 	else{

	  	  	 	  			  	 	  		   	 			  	  		    charging = 0;

	  	  	 	  			  	 	  		   	 	}
	  	  	 	  			  	 	  		   	  if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_15))  { //Check if button pressed

	  	  	 	  			  	 	  		   	 			  	  		   vechicle_on = 1; //vechicle status check


	  	  	 	  			  	 	  		   	 	}
	  	  	 	  			  	 	  		   	 	else{

	  	  	 	  			  	 	  		   	 			  	  		  vechicle_on = 0;

	  	  	 	  			  	 	  		   	 	}
	  	  	 	  			  	 	  		   	  if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_4))  { //Check if button pressed

	  	  	 	  			  	 	  		   	 			  	  	  		  DAS = 1; // DAS status check
	  	  	 	  			  	 	  		   	 			  	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);


	  	  	 	  			  	 	  		   	 	 }
	  	  	 	  			  	 	  		   	 	else{

	  	  	 	  			  	 	  		   	 			  	  	  		 DAS = 0;

	  	  	 	  			  	 	  		   	 	}
	  	  	 	  			  	 	  		   	 	if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_7))  { //Check if button pressed

	  	  	 	  			  	 	  		   	 		 	  			  	  brake = 1; // brake status check


	  	  	 	  			  	 	  		   	 		 }
	  	  	 	  			  	 	  		   	 	else{

	  	  	 	  			  	 	  		   	 		 	  			  	   brake = 0;

	  	  	 	  			  	 	  		   	 	}
	  	  	 	  			  	 	  		   	 	 if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_6))  { //Check if button pressed

	  	  	 	  			  	 	  		   	 		 	  			  	 	left_ind = 1; // indicator status check


	  	  	 	  			  	 	  		   	 		  }
	  	  	 	  			  	 	  		   	 		 else{

	  	  	 	  			  	 	  		   	 		 	  			  	 	 left_ind = 0;

	  	  	 	  			  	 	  		   	 		}
	  	  	 	  			  	 	  		   	 	 if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5))  { //Check if button pressed

	  	  	 	  			  	 	  		   	 		 	  			  right_ind = 1; // indicator status check

	  	  	 	                                           }
	  	  	 	  			  	 	  		   	 		else{

	  	  	 	  			  	 	  		   	 		 	  			    right_ind = 0;
	  	  	 	  			  	 	  		   	 		 	 }
	  	  	 	  			  	 	  		   	 	if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_8))  { //Check if button pressed

	  	  	 	  			  	 	  		   	 		 		           low_beam = 1; // low beam status check


	  	  	 	  			  	 	  		   	 	 }
	  	  	 	  			  	 	  		   	 	else{

	  	  	 	  			  	 	  		   	 		 	                low_beam = 0;

	  	  	 	  			  	 	  		   	 		}
	  	  	 	  		 		   	 			  if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_9))  { //Check if button pressed

	  	  	 	  			  	 	  		   	 			 		  	 high_beam = 1; // high beam status check

	  	  	 	                                         }
	  	  	 	  			  	 	  		   	 	else{//VBAT_Out_Voltage,VBAT_current,speed,contemp,normal,echo,sports,side_stand,charging,vechicle_on, DAS,brake,left_ind,right_ind,low_beam,high_beam
	  	  	 	  			  	 	  		   	 		                 high_beam = 0;

	  	  	 	  			  	 	  		   	 		 	}

	  	  	 	  		 		   	 			  if((side_stand == 1) && (charging == 1)&&(( soc<=5))&&( (contemp<=15)||(contemp>40))){
	  	  	 	  		 		   	 			        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	  	  	 	  		 		   	 	                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	  	  	 	  		 		   	 	                 alert=1;// Alert signal
	  	  	 	  		 		   	 			  }
	  	  	 	  		 		   	 			  else{
	  	  	 	  		 		   	            		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	  	  	 	  		 		   	 				 	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	  	  	 	  		 		   	 				     alert=0;
	  	  	 	  		 		   	 			  }
	  	  	 	  		 		   	 			memset(VBAT_Out_Voltage_string, 0, sizeof(VBAT_Out_Voltage_string));
	  	  	 	  		 		   	 					 		   	 			 	      	        memset(VBAT_current_string, 0, sizeof(VBAT_current_string));
	  	  	 	  		 		   	 					 		   	 			 	      	        memset(speed_string, 0, sizeof(speed_string));

	  	  	 	  		 		   	 					 		   	 		                        memset(contemp_string, 0, sizeof(contemp_string));
	  	  	 	  		 		   	 					 		   	 			 	      	        memset(normal_string, 0, sizeof(normal_string));
	  	  	 	  		 		   	 					 		   	 			 	      	        memset(echo_string, 0, sizeof(echo_string));

	  	  	 	  		 		   	 					 		   	 		                        memset(sports_string, 0, sizeof(sports_string));
	  	  	 	  		 		   	 					 		   	 			 	      	        memset(side_stand_string, 0, sizeof(side_stand_string));
	  	  	 	  		 		   	 					 		   	 			 	      	        memset(charging_string, 0, sizeof(charging_string));

	  	  	 	  		 		   	 					 		   	 		                        memset(vechicle_on_string, 0, sizeof(vechicle_on_string));
	  	  	 	  		 		   	 					 		   	 			 	      	        memset(DAS_string, 0, sizeof(DAS_string));
	  	  	 	  		 		   	 					 		   	 			 	      	        memset(brake_string, 0, sizeof(brake_string));

	  	  	 	  		 		   	 					 		   	 		                        memset(left_ind_string, 0, sizeof(left_ind_string));
	  	  	 	  		 		   	 					 		   	 			 	      	        memset(right_ind_string, 0, sizeof(right_ind_string));
	  	  	 	  		 		   	 					 		   	 			 	      	        memset(low_beam_string, 0, sizeof(low_beam_string));

	  	  	 	  		 		   	 					 		   	 		                        memset(high_beam_string, 0, sizeof(high_beam_string));
	  	  	 	  		 		   	 					 		   	 			 	      	       // memset(temp_string, 0, sizeof(temp_string));
	  	  	 	  		 		   	 					 		   	 			 	      	       // memset(press_string, 0, sizeof(press_string));

	  	  	 	  		 		   	 			 		   	 		                        memset(soc_string, 0, sizeof(soc_string));
	  	  	 	  		 		   	 			 		   	 			 	      	        memset(dod_string, 0, sizeof(dod_string));
	  	  	 	  		 		   	 			 		   	 			 	      	        memset(batteryPercentage_string, 0, sizeof(batteryPercentage_string));





	  	  	 	  		 		   	 					 		   	 			 	                sprintf(VBAT_Out_Voltage_string, "VBAT_Out_Voltage %03.1f%% ",VBAT_Out_Voltage);

	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(VBAT_current_string, "VBAT_current %03.1fC ", VBAT_current);
	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(speed_string, "speed %03.1fhPa\n", speed);
	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(contemp_string, "temperature %03.1f%% ",contemp);
	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(normal_string, "normal %03.1fC ", normal);
	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(echo_string, "echo %03.1fhPa\n",echo);

	  	  	 	  		 		   	 					 		   	 			 	                sprintf(sports_string, "sports %03.1f%% ",sports);

	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(side_stand_string, "side_stand %03.1fC ", side_stand);
	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(charging_string, "charging %03.1fhPa\n", charging);
	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(vechicle_on_string, "vechicle_on %03.1f%% ",vechicle_on);
	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(DAS_string, "DAS %03.1fC ", DAS);
	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(brake_string, "brake %03.1fhPa\n",brake);

	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(left_ind_string, "left_ind %03.1fC ", left_ind);
	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(right_ind_string, "right_ind %03.1fhPa\n", right_ind);
	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(low_beam_string, "low_beam %03.1f%% ",low_beam);

	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(high_beam_string, "high_beam %03.1fC ", high_beam);

	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(dod_string, "dod %03.1fC ", dod);
	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(soc_string, "soc %03.1fhPa\n", soc);
	  	  	 	  		 		   	 					 		   	 			 	      	 	    sprintf(batteryPercentage_string, "batteryPercentage %03.1f%% ",batteryPercentage);

	  	  	 	  		 		   	 					 		   	 			 	      	f_open(&fil, "DAS.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_lseek(&fil, fil.fsize);

	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(VBAT_Out_Voltage_string, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(VBAT_current_string, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(contemp_string, &fil);

	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(normal_string, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(echo_string, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(sports_string, &fil);

	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(side_stand_string, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(charging_string, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(vechicle_on_string, &fil);

	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(DAS_string, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(brake_string, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(left_ind_string, &fil);

	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(right_ind_string, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(low_beam_string, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(high_beam_string, &fil);

	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(dod_string, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(soc_string, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(batteryPercentage_string, &fil);

	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(time, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_puts(location, &fil);
	  	  	 	  		 		   	 					 		   	 			 	      		 	        f_close(&fil);

	  	  	 	  			// HAL_UART_Transmit (&huart1, data, sizeof (data), 1000);//VBAT_Out_Voltage,VBAT_current,speed,contemp,normal,echo,sports,side_stand,charging,vechicle_on, DAS,brake,left_ind,right_ind,low_beam,high_beam
	  	  	 	   uart_buf_len = sprintf(uart_buf,"{\"voltage\":%d,\"current\":%d,\"speed\":%d,\"temp\":%d,\"normal\":%d,\"echo\":%d,\"sports\":%d,\"side_stand\":%d,\"charging\":%d,\"vechicle_on\":%d,\"DAS\":%d,\"brake\":%d,\"left_ind\":%d,\"right_ind\":%d,\"low_beam\":%d,\"high_beam\":%d,\"soc\":%d,\"dod\":%d,\"batteryPercentage\":%d,\"latitude\":13.0728,\"NS\":N,\"longitude\":80.2042,\"EW\":E,\"hour\":23,\"min\":30,\"sec\":15,\"day\":29,\"mon\":03,\"year\":2003,\"trip\":60,\"range\":80}\n",(int)VBAT_Out_Voltage,(int)VBAT_current,(int)speed,(int)contemp,(int)normal,(int)echo,(int)sports,(int)side_stand,(int)charging,(int)vechicle_on,(int)DAS,(int)brake,(int)left_ind,(int)right_ind,(int)low_beam,(int)high_beam,(int)soc,(int)dod);//,(int)batteryPercentage,(int)latitude,NS,(int)longitude,EW,(int)hours,(int)mins,(int)secs,(int)days,(int)mont,(int)years);

	  	  	 	  	 HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf,uart_buf_len, 1000);
	  	  	 	  			 HAL_Delay(1000);
    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB3
                           PB4 PB5 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
