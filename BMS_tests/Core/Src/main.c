/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "math.h"
#include "string.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEED_VALUE    0x38
#define CRC_LENGTH     6
#define FRAME_LENGTH  40
#define POLYNOM     0x59
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

HAL_StatusTypeDef status;
uint8_t dummy = 0;
static uint8_t response_arr[5];
uint8_t burst_response_arr[95];
const uint8_t crcTable[] = {
      0, 100, 200, 172, 244, 144,  60,  88, 140, 232,  68,  32, 120,  28, 176, 212,
    124,  24, 180, 208, 136, 236,  64,  36, 240, 148,  56,  92,   4,  96, 204, 168,
    248, 156,  48,  84,  12, 104, 196, 160, 116,  16, 188, 216, 128, 228,  72,  44,
    132, 224,  76,  40, 112,  20, 184, 220,   8, 108, 192, 164, 252, 152,  52,  80,
    148, 240,  92,  56,  96,   4, 168, 204,  24, 124, 208, 180, 236, 136,  36,  64,
    232, 140,  32,  68,  28, 120, 212, 176, 100,   0, 172, 200, 144, 244,  88,  60,
    108,   8, 164, 192, 152, 252,  80,  52, 224, 132,  40,  76,  20, 112, 220, 184,
     16, 116, 216, 188, 228, 128,  44,  72, 156, 248,  84,  48, 104,  12, 160, 196,
     76,  40, 132, 224, 184, 220, 112,  20, 192, 164,   8, 108,  52,  80, 252, 152,
     48,  84, 248, 156, 196, 160,  12, 104, 188, 216, 116,  16,  72,  44, 128, 228,
    180, 208, 124,  24,  64,  36, 136, 236,  56,  92, 240, 148, 204, 168,   4,  96,
    200, 172,   0, 100,  60,  88, 244, 144,  68,  32, 140, 232, 176, 212, 120,  28,
    216, 188,  16, 116,  44,  72, 228, 128,  84,  48, 156, 248, 160, 196, 104,  12,
    164, 192, 108,   8,  80,  52, 152, 252,  40,  76, 224, 132, 220, 184,  20, 112,
     32,  68, 232, 140, 212, 176,  28, 120, 172, 200, 100,   0,  88,  60, 144, 244,
     92,  56, 148, 240, 168, 204,  96,   4, 208, 180,  24, 124,  36,  64, 236, 136
};

uint8_t VCELL_EN[15], d_rdy_Vcell[15], vsum_batt1_0, data_ready_vsum, data_ready_v_battdiv, SOC, OVR_LATCH, CONF_CYCLIC_EN,
		DUTY_ON, VSUM_OV, VSUM_UV, TimedBalacc, TimedBalTimer, bal_on, eof_bal, OVR_LATCH, TCYCLE_OVFm,  sense_plus_open,
		sense_minus_open,  Otchip,  VANA_OV,  VDIG_OV,  VTREF_UV,  VTREF_OV,  VREG_UV,  VREG_OV, VCOM_OV, VCOM_UV,  wu_gpio7,
		wu_spi, wu_isoline,  wu_faulth,  wu_cyc_wup, loss_agnd, loss_dgnd, loss_cgnd, loss_gndref, TrimmCalOk,CoCouOvF,
		EoBtimeerror, GPI_fastchg_OT[10], GPI_OPEN[10],VBAT_COMP_BIST_FAIL, VREG_COMP_BIST_FAIL, VCOM_COMP_BIST_FAIL,
		VTREF_COMP_BIST_FAIL, BAL_SHORT[15], EEPROM_CRC_ERR_CAL_FF, EEPROM_DWNLD_DONE, HWSC_DONE,  VBAT_OPEN,  CELL_OPEN[15], EEPROM_CRC_ERR_SECT_0,
		Comm_timeout_flt, EEPROM_CRC_ERR_CAL_RAM, RAM_CRC_ERR, VCELL_UV[15], VBATT_WRN_OV, VBATT_WRN_UV, VBATTCRIT_UV, VSUM_UV,
		VCELL_OV[15], bal_on, eof_bal, VBATTCRIT_OV, VSUM_OV, GPIO_OT[10], GPIO_UT[10], GPOon[10], VCELL_BAL_UV[15],
		Fault_L_line_status, GPOshort[10], GPIO_BIST_FAIL, HeartBeat_fault, FaultHline_fault, FaultH_EN, HeartBeat_En,
		curr_sense_ovc_norm, OSCFail, clk_mon_en, clk_mon_init_done, CoulombCounter_en, CoCouOvF, curr_sense_ovc_sleep,
		GPIO3_OT, d_rdy_gpio3, GPIO4_OT, d_rdy_gpio4, GPIO5_OT, d_rdy_gpio5, GPIO6_OT, d_rdy_gpio6, GPIO7_OT, d_rdy_gpio7,
		GPIO8_OT, d_rdy_gpio8, GPIO9_OT, d_rdy_gpio9, TrimmCalOk, d_rdy_vtref, GPIO3_UT, GPIO4_UT, GPIO5_UT, GPIO6_UT, GPIO7_UT,
		GPIO8_UT, GPIO9_UT, bal_on, eof_bal, OTchip, TempChip, BAL_OPEN[15];

uint16_t VCell[15], VBATT_DIV, MUX_BIST_FAIL, HeartBeatCycle, BIST_BAL_COMP_HS_FAIL, BIST_BAL_COMP_LS_FAIL,
		 OPEN_BIST_FAIL, CoulombCntTime, CoulombCounter_msb, CoulombCounter_lsb, GPIO4_MEAS,
		 GPIO3_MEAS, GPIO5_MEAS, GPIO6_MEAS, GPIO7_MEAS, GPIO8_MEAS, GPIO9_MEAS, VTREF_MEAS;
uint32_t vsum_batt19_2, CUR_INST_calib, CUR_INST_synch, CUR_INST_calib;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void L9963_CrcCalc8bitLookupTab (uint64_t *);
uint16_t L9963_CrcVer8bitLookupTab (uint64_t *);
void readBurst0x78 (void);
void readBurst0x7A (void);
void readBurst0x7B (void);
void singleFrameTransaction(char *,char *,char *,char *,char *,char *, bool);
void configRegADCV_CONV(char *,char *,char *,char *,char *,char *,char *,char *,char *,char *,char *);
void configRegNCYCLE_PROG_1(char *,char *,char *,char *,char *,char *,char *,char *,char *,char *);
void configRegNCYCLE_PROG_2(char *,char *,char *,char *,char *,char *,char *);
void balanceCells();
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
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //---------------------------SLEEP-----------------------------------------

	  singleFrameTransaction("1","0","00000","0000000","00","000000000000000000",false);
	  /*
	  char PA[2] = "1"; // 0 = MISO answer ; 1 = MOSI command
	  char RW[2] = "0"; // 0 = read ; 1 = write
	  char DevID[6] = "00000";
	  char Addr[8] = "0000000";
	  char GSW[3] = "00"; // 0x = ok ; 1x = internal failure detected
	  char data[19] = "000000000000000000"; //VCELL_EN 1,2,13,14
	  char CRC_code[7]="000000";
	  char frame[41];
	  char frame_a[41];

	  strcpy(frame, PA);
	  strcat(frame,RW);
	  strcat(frame,DevID);
	  strcat(frame,Addr);
	  strcat(frame,GSW);
	  strcat(frame,data);
	  strcat(frame,CRC_code);
	  strcpy(frame_a,frame);
	  uint8_t command_data_arr[5];


	  for (int i=0;i<5;i++){
	     command_data_arr[i]=0;
	     for(int j=7;j>=0;j--){
	         if(frame_a[i*8+j]=='1')
	            command_data_arr[i]=command_data_arr[i] + pow(2,7-j);
	         }
	  }

	  uint64_t command_data = 0;
	  for (int i=39;i>=0;i--){
		  command_data = command_data + (frame[i]-48)*pow(2,39-i);
	  }
	  L9963_CrcCalc8bitLookupTab(&command_data);
	  command_data_arr[4]=command_data_arr[4] | (uint8_t)(command_data & 0b00111111);


	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // NCS low
	  status = HAL_SPI_TransmitReceive(&hspi2, command_data_arr, response_arr, 5, HAL_MAX_DELAY);
	  while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){};	//wait for transaction to end
	  //HAL_Delay(5);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // NCS High
	  */

	  //------------------------------INIT---------------------------------------------------
	  singleFrameTransaction("1","1","00000","0000001","00","000010000000000010",false);


	  //--------------------------NORMAL--------------------------------------------
	  // READ DEV_GEN_CFG:
	  singleFrameTransaction("1","0","00001","0000001","00","000000000000000000",true);
	  dummy+=1;
	  // ENABLE CELLS:
	  singleFrameTransaction("1","1","00001","0011100","00","000011000000000011",true);
	  dummy+=1;



	  // PERFORM CYCLIC ADC VOLTAGE CONVERSION:

	  configRegNCYCLE_PROG_1("10","000","000","000","1","1","1","0","0","0");
	  dummy+=1;
	  configRegNCYCLE_PROG_2("1","0","000","000","001","011","001");
	  dummy+=1;
	  configRegADCV_CONV("0","0","0","000","0","0","0","0","0","011","1");
	  dummy+=1;
	  configRegADCV_CONV("0","0","1","000","0","0","0","0","0","011","1");
	  dummy+=1;

	  /*
	  singleFrameTransaction("1","1","00001","0001110","00","010000000000010000",true);
	  dummy+=1;
	  singleFrameTransaction("1","1","00001","0001111","00","100000000000000000",true);
	  dummy+=1;
	  	  */

	  while(1){
	  // READ BURSTS
	  HAL_Delay(1000);
	  readBurst0x78();
	  readBurst0x7A();
	  readBurst0x7B();
	  //configRegADCV_CONV("0","1","1","000","0","0","0","0","0","011","1");
	  dummy+=1; // <--optional breakpoint here
};
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void singleFrameTransaction(char PA[],char RW[],char DevID[],char Addr[],char GSW[],char data[], bool acquireResponse){
	  char frame[41];char CRC_code[7]="000000";
	  strcpy(frame, PA);
	  strcat(frame,RW);
	  strcat(frame,DevID);
	  strcat(frame,Addr);
	  strcat(frame,GSW);
	  strcat(frame,data);
	  strcat(frame,CRC_code);
	  uint8_t command_data_arr[5];


	  for (int i=0;i<5;i++){
	     command_data_arr[i]=0;
	     for(int j=7;j>=0;j--){
	         if(frame[i*8+j]=='1')
	            command_data_arr[i]=command_data_arr[i] + pow(2,7-j);
	         }
	  }

	  uint64_t command_data = 0;
	  for (int i=39;i>=0;i--){
		  command_data = command_data + (frame[i]-48)*pow(2,39-i);
	  }
	  L9963_CrcCalc8bitLookupTab(&command_data);
	  command_data_arr[4]=command_data_arr[4] | (uint8_t)(command_data & 0b00111111);


	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // NCS low
	  status = HAL_SPI_TransmitReceive(&hspi2, command_data_arr, response_arr, 5, HAL_MAX_DELAY);
	  while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){};	//wait for transaction to end
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // NCS High

	  if(acquireResponse){
		  //use dummy command (read DEV_GEN_CFG reg) to acquire the out of frame response to the previous command
		  command_data_arr[0]=0b10000010;
		  command_data_arr[1]=0b00000100;
		  command_data_arr[2]=0b00000000;
		  command_data_arr[3]=0b00000000;
		  command_data_arr[4]=0b00010111;

		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // NCS low
		  status = HAL_SPI_TransmitReceive(&hspi2, command_data_arr, response_arr, 5, HAL_MAX_DELAY);
		  while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){};	//wait for transaction to end
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // NCS High
	  }
}

void configRegADCV_CONV(char ADC_CROSS_CHECK[],char SOC[],char CONF_CYCLIC_EN[],char ADC_FILTER_SOC[],char GPIO_CONV[],
		char GPIO_TERM_CONV[],char CELL_TERM_CONV[],char BAL_TERM_CONV[],char HWSC[],char TCYCLE[],char CYCLIC_CONTINOUS[]){
	char data[19];
	strcpy(data,ADC_CROSS_CHECK);
	strcat(data,"0"); //TCYCLE_OVF
	strcat(data,SOC);
	strcat(data,"0"); //OVR_LATCH
	strcat(data,CONF_CYCLIC_EN);
	strcat(data,"0"); //DUTY_ON
	strcat(data,ADC_FILTER_SOC);
	strcat(data,GPIO_CONV);
	strcat(data,GPIO_TERM_CONV);
	strcat(data,CELL_TERM_CONV);
	strcat(data,BAL_TERM_CONV);
	strcat(data,HWSC);
	strcat(data,TCYCLE);
	strcat(data,CYCLIC_CONTINOUS);

	singleFrameTransaction("1","1","00001","0001101","00",data,true);

}

void configRegNCYCLE_PROG_1(char T_CELL_SET[],char NCYCLE_GPIO_TERM[],char NCYCLE_CELL_TERM[],char NCYCLE_BAL_TERM[],char BAL_TIM_AUTO_PAUSE[],
		char BAL_AUTO_PAUSE[],char CYCLIC_UPDATE[], char CROSS_ODD_EVEN_CELL[], char PCB_open_en_odd_curr[], char PCB_open_en_even_curr[]){
	char data[19];
	strcpy(data,T_CELL_SET);
	strcat(data,NCYCLE_GPIO_TERM);
	strcat(data,NCYCLE_CELL_TERM);
	strcat(data,NCYCLE_BAL_TERM);
	strcat(data,BAL_TIM_AUTO_PAUSE);
	strcat(data,BAL_AUTO_PAUSE);
	strcat(data,CYCLIC_UPDATE);
	strcat(data,CROSS_ODD_EVEN_CELL);
	strcat(data,PCB_open_en_odd_curr);
	strcat(data,PCB_open_en_even_curr);
	strcat(data,"0"); //NOREG0

	singleFrameTransaction("1","1","00001","0001110","00",data,true);
}

void configRegNCYCLE_PROG_2(char VTREF_EN[],char VTREF_DYN_EN[],char NCYCLE_GPIO[],char NCYCLE_HWSC[], char ADC_FILTER_CYCLE[], char TCYCLE_SLEEP[],
		char ADC_FILTER_SLEEP[]){
	char data[19];
	strcpy(data,VTREF_EN);
	strcat(data,VTREF_DYN_EN);
	strcat(data,NCYCLE_GPIO);
	strcat(data,NCYCLE_HWSC);
	strcat(data,"0"); //NOREG9
	strcat(data,ADC_FILTER_CYCLE);
	strcat(data,TCYCLE_SLEEP);
	strcat(data,ADC_FILTER_SLEEP);

	singleFrameTransaction("1","1","00001","0001111","00",data,true);
}

void readBurst0x78 (){
	//FOR VOLTAGE CONVERSIONS CHECK DOCUMENTATION -> TABLE 4.5.2


	uint8_t command_data_arr[5]; //initial burst command data to be sent by MCU, 8 bit array form
	uint64_t command_data = 0; //initial burst command data to be sent by MCU
	uint8_t nofFrames = 18; //number of frames received from burst command
	uint8_t data_burst_arr[nofFrames*5]; //dummy data sent by MCU during burst frame receival
	for(int i=0;i<nofFrames*5;i++) data_burst_arr[i]=0; //all zeroes as per manufacturer documentation
	data_burst_arr[nofFrames*5-5]=0b10000010;
	data_burst_arr[nofFrames*5-4]=0b00000100;
	data_burst_arr[nofFrames*5-3]=0b00000000;
	data_burst_arr[nofFrames*5-2]=0b00000000;
	data_burst_arr[nofFrames*5-1]=0b00010111;
	uint8_t burst_response_arr[nofFrames*5]; // data received by MCU during burst response

	char PA[2] = "1"; // 0 = MISO answer ; 1 = MOSI command
    char RW[2] = "0"; // 0 = read ; 1 = write
    char DevID[6] = "00001";
	char Addr[8] = "1111000"; 	// 0x78 burst command
	char GSW[3] = "00"; // 0x = ok ; 1x = internal failure detected
	char data[19] = "111111111111111111";
	char CRC_code[7] = "000000";
	char frame[41];

	strcpy(frame, PA);
	strcat(frame,RW);
	strcat(frame,DevID);
	strcat(frame,Addr);
	strcat(frame,GSW);
	strcat(frame,data);
	strcat(frame,CRC_code);

    //convert char array to 8 bit number array:
    for (int i=4;i>=0;i--){
    	command_data_arr[i]=0;
    	for(int j=7;j>=0;j--){
    		command_data_arr[i]=command_data_arr[i] + (frame[i*8+j]-48)*pow(2,7-j);
    	}
    }
    //convert char array to number:
    for (int i=39;i>=0;i--){
    	command_data = command_data + (frame[i]-48)*pow(2,39-i);
    }

    L9963_CrcCalc8bitLookupTab(&command_data); //compute CRC for command
    command_data_arr[4]=command_data_arr[4] | (uint8_t)(command_data & 0b00111111);

    uint8_t nOfTrials = 0;
    TRANSACTION: //for goto instruction in case there is a problem with sent/received data
	nOfTrials+=1;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // NCS low
    status = HAL_SPI_TransmitReceive(&hspi2, command_data_arr, response_arr, 5, HAL_MAX_DELAY);
    while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){};	//wait for transaction to end
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // NCS High

    HAL_Delay(4);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // NCS low
    status = HAL_SPI_TransmitReceive(&hspi2, data_burst_arr, burst_response_arr, nofFrames*5, HAL_MAX_DELAY);
    while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){};	//wait for transaction to end
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // NCS High

    uint64_t frame_check = ((uint64_t)burst_response_arr[0]<<32) | ((uint64_t)burst_response_arr[1]<<24)|
    					   ((uint64_t)burst_response_arr[2]<<16) | ((uint64_t)burst_response_arr[3]<<8) | (uint64_t)burst_response_arr[4];

    //check first frame (for special frames - documentation 4.2.4.4)
    if((frame_check == 0xC1FCFFFD08 || //CRC error frame
        frame_check == 0xC1FCFFFCDE || //Busy frame
	    frame_check == 0xC1FCFFFC87 || //Timeout frame
	   frame_check == 0xC1FCFFFC6C) && //Not Expected frame
	   nOfTrials<10)
    goto TRANSACTION; //repeat process in case of error


    //check CRC for every frame from burst response:
    for(int i=0; i<nofFrames; i++){
    	frame_check = ((uint64_t)burst_response_arr[0]<<32) | ((uint64_t)burst_response_arr[1]<<24)|
    	    		  ((uint64_t)burst_response_arr[2]<<16) | ((uint64_t)burst_response_arr[3]<<8) | (uint64_t)burst_response_arr[4];
    	if(L9963_CrcVer8bitLookupTab(&frame_check)==0 && nOfTrials<10) goto TRANSACTION; //repeat process in case of error
    }

    /*
    "burst_response_arr" is an 8bit array containing all the burst_response_arr received from a burst command,
    stored as bytes one after the other
    example: 0x78 command --> 18 burst_response_arr response --> 720 bits/90 bytes
            frame 1: burst_response_arr[0] --> burst_response_arr[4]
            frame 1 actual data: burst_response_arr[2] (bit 17->bit 10), burst_response_arr[3] (bit 9->bit 2),
                                 first 2 MSB's of burst_response_arr[4] (bit 1->bit 0)
            frame 2: burst_response_arr[5] --> burst_response_arr[9]
            frame 2 actual data: burst_response_arr[7], burst_response_arr[8], first 2 MSB's of burst_response_arr[9]
            .
            .
            .
            frame 18: burst_response_arr[85] --> burst_response_arr[89]
            frame 18 actual data: frame[87],frame[88], first 2 MSB's of burst_response_arr[89]
    */
    for(int i=1;i<=14;i++){
        VCELL_EN[i]=burst_response_arr[(i-1)*5+2] >> 7;

        d_rdy_Vcell[i]=(burst_response_arr[(i-1)*5+2] >> 6) & 1; //&0b00000001, extract only LSB

        VCell[i]=(((burst_response_arr[(i-1)*5+2] & 0b00111111) << 8) | burst_response_arr[(i-1)*5+3]) << 2
                    | (burst_response_arr[(i-1)*5+4]>>6);
    }

    vsum_batt19_2 = (((burst_response_arr[(15-1)*5+2] << 8) | burst_response_arr[(15-1)*5+3]) << 2) | (burst_response_arr[(15-1)*5+4]>>6);

    vsum_batt1_0 = burst_response_arr[(16-1)*5+2] >> 6;
    VBATT_DIV = (((burst_response_arr[(16-1)*5+2] & 0b00111111) << 8 ) | burst_response_arr[(16-1)*5+3]) << 2 | (burst_response_arr[(16-1)*5+4]>>6);

    data_ready_vsum = burst_response_arr[(17-1)*5+2] >>7;
    data_ready_v_battdiv = (burst_response_arr[(17-1)*5+2] >> 6) & 1;
    SOC = (burst_response_arr[(17-1)*5+2] >> 5) & 1;
    OVR_LATCH = (burst_response_arr[(17-1)*5+2] >> 4) & 1;
    CONF_CYCLIC_EN = (burst_response_arr[(17-1)*5+2] >> 3) & 1;
    DUTY_ON = (burst_response_arr[(17-1)*5+2] >> 2) & 1;
    VSUM_OV = (burst_response_arr[(17-1)*5+2] >> 1) & 1;
    VSUM_UV = burst_response_arr[(17-1)*5+2]  & 1;
    TimedBalacc = burst_response_arr[(17-1)*5+3] >> 7;
    TimedBalTimer = burst_response_arr[(17-1)*5+3] & 0b01111111;
    bal_on = burst_response_arr[(17-1)*5+4] >> 7;
    eof_bal = (burst_response_arr[(17-1)*5+4] >> 6) & 1;

    CUR_INST_calib = (((burst_response_arr[(18-1)*5+2] <<8) | burst_response_arr[(18-1)*5+3]) << 2) | (burst_response_arr[(18-1)*5+4]>>6);

}



void readBurst0x7A (){
	uint8_t command_data_arr[5]; //initial burst command data to be sent by MCU, 8 bit array form
	uint64_t command_data = 0; //initial burst command data to be sent by MCU
	uint8_t nofFrames = 13; //number of frames received from burst command
	uint8_t data_burst_arr[nofFrames*5]; //dummy data sent by MCU during burst frame receival
	for(int i=0;i<nofFrames*5;i++) data_burst_arr[i]=0; //all zeroes as per manufacturer documentation
	data_burst_arr[nofFrames*5-5]=0b10000010;
	data_burst_arr[nofFrames*5-4]=0b00000100;
	data_burst_arr[nofFrames*5-3]=0b00000000;
	data_burst_arr[nofFrames*5-2]=0b00000000;
	data_burst_arr[nofFrames*5-1]=0b00010111;
	uint8_t burst_response_arr[nofFrames*5]; // data received by MCU during burst response

	char PA[2] = "1"; // 0 = MISO answer ; 1 = MOSI command
    char RW[2] = "0"; // 0 = read ; 1 = write
    char DevID[6] = "00001";
	char Addr[8] = "1111010"; 	// 0x7A burst command
	char GSW[3] = "00"; // 0x = ok ; 1x = internal failure detected
	char data[19] = "111111111111111111";
	char CRC_code[7] = "000000";
	char frame[41];

	strcpy(frame, PA);
	strcat(frame,RW);
	strcat(frame,DevID);
	strcat(frame,Addr);
	strcat(frame,GSW);
	strcat(frame,data);
	strcat(frame,CRC_code);

    //convert char array to 8 bit number array:
    for (int i=4;i>=0;i--){
    	command_data_arr[i]=0;
    	for(int j=7;j>=0;j--){
    		command_data_arr[i]=command_data_arr[i] + (frame[i*8+j]-48)*pow(2,7-j);
    	}
    }
    //convert char array to number:
    for (int i=39;i>=0;i--){
    	command_data = command_data + (frame[i]-48)*pow(2,39-i);
    }

    L9963_CrcCalc8bitLookupTab(&command_data); //compute CRC for command
    command_data_arr[4]=command_data_arr[4] | (uint8_t)(command_data & 0b00111111);

    uint8_t nOfTrials=0;
    TRANSACTION: //for goto instruction in case there is a problem with sent/received data
	nOfTrials +=1;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // NCS low
    status = HAL_SPI_TransmitReceive(&hspi2, command_data_arr, response_arr, 5, HAL_MAX_DELAY);
    while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){};	//wait for transaction to end
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // NCS High

    HAL_Delay(4);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // NCS low
    status = HAL_SPI_TransmitReceive(&hspi2, data_burst_arr, burst_response_arr, nofFrames*5, HAL_MAX_DELAY);
    while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){};	//wait for transaction to end
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // NCS High

    uint64_t frame_check = ((uint64_t)burst_response_arr[0]<<32) | ((uint64_t)burst_response_arr[1]<<24)|
    					   ((uint64_t)burst_response_arr[2]<<16) | ((uint64_t)burst_response_arr[3]<<8) | (uint64_t)burst_response_arr[4];

    //check first frame (for special frames - documentation 4.2.4.4)
    if((frame_check == 0xC1FCFFFD08 || //CRC error frame
        frame_check == 0xC1FCFFFCDE || //Busy frame
	    frame_check == 0xC1FCFFFC87 || //Timeout frame
	    frame_check == 0xC1FCFFFC6C) && //Not Expected frame
	   nOfTrials<10)
    goto TRANSACTION; //repeat process in case of error


    //check CRC for every frame from burst response:
    for(int i=0; i<nofFrames; i++){
    	frame_check = ((uint64_t)burst_response_arr[0]<<32) | ((uint64_t)burst_response_arr[1]<<24)|
    	    		  ((uint64_t)burst_response_arr[2]<<16) | ((uint64_t)burst_response_arr[3]<<8) | (uint64_t)burst_response_arr[4];
    	if(L9963_CrcVer8bitLookupTab(&frame_check)==0 && nOfTrials<10) goto TRANSACTION; //repeat process in case of error
    }

    /*
    "burst_response_arr" is an 8bit array containing all the burst_response_arr received from a burst command,
    stored as bytes one after the other
    example: 0x78 command --> 18 burst_response_arr response --> 720 bits/90 bytes
            frame 1: burst_response_arr[0] --> burst_response_arr[4]
            frame 1 actual data: burst_response_arr[2] (bit 17->bit 10), burst_response_arr[3] (bit 9->bit 2),
                                 first 2 MSB's of burst_response_arr[4] (bit 1->bit 0)
            frame 2: burst_response_arr[5] --> burst_response_arr[9]
            frame 2 actual data: burst_response_arr[7], burst_response_arr[8], first 2 MSB's of burst_response_arr[9]
            .
            .
            .
            frame 18: burst_response_arr[85] --> burst_response_arr[89]
            frame 18 actual data: frame[87],frame[88], first 2 MSB's of burst_response_arr[89]
    */
    for (int i = 1; i <= 2; i++) {
        BAL_OPEN[i] = burst_response_arr[14] >> (8 - i) & 1;
        BAL_SHORT[i] = burst_response_arr[19] >> (8 - i) & 1;
        CELL_OPEN[i - 1] = burst_response_arr[24] >> (8 - i) & 1;
    }

    for (int i = 3; i <= 10; i++) {
        BAL_OPEN[i] = burst_response_arr[13] >> (i - 3) & 1;
        BAL_SHORT[i] = burst_response_arr[18] >> (i - 3) & 1;
        CELL_OPEN[i - 1] = burst_response_arr[23] >> (i - 3) & 1;
        if (i > 5)
            GPI_fastchg_OT[i - 2] = burst_response_arr[8] >> (i - 3) & 1;
    }

    GPI_fastchg_OT[9] = burst_response_arr[7] & 1;

    for (int i = 11; i <= 14; i++) {
        BAL_OPEN[i] = burst_response_arr[12] >> (i - 11) & 1;
        BAL_SHORT[i] = burst_response_arr[17] >> (i - 11) & 1;
        CELL_OPEN[i - 1] = burst_response_arr[22] >> (i - 11) & 1;
    }

    CELL_OPEN[14] = burst_response_arr[22] >> 4 & 1;

    OVR_LATCH = burst_response_arr[2] >> 7 & 1;
    TCYCLE_OVFm = burst_response_arr[2] >> 6 & 1;
    sense_plus_open = burst_response_arr[2] >> 5 & 1;
    sense_minus_open = burst_response_arr[2] >> 4 & 1;
    Otchip = burst_response_arr[2] >> 3 & 1;
    VANA_OV = burst_response_arr[2] >> 2 & 1;
    VDIG_OV = burst_response_arr[2] >> 1 & 1;
    VTREF_UV = burst_response_arr[2] & 1;
    VTREF_OV = burst_response_arr[3] >> 7 & 1;
    VREG_UV = burst_response_arr[3] >> 6 & 1;
    VREG_OV = burst_response_arr[3] >> 5 & 1;
    VCOM_OV = burst_response_arr[3] >> 4 & 1;
    VCOM_UV = burst_response_arr[3] >> 3 & 1;
    wu_gpio7 = burst_response_arr[3] >> 2 & 1;
    wu_spi = burst_response_arr[3] >> 1 & 1;
    wu_isoline = burst_response_arr[3] & 1;
    wu_faulth = burst_response_arr[4] >> 6 & 1;
    wu_cyc_wup = burst_response_arr[4] >> 7 & 1;


    loss_agnd = burst_response_arr[7] >> 7 & 1;
    loss_dgnd = burst_response_arr[7] >> 6 & 1;
    loss_cgnd = burst_response_arr[7] >> 5 & 1;
    loss_gndref = burst_response_arr[7] >> 4 & 1;
    TrimmCalOk = burst_response_arr[7] >> 3 & 1;
    CoCouOvF = burst_response_arr[7] >> 2 & 1;
    EoBtimeerror = burst_response_arr[7] >> 1 & 1;
    GPI_OPEN[9] = burst_response_arr[(2 - 1) * 5 + 3] >> 1 & 1;
    GPI_OPEN[8] = burst_response_arr[(2 - 1) * 5 + 3] & 1;
    GPI_OPEN[7] = burst_response_arr[(2 - 1) * 5 + 4] >> 6 & 1;
    GPI_OPEN[6] = burst_response_arr[(2 - 1) * 5 + 4] >> 7 & 1;

    GPI_OPEN[5] = burst_response_arr[12] >> 7 & 1;
    GPI_OPEN[4] = burst_response_arr[12] >> 6 & 1;
    GPI_OPEN[3] = burst_response_arr[12] >> 5 & 1;
    EEPROM_DWNLD_DONE = burst_response_arr[12] >> 4 & 1;

    VBAT_COMP_BIST_FAIL = burst_response_arr[(4 - 1) * 5 + 2] >> 7 & 1;
    VREG_COMP_BIST_FAIL = burst_response_arr[(4 - 1) * 5 + 2] >> 6 & 1;
    VCOM_COMP_BIST_FAIL = burst_response_arr[(4 - 1) * 5 + 2] >> 5 & 1;
    VTREF_COMP_BIST_FAIL = burst_response_arr[(4 - 1) * 5 + 2] >> 4 & 1;

    EEPROM_CRC_ERR_CAL_FF = burst_response_arr[(5 - 1) * 5 + 2] >> 7 & 1;
    HWSC_DONE = burst_response_arr[(5 - 1) * 5 + 2] >> 6 & 1;
    VBAT_OPEN = burst_response_arr[(5 - 1) * 5 + 2] >> 5 & 1;

    for (int i = 1; i <= 2; i++) {
        VCELL_UV[i] = burst_response_arr[(6 - 1) * 5 + 4] >> (8 - i) & 1;
        VCELL_OV[i] = burst_response_arr[(7 - 1) * 5 + 4] >> (8 - i) & 1;
        VCELL_BAL_UV[i] = burst_response_arr[(9 - 1) * 5 + 4] >> (8 - i) & 1;
        GPIO_UT[i + 2] = burst_response_arr[(8 - 1) * 5 + 4] >> (8 - i) & 1;
    }

    for (int i = 3; i <= 10; i++) {
        VCELL_UV[i] = burst_response_arr[(6 - 1) * 5 + 3] >> (i - 3) & 1;
        VCELL_OV[i] = burst_response_arr[(7 - 1) * 5 + 3] >> (i - 3) & 1;
        VCELL_BAL_UV[i] = burst_response_arr[(9 - 1) * 5 + 3] >> (i - 3) & 1;

        if (i <= 7)
            GPIO_UT[i + 2] = burst_response_arr[(8 - 1) * 5 + 3] >> (i - 3) & 1;
        else
            GPIO_OT[i - 5] = burst_response_arr[(8 - 1) * 5 + 3] >> (i - 3) & 1;
    }

    for (int i = 11; i <= 14; i++) {
        VCELL_UV[i] = burst_response_arr[(6 - 1) * 5 + 2] >> (i - 11) & 1;
        VCELL_OV[i] = burst_response_arr[(7 - 1) * 5 + 2] >> (i - 11) & 1;
        VCELL_BAL_UV[i] = burst_response_arr[(9 - 1) * 5 + 2] >> (i - 11) & 1;
        GPIO_OT[i - 5] = burst_response_arr[(8 - 1) * 5 + 2] >> (i - 11) & 1;
    }

    EEPROM_CRC_ERR_SECT_0 = burst_response_arr[(6 - 1) * 5 + 2] >> 7 & 1;
    Comm_timeout_flt = burst_response_arr[(6 - 1) * 5 + 2] >> 6 & 1;
    EEPROM_CRC_ERR_CAL_RAM = burst_response_arr[(6 - 1) * 5 + 2] >> 5 & 1;
    RAM_CRC_ERR = burst_response_arr[(6 - 1) * 5 + 2] >> 4 & 1;

    VBATT_WRN_OV = burst_response_arr[(7 - 1) * 5 + 2] >> 7 & 1;
    VBATT_WRN_UV = burst_response_arr[(7 - 1) * 5 + 2] >> 6 & 1;
    VBATTCRIT_UV = burst_response_arr[(7 - 1) * 5 + 2] >> 5 & 1;
    VSUM_UV = burst_response_arr[(7 - 1) * 5 + 2] >> 4 & 1;

    bal_on = burst_response_arr[(8 - 1) * 5 + 2] >> 7 & 1;
    eof_bal = burst_response_arr[(8 - 1) * 5 + 2] >> 6 & 1;
    VBATTCRIT_OV = burst_response_arr[(8 - 1) * 5 + 2] >> 5 & 1;
    VSUM_OV = burst_response_arr[(8 - 1) * 5 + 2] >> 4 & 1;

    for (int i = 17; i > 13; i--) {
        GPOon[i - 11] = burst_response_arr[(9 - 1) * 5 + 2] >> (i - 10) & 1;
    }

    Fault_L_line_status = burst_response_arr[(10 - 1) * 5 + 2] >> 7 & 1;
    GPOon[9] = burst_response_arr[(10 - 1) * 5 + 2] >> 6 & 1;
    GPOon[8] = burst_response_arr[(10 - 1) * 5 + 2] >> 5 & 1;
    GPOon[7] = burst_response_arr[(10 - 1) * 5 + 2] >> 4 & 1;

    for (int i = 13; i > 9; i--) {
        GPOshort[i - 4] = burst_response_arr[(10 - 1) * 5 + 2] >> (i - 10) & 1;
    }
    GPOshort[5] = burst_response_arr[(10 - 1) * 5 + 3] >> 7 & 1;
    GPOshort[4] = burst_response_arr[(10 - 1) * 5 + 3] >> 6 & 1;
    GPOshort[3] = burst_response_arr[(10 - 1) * 5 + 3] >> 5 & 1;

    HeartBeat_fault = burst_response_arr[(11 - 1) * 5 + 2] >> 7 & 1;
    FaultHline_fault = burst_response_arr[(11 - 1) * 5 + 2] >> 6 & 1;
    FaultH_EN = burst_response_arr[(11 - 1) * 5 + 2] >> 5 & 1;
    HeartBeat_En = burst_response_arr[(11 - 1) * 5 + 2] >> 4 & 1;

    MUX_BIST_FAIL = (((burst_response_arr[(11 - 1) * 5 + 4] >> 6) & 0b00000011) | (burst_response_arr[(11 - 1) * 5 + 3] << 2)) | ((burst_response_arr[(11 - 1) * 5 + 2] & 0b00001111) << 8);

    GPIO_BIST_FAIL = ((burst_response_arr[(10 - 1) * 5 + 4] >> 6) & 0b00000011) | ((burst_response_arr[(11 - 1) * 5 + 3] & 0b00011111) << 2);

    curr_sense_ovc_sleep = burst_response_arr[(12 - 1) * 5 + 2] >> 7 & 1;
    HeartBeatCycle = burst_response_arr[(12 - 1) * 5 + 2] >> 4 & 0b00000111;
    BIST_BAL_COMP_HS_FAIL = ((burst_response_arr[(12 - 1) * 5 + 2] & 0b00001111) << 3) | ((burst_response_arr[(12 - 1) * 5 + 3] >> 5) & 0b00000111);
    BIST_BAL_COMP_LS_FAIL = ((burst_response_arr[(12 - 1) * 5 + 3] & 0b00011111) << 2) | (burst_response_arr[(12 - 1) * 5 + 4] >> 6);

    curr_sense_ovc_norm = burst_response_arr[(13 - 1) * 5 + 2] >> 7 & 1;
    OSCFail = burst_response_arr[(13 - 1) * 5 + 2] >> 6 & 1;
    clk_mon_en = burst_response_arr[(13 - 1) * 5 + 2] >> 5 & 1;
    clk_mon_init_done = burst_response_arr[(13 - 1) * 5 + 2] >> 4 & 1;

    OPEN_BIST_FAIL = (((burst_response_arr[(13 - 1) * 5 + 3] & 0b00001111) << 8) | (burst_response_arr[(13 - 1) * 5 + 3] << 2)) | (burst_response_arr[(13 - 1) * 5 + 4] >> 6);

}

void readBurst0x7B()
{
	uint8_t command_data_arr[5]; //initial burst command data to be sent by MCU, 8 bit array form
	uint64_t command_data = 0; //initial burst command data to be sent by MCU
	uint8_t nofFrames = 14; //number of frames received from burst command
	uint8_t data_burst_arr[nofFrames*5]; //dummy data sent by MCU during burst frame receival
	for(int i=0;i<nofFrames*5;i++) data_burst_arr[i]=0; //all zeroes as per manufacturer documentation
	data_burst_arr[nofFrames*5-5]=0b10000010;
	data_burst_arr[nofFrames*5-4]=0b00000100;
	data_burst_arr[nofFrames*5-3]=0b00000000;
	data_burst_arr[nofFrames*5-2]=0b00000000;
	data_burst_arr[nofFrames*5-1]=0b00010111;
	uint8_t burst_response_arr[nofFrames*5]; // data received by MCU during burst response

	char PA[2] = "1"; // 0 = MISO answer ; 1 = MOSI command
    char RW[2] = "0"; // 0 = read ; 1 = write
    char DevID[6] = "00001";
	char Addr[8] = "1111011"; 	// 0x7B burst command
	char GSW[3] = "00"; // 0x = ok ; 1x = internal failure detected
	char data[19] = "111111111111111111";
	char CRC_code[7] = "000000";
	char frame[41];

	strcpy(frame, PA);
	strcat(frame,RW);
	strcat(frame,DevID);
	strcat(frame,Addr);
	strcat(frame,GSW);
	strcat(frame,data);
	strcat(frame,CRC_code);

    //convert char array to 8 bit number array:
    for (int i=4;i>=0;i--){
    	command_data_arr[i]=0;
    	for(int j=7;j>=0;j--){
    		command_data_arr[i]=command_data_arr[i] + (frame[i*8+j]-48)*pow(2,7-j);
    	}
    }
    //convert char array to number:
    for (int i=39;i>=0;i--){
    	command_data = command_data + (frame[i]-48)*pow(2,39-i);
    }

    L9963_CrcCalc8bitLookupTab(&command_data); //compute CRC for command
    command_data_arr[4]=command_data_arr[4] | (uint8_t)(command_data & 0b00111111);

    uint8_t nOfTrials=0;
    TRANSACTION: //for goto instruction in case there is a problem with sent/received data
	nOfTrials+=1;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // NCS low
    status = HAL_SPI_TransmitReceive(&hspi2, command_data_arr, response_arr, 5, HAL_MAX_DELAY);
    while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){};	//wait for transaction to end
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // NCS High

    HAL_Delay(4);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // NCS low
    status = HAL_SPI_TransmitReceive(&hspi2, data_burst_arr, burst_response_arr, nofFrames*5, HAL_MAX_DELAY);
    while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){};	//wait for transaction to end
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // NCS High

    uint64_t frame_check = ((uint64_t)burst_response_arr[0]<<32) | ((uint64_t)burst_response_arr[1]<<24)|
    					   ((uint64_t)burst_response_arr[2]<<16) | ((uint64_t)burst_response_arr[3]<<8) | (uint64_t)burst_response_arr[4];

    //check first frame (for special frames - documentation 4.2.4.4)
    if((frame_check == 0xC1FCFFFD08 || //CRC error frame
        frame_check == 0xC1FCFFFCDE || //Busy frame
	    frame_check == 0xC1FCFFFC87 || //Timeout frame
	   frame_check == 0xC1FCFFFC6C) && //Not Expected frame
       nOfTrials<10)
    goto TRANSACTION; //repeat process in case of error


    //check CRC for every frame from burst response:
    for(int i=0; i<nofFrames; i++){
    	frame_check = ((uint64_t)burst_response_arr[0]<<32) | ((uint64_t)burst_response_arr[1]<<24)|
    	    		  ((uint64_t)burst_response_arr[2]<<16) | ((uint64_t)burst_response_arr[3]<<8) | (uint64_t)burst_response_arr[4];
    	if(L9963_CrcVer8bitLookupTab(&frame_check)==0 && nOfTrials<10) goto TRANSACTION; //repeat process in case of error
    }

    /*
    "burst_response_arr" is an 8bit array containing all the burst_response_arr received from a burst command,
    stored as bytes one after the other
    example: 0x7B command --> 14 burst_response_arr response --> 560 bits/70 bytes
            frame 1: burst_response_arr[0] --> burst_response_arr[4]
            frame 1 actual data: burst_response_arr[2] (bit 17->bit 10), burst_response_arr[3] (bit 9->bit 2),
                                 first 2 MSB's of burst_response_arr[4] (bit 1->bit 0)
            frame 2: burst_response_arr[5] --> burst_response_arr[9]
            frame 2 actual data: burst_response_arr[7], burst_response_arr[8], first 2 MSB's of burst_response_arr[9]
            .
            .
            .
            frame 15: burst_response_arr[65] --> burst_response_arr[69]
            frame 18 actual data: frame[67],frame[68], first 2 MSB's of burst_response_arr[69]
    */

    //frame1 0-4
     CoulombCounter_en = burst_response_arr[2] >> 7;
    CoCouOvF = (burst_response_arr[2] >> 6) & 1;

    CoulombCntTime= (((burst_response_arr[(1-1)*5+2] & 0b00111111) << 8) | burst_response_arr[(1-1)*5+3]) << 2
                    | (burst_response_arr[(1-1)*5+4]>>6);

     //frame2 5-9
      sense_plus_open = burst_response_arr[(2-1)*5+2] >> 7;
    sense_minus_open = (burst_response_arr[(2-1)*5+2] >> 6) & 1;
    CoulombCounter_msb= (((burst_response_arr[(2-1)*5+2] & 0b00111111) << 8) | burst_response_arr[(2-1)*5+3]) << 2
                    | (burst_response_arr[(2-1)*5+4]>>6);
     //frame3 10-14
    curr_sense_ovc_sleep = burst_response_arr[(3-1)*5+2] >> 7;
    curr_sense_ovc_norm = (burst_response_arr[(3-1)*5+2] >> 6) & 1;

    CoulombCounter_lsb= (((burst_response_arr[(3-1)*5+2] & 0b00111111) << 8) | burst_response_arr[(3-1)*5+3]) << 2
                    | (burst_response_arr[(3-1)*5+4]>>6);

     //frame4 15-19
    CUR_INST_synch = (((burst_response_arr[(4-1)*5+2] << 8) | (burst_response_arr[(4-1)*5+3]) << 2) | burst_response_arr[(4-1)*5+4]>>6);
     //frame5 20-24
    CUR_INST_calib = (((burst_response_arr[(5-1)*5+2] << 8) | (burst_response_arr[(5-1)*5+3]) << 2) | burst_response_arr[(5-1)*5+4]>>6);

     //frame6 25-29
     GPIO3_OT = burst_response_arr[(6-1)*5+2] >> 7;
     d_rdy_gpio3 = (burst_response_arr[(6-1)*5+2] >> 6) & 1;

     GPIO3_MEAS= (((burst_response_arr[(6-1)*5+2] & 0b00111111) << 8) | burst_response_arr[(6-1)*5+3]) << 2
                    | (burst_response_arr[(6-1)*5+4]>>6);
     //frame7 30-34
     GPIO4_OT = burst_response_arr[(7-1)*5+2] >> 7;
     d_rdy_gpio4 = (burst_response_arr[(7-1)*5+2] >> 6) & 1;

     GPIO4_MEAS= (((burst_response_arr[(7-1)*5+2] & 0b00111111) << 8) | burst_response_arr[(7-1)*5+3]) << 2
                    | (burst_response_arr[(7-1)*5+4]>>6);

     //frame8 35-39
     GPIO5_OT = burst_response_arr[(8-1)*5+2] >> 7;
     d_rdy_gpio5 = (burst_response_arr[(8-1)*5+2] >> 6) & 1;
     GPIO5_MEAS= (((burst_response_arr[(8-1)*5+2] & 0b00111111) << 8) | burst_response_arr[(8-1)*5+3]) << 2
                    | (burst_response_arr[(8-1)*5+4]>>6);

     //frame9 40-44
     GPIO6_OT = burst_response_arr[(9-1)*5+2] >> 7;
     d_rdy_gpio6 =(burst_response_arr[(9-1)*5+2] >> 6) & 1;
     GPIO6_MEAS= (((burst_response_arr[(9-1)*5+2] & 0b00111111) << 8) | burst_response_arr[(9-1)*5+3]) << 2
                    | (burst_response_arr[(9-1)*5+4]>>6);

     //frame10 45-49
     GPIO7_OT = burst_response_arr[(10-1)*5+2] >> 7;
     d_rdy_gpio7 =(burst_response_arr[(10-1)*5+2] >> 6) & 1;
     GPIO7_MEAS= (((burst_response_arr[(10-1)*5+2] & 0b00111111) << 8) | burst_response_arr[(10-1)*5+3]) << 2
                    | (burst_response_arr[(10-1)*5+4]>>6);

     //frame11 50-54
     GPIO8_OT = burst_response_arr[(11-1)*5+2] >> 7;
     d_rdy_gpio8 = (burst_response_arr[(11-1)*5+2] >> 6) & 1;
     GPIO8_MEAS= (((burst_response_arr[(11-1)*5+2] & 0b00111111) << 8) | burst_response_arr[(11-1)*5+3]) << 2
                    | (burst_response_arr[(11-1)*5+4]>>6);

    //frame12 55-59
     GPIO9_OT = burst_response_arr[(12-1)*5+2] >> 7;
     d_rdy_gpio9 =( burst_response_arr[(12-1)*5+2] >> 6) & 1;
     GPIO9_MEAS= (((burst_response_arr[(12-1)*5+2] & 0b00111111) << 8) | burst_response_arr[(12-1)*5+3]) << 2
                    | (burst_response_arr[(12-1)*5+4]>>6);


     //frame13 60-64
       TrimmCalOk = burst_response_arr[(13-1)*5+2] >> 7;
     d_rdy_vtref = (burst_response_arr[(13-1)*5+2] >> 6) & 1;
     VTREF_MEAS= (((burst_response_arr[(13-1)*5+2] & 0b00111111) << 8) | burst_response_arr[(13-1)*5+3]) << 2
                    | (burst_response_arr[(13-1)*5+4]>>6);

     //frame14 65-69


      GPIO3_UT= burst_response_arr[(14-1)*5+2] >>7;

      GPIO4_UT = (burst_response_arr[(14-1)*5+2] >> 6) & 1;
      GPIO5_UT = (burst_response_arr[(14-1)*5+2] >> 5) & 1;
      GPIO6_UT = (burst_response_arr[(14-1)*5+2] >> 4) & 1;
      GPIO7_UT = (burst_response_arr[(14-1)*5+2] >> 3) & 1;
      GPIO8_UT = (burst_response_arr[(14-1)*5+2] >> 2) & 1;
      GPIO9_UT = (burst_response_arr[(14-1)*5+2] >> 1) & 1;
      bal_on  = burst_response_arr[(14-1)*5+2]  & 1;
      eof_bal = burst_response_arr[(14-1)*5+3] >> 7;
      OTchip = (burst_response_arr[(14-1)*5+3] >> 6) & 1;
      TempChip =(((burst_response_arr[(14-1)*5+3]) << 2) | (burst_response_arr[(14-1)*5+4]>>6));

}

// ****************************************************************************
// Name:        L9963_CrcCalc8bitLookupTab
// Parameters:
//              uint64_t * pointer to frame
// Returns:
//               none
// Description:
//              CRC calculation based on precalculated 8bits table
//              FRAME_LENGTH  40 (could not be changed)
//              CRC_LENGTH     6 (could not be changed)
//              POLYNOM     0x59 (if changed the crcTable must be recalculated)
//              seed value is defined by  SEED_VALUE
// ****************************************************************************
void L9963_CrcCalc8bitLookupTab (uint64_t * frame){

    const uint8_t * crcTable_p = crcTable;
    uint8_t dataout;
    *frame &= (uint64_t)0xFFFFFFFFFF<<CRC_LENGTH;
    uint8_t tabPointer = (uint8_t)((*frame>>(4*8)) ^ ((uint32_t)SEED_VALUE<<2)); //highest 8bits (bit 32..39)
    dataout =  crcTable_p[tabPointer];

    tabPointer =  (uint8_t)((uint32_t)dataout ^ (((uint32_t)*frame)>>(3*8)));   //next  8bits (bit 24..31);
    dataout =  crcTable_p[tabPointer];

    tabPointer =  (uint8_t)((uint32_t)dataout ^ (((uint32_t)*frame)>>(2*8)));   //next  8bits (bit 16..23);
    dataout =  crcTable_p[tabPointer];

    tabPointer =  (uint8_t)((uint32_t)dataout ^ (((uint32_t)*frame)>>(1*8)));   //next  8bits (bit 8..15);
    dataout =  crcTable_p[tabPointer];

    dataout =  (uint8_t)((uint32_t)dataout ^ ((uint32_t)*frame));          //next  8bits (bit 0..7);

    if(0x80 & dataout ){                     //bit 7
        dataout = dataout ^ POLYNOM<<1;
    }
    if(0x40 & dataout ){                     //last bit 6, so the rest is bit 0..5
        dataout = dataout ^ POLYNOM;
    }
    *frame |= (uint64_t)dataout;
}

// ****************************************************************************
// Name:        L9963_CrcVer
// Parameters:
//              uint64_t * pointer to frame
// Returns:
//               0 - If CRC is Valid
//               1 - Wrong CRC
// Description:
//              CRC calculation based on precalculated 8bits table
//              FRAME_LENGTH  40 (could not be changed)
//              CRC_LENGTH     6 (could not be changed)
//              POLYNOM     0x59 (if changed the crcTable must be recalculated)
//              seed value is defined by  SEED_VALUE
// ****************************************************************************
uint16_t L9963_CrcVer8bitLookupTab (uint64_t * frame){
    const uint8_t * crcTable_p = crcTable;
    uint8_t dataout;
    uint8_t tabPointer = (uint8_t)((*frame>>(4*8)) ^ ((uint32_t)SEED_VALUE<<2)); //highest 8bits (bit 32..39)
    dataout =  crcTable_p[tabPointer];

    tabPointer =  (uint8_t)((uint32_t)dataout ^ (((uint32_t)*frame)>>(3*8)));   //next  8bits (bit 24..31);
    dataout =  crcTable_p[tabPointer];

    tabPointer =  (uint8_t)((uint32_t)dataout ^ (((uint32_t)*frame)>>(2*8)));   //next  8bits (bit 16..23);
    dataout =  crcTable_p[tabPointer];

    tabPointer =  (uint8_t)((uint32_t)dataout ^ (((uint32_t)*frame)>>(1*8)));   //next  8bits (bit 8..15);
    dataout =  crcTable_p[tabPointer];

    dataout =  (uint8_t)((uint32_t)dataout ^ ((uint32_t)*frame));          //next  8bits (bit 0..7);

    if(0x80 & dataout ){                     //bit 7
        dataout = dataout ^ POLYNOM<<1;
    }
    if(0x40 & dataout ){                     //last bit 6, so the rest is bit 0..5
        dataout = dataout ^ POLYNOM;
    }
    if(0 == dataout){
        return 0;
    }
    else{
        return 1;
    }
}

void balanceCells(){
	uint8_t activeCells=0;
	for(int i=1;i<=14;i++)
		activeCells+=VCELL_EN[i]; //acquire number of active cells
	uint8_t voltagesAscOrder[activeCells]; //cell voltages in ascending order
	uint8_t cells_VoltageAscOrder[activeCells]; //cell indexes in ascending order of voltage
	uint8_t balanceFlag[15]={0};

	//gather active cell indexes and their respective voltages:
	int j=0;
	for(int i=1;i<=14;i++){
		if(VCELL_EN[i]==1){
			voltagesAscOrder[j]=VCell[i];
			cells_VoltageAscOrder[j]=i;
			j++;
		}
	}

	//sort the 2 arrays in ascending order of voltage:
	for(int i=0;i<activeCells;i++){
		for(j=i+1;j<activeCells-1;j++){
			if(voltagesAscOrder[i]>voltagesAscOrder[j]){
				uint8_t aux = voltagesAscOrder[i];
				voltagesAscOrder[i] = voltagesAscOrder[j];
				voltagesAscOrder[j] = aux;

				cells_VoltageAscOrder[i]=j;
				cells_VoltageAscOrder[j]=i;
			}
		}
	}

	//determine which cells need balancing and set flags:
	uint8_t minV = voltagesAscOrder[0];
	uint8_t cell_minV=cells_VoltageAscOrder[0];
    balanceFlag[cell_minV]=0;

	uint8_t voltage_diff;
	uint8_t diff_threshold=0.01;
	uint8_t cell_index;
	for(int i=1;i<activeCells;i++){
		cell_index=cells_VoltageAscOrder[i];
		voltage_diff = VCell[cell_index] - minV;
		if(voltage_diff>=diff_threshold){
			balanceFlag[cell_index]=1;
		}
	}

	//send commands to flag cells for balancing inside L9963
	char command_BalCell14_7act[19]="00"; // configuration for register 0x10
	char command_BalCell6_1act[19]="00"; //configuration for register 0x11
	for(int i=14;i>=7;i--){
		if(balanceFlag[i]==1)
			strcat(command_BalCell14_7act,"10");
		else
			strcat(command_BalCell14_7act,"00");
	}

	for(int i=6;i>=1;i--){
		if(balanceFlag[i]==1)
			strcat(command_BalCell6_1act,"10");
		else
			strcat(command_BalCell6_1act,"00");
	}

	singleFrameTransaction("1","1","00001","00010000","00",command_BalCell14_7act,false);
	singleFrameTransaction("1","1","00001","00010001","00",command_BalCell6_1act,false);

	//configure balance time window for each cell:

	//start balancing:


}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
