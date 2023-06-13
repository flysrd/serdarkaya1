#include "main.h"
//#include "usb_device.h"
#include "usbd_cdc_if.h"
//#include "adc.h"
//#include "tim.h"

// Pin Definitions
// Pin to output the character signal
#define CHARACTER_OUTPUT_PIN GPIO_PIN_0
#define CHARACTER_OUTPUT_PORT GPIOA

// Pin to read the output value
#define VALUE_INPUT_PIN GPIO_PIN_12
#define VALUE_INPUT_PIN GPIO_PIN_13
#define VALUE_INPUT_PORT GPIOA

// ADC Configuration
#define ADC_CHANNEL ADC_CHANNEL_1

// PWM Configuration
#define PWM_TIM TIM2
#define PWM_CHANNEL TIM_CHANNEL_1

// Handcrafted Sensor Configuration
#define HANDCRAFTED_SENSOR_PIN GPIO_PIN_2
#define HANDCRAFTED_SENSOR_PORT GPIOA

// Phototransistor Configuration
#define PHOTOTRANSISTOR_PIN GPIO_PIN_3
#define PHOTOTRANSISTOR_PORT GPIOA

// USB Communication Variables
#define USB_TX_BUFFER_SIZE 64
#define USB_RX_BUFFER_SIZE 64
uint8_t usbTxBuffer[USB_TX_BUFFER_SIZE];
uint8_t usbRxBuffer[USB_RX_BUFFER_SIZE];

// PWM and Duty Cycle Parameters
uint32_t pwmFrequency = 10000; // PWM frequency in Hz
uint32_t dutyCycle = 50;      // Duty cycle in percentage

// Handcrafted Sensor Parameters
uint32_t sensorParam1 = 100;  // Example configurable parameter 1
uint32_t sensorParam2 = 200;  // Example configurable parameter 2

// Phototransistor Parameters
uint32_t flickerThreshold = 80;  // Flicker detection threshold
uint32_t flickerPeriod = 16;     // Flicker detection period in milliseconds

// Function prototypes
void SystemClock_Config(void);
void USB_Transmit(uint8_t *data, uint32_t length);
void ProcessUSBCommand(uint8_t *data, uint32_t length);
void UpdatePWM(void);
void ReadHandcraftedSensor(void);
void DetectLampFlickering(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();

  while (1)
  {
    // Check for USB data reception
    if (CDC_Receive_FS(usbRxBuffer, USB_RX_BUFFER_SIZE) == USBD_OK)
    {
      ProcessUSBCommand(usbRxBuffer, USB_RX_BUFFER_SIZE);
    }

    // Update PWM settings
    UpdatePWM();

    // Read handcrafted sensor
    ReadHandcraftedSensor();

    // Detect lamp flickering
    DetectLampFlickering();
  }
}

void USB_Transmit(uint8_t *data, uint32_t length)
{
  CDC_Transmit_FS(data, length);
}

void ProcessUSBCommand(uint8_t *data, uint32_t length)
{
  // Process USB commands and update corresponding variables/configurations
  // You can define custom command formats and protocols based on your requirements
  // Here, we assume the received command is a single character



void UpdatePWM(void)
{
  // Calculate the period and pulse width based on the PWM frequency and duty cycle
  uint32_t period = HAL_RCC_GetHCLKFreq() / pwmFrequency;
  uint32_t pulseWidth = (period * dutyCycle) / 100;

  // Update the PWM configuration
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulseWidth;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  // Set the PWM period
  __HAL_TIM_SET_AUTORELOAD(&htim2, period);

  // Start the PWM generation
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void ReadHandcraftedSensor(void)
{
  // Read the analog value from the handcrafted sensor
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
  uint32_t sensorValue = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);

  // Process the sensor value (if needed)

  // Send the sensor value via USB
  snprintf((char *)usbTxBuffer, USB_TX_BUFFER_SIZE, "Sensor Value: %lu\r\n", sensorValue);
  USB_Transmit(usbTxBuffer, strlen((char *)usbTxBuffer));
}

void DetectLampFlickering(void)
{
  // Read the analog value from the phototransistor
  GPIO_PinState phototransistorState = HAL_GPIO_ReadPin(PHOTOTRANSISTOR_PORT, PHOTOTRANSISTOR_PIN);

  // Check if the phototransistor state is flickering (assuming 60Hz flickering frequency)
  static uint32_t prevMillis = 0;
  uint32_t currentMillis = HAL_GetTick();

  if ((currentMillis - prevMillis) >= flickerPeriod)
  {
    if (phototransistorState == GPIO_PIN_RESET)
    {
      // Lamp is flickering
      // Perform necessary actions

      // Send flickering status via USB
      snprintf((char *)usbTxBuffer, USB_TX_BUFFER_SIZE, "Lamp Flickering Detected!\r\n");
      USB_Transmit(usbTxBuffer, strlen((char *)usbTxBuffer));
    }

    prevMillis = currentMillis;
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }
  /** Macro to configure the PLL clock source
   */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the SYSCFG APB clock
   */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
}
