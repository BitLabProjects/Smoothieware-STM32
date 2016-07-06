#include "AnalogInEx.h"

#include "mbed_assert.h"
#include "analogin_api.h"

#include "wait_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "PeripheralPins.h"

ADC_HandleTypeDef AdcHandle;
DMA_HandleTypeDef DmaHandle;

extern RawSerial pc;

int adc_inited = 0;
uint32_t mDstBufferIndex;
uint32_t* mDstBuffer;
uint32_t mDstBufferLength;
void (*mDataReadyCallback)(uint32_t, uint32_t);

void adc_irq_handler(void);

AnalogInEx::AnalogInEx() {
}

void AnalogInEx::init(PinName pin,
		                  uint32_t* dstBuffer,
					            uint32_t dstBufferLength,
					            void (*dataReadyCallback)(uint32_t, uint32_t)) {
	mDstBuffer = dstBuffer;
	mDstBufferLength = dstBufferLength;
	mDataReadyCallback = dataReadyCallback;

	// Get the peripheral name from the pin and assign it to the object
	mAdc.adc = (ADCName)pinmap_peripheral(pin, PinMap_ADC);
	MBED_ASSERT(mAdc.adc != (ADCName)NC);

	// Get the functions (adc channel) from the pin and assign it to the object
	uint32_t function = pinmap_function(pin, PinMap_ADC);
	MBED_ASSERT(function != (uint32_t)NC);
	mAdc.channel = STM_PIN_CHANNEL(function);

	// Configure GPIO
	pinmap_pinout(pin, PinMap_ADC);

	// Save pin number for the read function
	mAdc.pin = pin;

	// The ADC initialization is done once
	if (adc_inited == 0) {
		adc_inited = 1;

		// Enable ADC clock
		__ADC1_CLK_ENABLE();

		// Configure ADC
		AdcHandle.Instance = (ADC_TypeDef *)(mAdc.adc);
		AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV8;
		AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;
		AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
		AdcHandle.Init.ScanConvMode          = DISABLE;
		AdcHandle.Init.EOCSelection          = DISABLE;
		AdcHandle.Init.ContinuousConvMode    = ENABLE;
		AdcHandle.Init.DMAContinuousRequests = DISABLE;
		AdcHandle.Init.NbrOfConversion       = 1;
		AdcHandle.Init.DiscontinuousConvMode = DISABLE; //???
		AdcHandle.Init.NbrOfDiscConversion   = 0;
		AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
		AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
		HAL_ADC_Init(&AdcHandle);
	}
}

uint32_t AnalogInEx::start() {
    ADC_ChannelConfTypeDef sConfig;

    AdcHandle.Instance = (ADC_TypeDef *)(mAdc.adc);

    // Configure ADC channel
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    sConfig.Offset       = 0;

    switch (mAdc.channel) {
        case 0:
            sConfig.Channel = ADC_CHANNEL_0;
            break;
        case 1:
            sConfig.Channel = ADC_CHANNEL_1;
            break;
        case 2:
            sConfig.Channel = ADC_CHANNEL_2;
            break;
        case 3:
            sConfig.Channel = ADC_CHANNEL_3;
            break;
        case 4:
            sConfig.Channel = ADC_CHANNEL_4;
            break;
        case 5:
            sConfig.Channel = ADC_CHANNEL_5;
            break;
        case 6:
            sConfig.Channel = ADC_CHANNEL_6;
            break;
        case 7:
            sConfig.Channel = ADC_CHANNEL_7;
            break;
        case 8:
            sConfig.Channel = ADC_CHANNEL_8;
            break;
        case 9:
            sConfig.Channel = ADC_CHANNEL_9;
            break;
        case 10:
            sConfig.Channel = ADC_CHANNEL_10;
            break;
        case 11:
            sConfig.Channel = ADC_CHANNEL_11;
            break;
        case 12:
            sConfig.Channel = ADC_CHANNEL_12;
            break;
        case 13:
            sConfig.Channel = ADC_CHANNEL_13;
            break;
        case 14:
            sConfig.Channel = ADC_CHANNEL_14;
            break;
        case 15:
            sConfig.Channel = ADC_CHANNEL_15;
            break;
        default:
            return 0;
    }

    HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

    NVIC_SetVector(ADC_IRQn, (uint32_t)adc_irq_handler);
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

    mDstBufferIndex = 0;
    HAL_ADC_Start_IT(&AdcHandle);
    return 1;

    //Code for conversion with polling
    /*
    HAL_ADC_Start(&AdcHandle);

    // Wait end of conversion and get value
    if (HAL_ADC_PollForConversion(&AdcHandle, 10) == HAL_OK) {
        return (HAL_ADC_GetValue(&AdcHandle));
    } else {
        return 0;
    }
    */
}

void AnalogInEx::stop() {
	AdcHandle.Instance = (ADC_TypeDef *)(mAdc.adc);

	HAL_ADC_Stop_IT(&AdcHandle);
	HAL_NVIC_DisableIRQ(ADC_IRQn);
}

void adc_irq_handler(void) {
  //if (TestFlag(ADC1->SR, ADC_FLAG_OVR))
  //  pc.printf("ADC Overflow\n");
  HAL_ADC_IRQHandler(&AdcHandle);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	mDstBuffer[mDstBufferIndex] = HAL_ADC_GetValue(hadc);
	mDstBufferIndex = (mDstBufferIndex+1) % mDstBufferLength;
	/* TODO: Double buffering with half dst buffer
	if (mDstBufferIndex == (mDstBufferLength >> 1))
		mDataReadyCallback(0, mDstBufferLength >> 1);
	else if (mDstBufferIndex == 0)
		mDataReadyCallback(mDstBufferLength >> 1, mDstBufferLength >> 1);
	*/
	if (mDstBufferIndex == 0)
		mDataReadyCallback(0, mDstBufferLength);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
}
