/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Adc.h"
#include "libs/nuts_bolts.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "libs/Median.h"

#include "StreamOutputPool.h"

#include <cstring>
#include <algorithm>

using namespace std;
#include <vector>
// This is an interface to the mbed.org ADC library you can find in libs/ADC/adc.h
// TODO : Having the same name is confusing, should change that

Adc *Adc::instance;

#define SAMPLE_BUFFER_LENGTH 16
uint32_t buffer[SAMPLE_BUFFER_LENGTH];

static void callback(uint32_t offset, uint32_t length)
{
    int channel = 0;
    for(uint32_t i=offset; i<offset+length; i++) {
        Adc::instance->new_sample(channel, buffer[i]<<4);
    }
}

Adc::Adc()//: input(PC_2)
{
    instance = this;
    
    // ADC sample rate need to be fast enough to be able to read the enabled channels within the thermistor poll time
    // even though ther maybe 32 samples we only need one new one within the polling time
    
    //TODO Configure the sampling time to be 1KHz
    const uint32_t sample_rate= 1000; // 1KHz sample rate

    input.init(PC_2, buffer, SAMPLE_BUFFER_LENGTH, callback);
    input.start();
}

// Enables ADC on a given pin
void Adc::enable_pin(Pin *pin)
{
#warning STM32
/* FIXME STM32
    PinName pin_name = this->_pin_to_pinname(pin);
    int channel = adc->_pin_to_channel(pin_name);
    memset(sample_buffers[channel], 0, sizeof(sample_buffers[0]));

    this->adc->burst(1);
    this->adc->setup(pin_name, 1);
    this->adc->interrupt_state(pin_name, 1);
*/
}

// Keeps the last 8 values for each channel
// This is called in an ISR, so sample_buffers needs to be accessed atomically
void Adc::new_sample(int chan, uint32_t value)
{
    // Shuffle down and add new value to the end
    if(chan < num_channels) {
        memmove(&sample_buffers[chan][0], &sample_buffers[chan][1], sizeof(sample_buffers[0]) - sizeof(sample_buffers[0][0]));
        sample_buffers[chan][num_samples - 1] = (value >> 4) & 0xFFF; // the 12 bit ADC reading
    }
}

//#define USE_MEDIAN_FILTER
// Read the filtered value ( burst mode ) on a given pin
unsigned int Adc::read(Pin *pin)
{
    //unsigned int value = (unsigned int) this->input.read_u16();
    //return value >> 4;    

#warning STM32
/* FIXME STM32
    PinName p = this->_pin_to_pinname(pin);
    int channel = adc->_pin_to_channel(p);
*/
    int channel = 0;
    uint16_t median_buffer[num_samples];
    // needs atomic access TODO maybe be able to use std::atomic here or some lockless mutex
    __disable_irq();
    memcpy(median_buffer, sample_buffers[channel], sizeof(median_buffer));
    __enable_irq();

#ifdef USE_MEDIAN_FILTER
    // returns the median value of the last 8 samples
    return median_buffer[quick_median(median_buffer, num_samples)];

#elif defined(OVERSAMPLE)
    // Oversample to get 2 extra bits of resolution
    // weed out top and bottom worst values then oversample the rest
    // put into a 4 element moving average and return the average of the last 4 oversampled readings
    static uint16_t ave_buf[num_channels][4] =  { {0} };
    std::sort(median_buffer, median_buffer + num_samples);
    uint32_t sum = 0;
    for (int i = num_samples / 4; i < (num_samples - (num_samples / 4)); ++i) {
        sum += median_buffer[i];
    }
    // this slows down the rate of change a little bit
    ave_buf[channel][3]= ave_buf[channel][2];
    ave_buf[channel][2]= ave_buf[channel][1];
    ave_buf[channel][1]= ave_buf[channel][0];
    ave_buf[channel][0]= sum >> OVERSAMPLE;
    return roundf((ave_buf[channel][0]+ave_buf[channel][1]+ave_buf[channel][2]+ave_buf[channel][3])/4.0F);

#else
    // sort the 8 readings and return the average of the middle 4
    std::sort(median_buffer, median_buffer + num_samples);
    int sum = 0;
    for (int i = num_samples / 4; i < (num_samples - (num_samples / 4)); ++i) {
        sum += median_buffer[i];
    }
    return sum / (num_samples / 2);

#endif
}

// Convert a smoothie Pin into a mBed Pin
PinName Adc::_pin_to_pinname(Pin *pin)
{
#warning STM32
/* FIXME stm32 
    if( pin->port == LPC_GPIO0 && pin->pin == 23 ) {
        return p15;
    } else if( pin->port == LPC_GPIO0 && pin->pin == 24 ) {
        return p16;
    } else if( pin->port == LPC_GPIO0 && pin->pin == 25 ) {
        return p17;
    } else if( pin->port == LPC_GPIO0 && pin->pin == 26 ) {
        return p18;
    } else if( pin->port == LPC_GPIO1 && pin->pin == 30 ) {
        return p19;
    } else if( pin->port == LPC_GPIO1 && pin->pin == 31 ) {
        return p20;
    } else {
        //TODO: Error
        return NC;
    }
    */
    return NC;
}

