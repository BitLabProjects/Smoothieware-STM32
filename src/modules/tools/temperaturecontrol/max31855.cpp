/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include <math.h>
#include "libs/Pin.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

#include "max31855.h"

#include "MRI_Hooks.h"

#define chip_select_checksum CHECKSUM("chip_select_pin")
#define spi_channel_checksum CHECKSUM("spi_channel")

Max31855::Max31855() :
    spi(nullptr)
{
}

Max31855::~Max31855()
{
    delete spi;
}

// Get configuration from the config file
void Max31855::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum)
{
    // Chip select
    this->spi_cs_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, chip_select_checksum)->by_default("A.13")->as_string());
    this->spi_cs_pin.set(true);
    this->spi_cs_pin.as_output();
    
    // select which SPI channel to use
    int spi_channel = THEKERNEL->config->value(module_checksum, name_checksum, spi_channel_checksum)->by_default(1)->as_number();
    PinName miso = PA_6;
    PinName mosi = PA_7;
    PinName sclk = PA_5;
    if(spi_channel == 1) {
        // Channel 1
        mosi=PA_7; miso=PA_6; sclk=PA_5;
    } else if(spi_channel == 2) {
        // Channel 2
        mosi=PB_15; miso=PB_14; sclk=PB_13;
    }else if(spi_channel == 3) {
        // Channel 3
        mosi=PB_5; miso=PB_4; sclk=PB_3;
    }

    delete spi;
    spi = new mbed::SPI(mosi, miso, sclk);

    // Spi settings: 1MHz (default), 16 bits, mode 0 (default)
    spi->format(16);
}

float Max31855::get_temperature()
{
	// Return an average of the last readings
    if (readings.size() >= readings.capacity()) {
        readings.delete_tail();
    }

	float temp = read_temp();

	// Discard occasional errors...
	if(!isinf(temp))
	{
		readings.push_back(temp);
	}

	if(readings.size()==0) return infinityf();

	float sum = 0;
    for (int i=0; i<readings.size(); i++)
        sum += *readings.get_ref(i);

	return sum / readings.size();
}

float Max31855::read_temp()
{
    this->spi_cs_pin.set(false);
    wait_us(1); // Must wait for first bit valid

    // Read 16 bits (writing something as well is required by the api)
    uint16_t data = spi->write(0);
	//  Read next 16 bits (diagnostics)
//	uint16_t data2 = spi->write(0);

    this->spi_cs_pin.set(true);
    
    float temperature;

    //Process temp
    if (data & 0x0001)
    {
        // Error flag.
        temperature = infinityf();
        // Todo: Interpret data2 for more diagnostics.
    }
    else
    {
        data = data >> 2;
        temperature = (data & 0x1FFF) / 4.f;

        if (data & 0x2000)
        {
            data = ~data;
            temperature = ((data & 0x1FFF) + 1) / -4.f;
        }
    }
    return temperature; 
}
