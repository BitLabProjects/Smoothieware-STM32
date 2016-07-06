#ifndef ANALOGINEX_H
#define ANALOGINEX_H

#include "mbed.h"

class AnalogInEx {

public:
	AnalogInEx();

	void init(PinName pin,
			      uint32_t* dstBuffer,
			      uint32_t dstBufferLength,
			      void (*dataReadyCallback)(uint32_t, uint32_t));
	uint32_t start();
  void stop();

protected:
  analogin_t mAdc;
};

#endif
