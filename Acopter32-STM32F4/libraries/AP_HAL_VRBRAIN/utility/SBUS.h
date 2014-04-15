#ifndef SBUS_h
#define SBUS_h

#include <AP_HAL.h>
//#include "Arduino.h"

#define SBUS_FAILSAFE_INACTIVE 0
#define SBUS_FAILSAFE_ACTIVE   1
#define SBUS_STARTBYTE         0x0f
#define SBUS_ENDBYTE           0x00


class SBUSClass {
	public:
		SBUSClass(){}
		void begin();
		void begin(bool useTimer);
		int getChannel(int channel);
		int getNormalizedChannel(int channel);
		int getFailsafeStatus();
		int getFrameLoss();
		long getGoodFrames();
		long getLostFrames();
		long getDecoderErrorFrames();
	private:
		void _process();
		AP_HAL::UARTDriver * _serial;
		int _channels[18];
		int _failsafe;
		long _goodFrames;
		long _lostFrames;
		long _decoderErrorFrames;
};

extern SBUSClass SBUS;

#endif
