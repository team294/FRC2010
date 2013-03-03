#ifndef WINCHOUTPUT_H
#define WINCHOUTPUT_H

#include "PIDOutput.h"

class DigitalInput;
class Encoder;
class SpeedController;

// PID output for winch control with safety limits.
class WinchOutput : public PIDOutput
{
public:
	WinchOutput(SpeedController& winchMotor, Encoder& enc, DigitalInput& backLimitSwitch);
	virtual ~WinchOutput();

	void Set(float speed);
	void PIDWrite(float output);

private:
	SpeedController& m_winchMotor;
	Encoder& m_enc;
	DigitalInput& m_backLimit;
};

#endif
