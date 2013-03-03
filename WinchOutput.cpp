#include "WinchOutput.h"
#include "SpeedController.h"
#include "Encoder.h"
#include "DigitalInput.h"

WinchOutput::WinchOutput(SpeedController &winchMotor, Encoder &enc, DigitalInput& backLimitSwitch)
	: m_winchMotor(winchMotor)
	, m_enc(enc)
	, m_backLimit(backLimitSwitch)
{
}

WinchOutput::~WinchOutput()
{
}

void WinchOutput::Set(float speed)
{
	if (speed < 0 && m_backLimit.Get() == 0)
		m_winchMotor.Set(0);
	else
		m_winchMotor.Set(speed);
}

void WinchOutput::PIDWrite(float output)
{
	Set(output);
}
