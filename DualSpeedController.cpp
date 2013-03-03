#include "DualSpeedController.h"

DualSpeedController::DualSpeedController(SpeedController &controller1, SpeedController &controller2)
	: m_controller1(controller1)
	, m_controller2(controller2)
{
	m_inverted[0] = 1;
	m_inverted[1] = 1;
}

DualSpeedController::~DualSpeedController()
{
}

void DualSpeedController::SetInverted(unsigned int motor, bool isInverted)
{
	if (motor > 1)
		return;
	m_inverted[motor] = isInverted ? -1 : 1;
}
	
void DualSpeedController::Set(float speed)
{
	m_controller1.Set(speed * m_inverted[0]);
	m_controller2.Set(speed * m_inverted[1]);
}

float DualSpeedController::Get(void)
{
	return m_controller1.Get() * m_inverted[0];
}
