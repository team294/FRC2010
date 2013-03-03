#ifndef DUALSPEEDCONTROLLER_H
#define DUALSPEEDCONTROLLER_H

#include "SpeedController.h"

// Control two SpeedControllers like a single one.
class DualSpeedController : public SpeedController
{
public:
	DualSpeedController(SpeedController &controller1, SpeedController &controller2);
	virtual ~DualSpeedController();

	void SetInverted(unsigned int motor, bool isInverted);
	
	void Set(float speed);
	float Get(void);

private:
	SpeedController &m_controller1;
	SpeedController &m_controller2;
	int m_inverted[2];
};

#endif
