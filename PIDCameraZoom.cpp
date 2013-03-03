#include "PIDCameraZoom.h"
#include "CameraTarget.h"
#include "PIDController.h"

PIDCameraZoom::PIDCameraZoom(PIDController* controller)
	: m_controller(controller)
{
	mLast = 0;
}

double PIDCameraZoom::PIDGet()
{
	float setpoint = m_controller->GetSetpoint();
	double dist=mLast;
	int unfoundCounter = 0;
	CameraTarget& target = CameraTarget::getInstance();
	if (target.isTargetFound())
	{
		dist = target.getDistance();
		mLast = dist;
		unfoundCounter = 0;
	}
	else
	{
		unfoundCounter++;
	}
	if(unfoundCounter>20)
	{
		dist = setpoint;
	}
	return dist;
}
