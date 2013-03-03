#include "PIDCamera.h"
#include "CameraTarget.h"
#define MINIMUM_SCORE 0.01

PIDCamera::PIDCamera()
{	
	mLast = 0;
}

double PIDCamera::PIDGet()
{
	double horizontalAngle;
	
	horizontalAngle=mLast;
	
	CameraTarget& target = CameraTarget::getInstance();
	if (target.isTargetFound())
	{
		horizontalAngle = target.getAngle();
		mLast = horizontalAngle;
	}
	return horizontalAngle;
}
