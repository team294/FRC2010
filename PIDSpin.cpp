#include "PIDSpin.h"
#include "RobotDrive.h"

PIDSpin::PIDSpin(RobotDrive& robot)
	: mRobot(robot)
{
}

void PIDSpin::PIDWrite(float output)
{
	mRobot.TankDrive(-output,output);
}
