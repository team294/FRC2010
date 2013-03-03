#include "PIDDrive.h"
#include "RobotDrive.h"

PIDDrive::PIDDrive(RobotDrive& robot)
	: mRobot(robot)
{
}

void PIDDrive::PIDWrite(float output)
{
	mRobot.ArcadeDrive(output, 0.0, false);
}
