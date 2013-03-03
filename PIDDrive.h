#include "PIDOutput.h"

class RobotDrive;

class PIDDrive : public PIDOutput
{
	public:
		PIDDrive(RobotDrive& robot);
		void PIDWrite(float output);
	private:
		RobotDrive& mRobot;
};
