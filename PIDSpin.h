#include "PIDOutput.h"

class RobotDrive;

class PIDSpin : public PIDOutput
{
	public:
		PIDSpin(RobotDrive& robot);
		void PIDWrite(float output);
	private:
		RobotDrive& mRobot;
};
