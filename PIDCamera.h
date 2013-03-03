#include "PIDSource.h"

class AxisCamera;

class PIDCamera : public PIDSource
{
public:
	PIDCamera();
	double PIDGet();
private:
	double mLast;
};
