#include "PIDSource.h"
#include "PIDController.h"

class PIDController;

class PIDCameraZoom : public PIDSource
{
public:
	PIDCameraZoom(PIDController* controller);
	double PIDGet();

private:
	double mLast;
	PIDController* m_controller;
};
