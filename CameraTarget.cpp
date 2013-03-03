#include <string.h>

#include "CameraTarget.h"
#include "Vision/AxisCamera.h"
#include "Vision/HSLImage.h"
#include "Target.h"
#include "Timer.h"

CameraTarget* CameraTarget::m_instance= NULL;
SEM_ID CameraTarget::m_sem = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);

#define MINIMUM_SCORE 0.01

bool CameraTarget::isTargetFound() {
	bool found;
	semTake(m_sem, -1);
	{
		found = m_found;
	}
	semGive(m_sem);
	return found;
}

double CameraTarget::getDistance() {
	double dist;
	semTake(m_sem, -1);
	{
		dist = m_distance;
	}
	semGive(m_sem);
	return dist;
}

double CameraTarget::getAngle() {
	double angle;
	semTake(m_sem, -1);
	{
		angle = m_angle;
	}
	semGive(m_sem);
	return angle;
}

/**
 * Static interface that will cause an instantiation if necessary.
 */
int CameraTarget::s_findTarget() {
	return CameraTarget::getInstance().findTarget();
}

/**
 * Task spawned by AxisCamera constructor to receive images from cam
 * If setNewImageSem has been called, this function does a semGive on each new image
 * Images can be accessed by calling getImage()
 */
int CameraTarget::findTarget() {
	//Infinite loop, task deletion handled by taskDeleteHook
	// Socket cleanup handled by destructor
	AxisCamera& camera = AxisCamera::GetInstance();
	ColorImage *image = new ColorImage(IMAQ_IMAGE_HSL);

	while (1) {
		if (camera.IsFreshImage())
		{
			if (!camera.GetImage(image->GetImaqImage()))
			{
				semTake(m_sem, -1);
				m_found = false;
				m_angle = 0;
				m_distance = 0;
				semGive(m_sem);
				Wait(0);
				continue;
			}
			vector<Target> targets = Target::FindCircularTargets(image);
			
			if (targets.size() == 0 || targets[0].m_score < MINIMUM_SCORE)
			{
				semTake(m_sem, -1);
				m_found = false;
				m_angle = 0;
				m_distance = 0;
				semGive(m_sem);
			} else {
				// angle
				double angle = targets[0].GetHorizontalAngle();
				
				// distance
				double dist = targets[0].m_majorRadius;
				dist = dist * 10000;
				//dist = (dist-4868.88)/-22.16; old
				dist =  (dist-3964.6148)/-14.76;
				
				semTake(m_sem, -1);
				m_found = true;
				m_angle = angle;
				m_distance = dist;
				semGive(m_sem);
			}
		}
		Wait(0.01);
	}
	delete image;
}

void CameraTarget::setCamParams()
{
	AxisCamera& camera = AxisCamera::GetInstance();
	camera.WriteResolution(AxisCameraParams::kResolution_160x120);
	camera.WriteBrightness(160);
}

/**
 * Get a pointer to the AxisCamera object, if the object does not exist, create it
 * @return reference to AxisCamera object
 */
CameraTarget& CameraTarget::getInstance() {
	if (NULL == m_instance) {
		m_instance = new CameraTarget();
		m_instance->setCamParams();
		m_instance->targetTask.Start();
	}
	return *m_instance;
}

/**
 * Does the global instance exist?
 */
bool CameraTarget::isInstance(void) {
	return m_instance != NULL;
}

void CameraTarget::deleteInstance() {
	delete m_instance;
}

CameraTarget::CameraTarget() :
	targetTask("targetTask", (FUNCPTR)s_findTarget, 105),
	m_found(false)
	{
	}

CameraTarget::~CameraTarget() {
	targetTask.Stop();
	semDelete(m_sem);
	m_instance = NULL;
}
