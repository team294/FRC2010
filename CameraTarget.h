#ifndef CAMERA_TARGET_
#define CAMERA_TARGET_

#include <taskLib.h>
#include "vxWorks.h" 

#include "ErrorBase.h"
#include "Task.h"

class CameraTarget: public ErrorBase {
public:
	static CameraTarget& getInstance();
	static bool isInstance(void);

	void deleteInstance();
	~CameraTarget();
	
	bool isTargetFound();
	double getDistance();
	double getAngle();
	
	int taskID;
	
private:
	CameraTarget();
	CameraTarget(CameraTarget const&);
	CameraTarget& operator=(CameraTarget const&);
	
	static int s_findTarget();
	int findTarget();
	
	void setCamParams();

	static CameraTarget* m_instance;
	
	Task targetTask;
	
	bool m_found;
	double m_distance;
	double m_angle;
	static SEM_ID m_sem;
};

#endif
