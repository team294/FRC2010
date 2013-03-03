#include "PIDSource.h"

class Encoder;

class AverageEncoder : public PIDSource
{
public:
	AverageEncoder(Encoder& enc1, Encoder& enc2);
	double PIDGet();
private:
	Encoder& mEnc1;
	Encoder& mEnc2;
};
