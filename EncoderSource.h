#include "PIDSource.h"

class Encoder;

class EncoderSource : public PIDSource
{
public:
	EncoderSource(Encoder& enc1);
	double PIDGet();
private:
	Encoder& mEnc1;
};
