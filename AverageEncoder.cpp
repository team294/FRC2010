#include "AverageEncoder.h"
#include "Encoder.h"

AverageEncoder::AverageEncoder(Encoder& enc1, Encoder& enc2)
	: mEnc1(enc1), mEnc2(enc2)
{
}

double AverageEncoder::PIDGet()
{
	return double(mEnc1.Get() + mEnc2.Get())/2;
}
