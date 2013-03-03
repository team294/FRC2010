#include "EncoderSource.h"
#include "Encoder.h"

EncoderSource::EncoderSource(Encoder& enc1)
	: mEnc1(enc1)
{
}

double EncoderSource::PIDGet()
{
	return (double)mEnc1.Get();
}
