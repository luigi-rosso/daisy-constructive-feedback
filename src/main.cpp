
#include "daisy_pod.h"
#include <cmath>
#define Q_DONT_USE_THREADS
#include <q/support/decibel.hpp>
#include <q/pitch/dual_pitch_detector.hpp>
#include <q/fx/biquad.hpp>
#include <q/fx/envelope.hpp>

using namespace daisy;
using namespace cycfi::q;
static DaisyPod pod;

constexpr auto delayRead = 0.0f;
constexpr auto sps = 48000;
constexpr auto dt = 1.0f / sps;

float phi = 0.0f;
float trackedFrequency = 0.0f;
float delayMix = 0.0f;

dual_pitch_detector detector(frequency(50), frequency(1200), sps, -20_dB);
delay feedback(duration(1), sps);
peaking lastFilter(10.0f, frequency(300), sps, 0.1);
auto env = fast_rms_envelope_follower(duration(0.05), sps);
pd_preprocessor::config cfg;
pd_preprocessor pp{cfg, frequency(50), frequency(1200), sps};

void AudioCallback(float* in, float* out, size_t size)
{
	auto buff_size = size / 2;

	float outl, outr, inl, inr;
	for (size_t i = 0; i < size; i += 2)
	{

		inl = in[i];
		inr = in[i + 1];

		float volume = float(env(inl));

		if (abs(volume) > 0.1f)
		{
			float cleaned = pp(inl);

			if (detector(cleaned) && detector.get_frequency() > 0.0f)
			{
				trackedFrequency = detector.get_frequency();
			}
		}

		float feedbackFrequency = trackedFrequency * 2.0f;
		phi += 2.0f * 3.141592653f * feedbackFrequency * dt;

		float rampSpeed = sps;
		if (abs(volume) < 0.4f)
		{
			delayMix += (1.0f - delayMix) / rampSpeed;
		}
		else
		{
			delayMix = (0.0f - delayMix) / rampSpeed;
		}

		// Hack to allow changing frequency of the filter
		peaking filter(10.0f, frequency(feedbackFrequency), sps, 0.1);
		filter.x1 = lastFilter.x1;
		filter.x2 = lastFilter.x2;
		filter.y1 = lastFilter.y1;
		filter.y2 = lastFilter.y2;
		lastFilter = filter;

		float feed = feedback(feedbackFrequency);
		float value = sin(phi);
		feedback.push(std::max(
		    -1.0f, std::min(1.0f, filter(inl + value) + feed * delayMix)));

		out[i] = inl + feed * delayMix;
		out[i + 1] = out[i];
	}
}

int main(void)
{
	pod.Init();
	pod.seed.StartLog(true);

	pod.seed.SetAudioBlockSize(512);

	pod.StartAdc();
	pod.StartAudio(AudioCallback);

	while (1)
	{
	}
}
