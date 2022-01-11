#pragma once

#include "Patch.h"
#include "OpenWareLibrary.h"
#include "PreAmpStage.hpp"
#include "ClippingStage.hpp"
#include "FeedForward2.hpp"

class InputBuffer : public SignalProcessor {
protected:
  BiquadFilter* filter;
public:
  InputBuffer(float sr) {
    filter = BiquadFilter::create(sr);
    float* coefficients = filter->getCoefficients();

    // component values
    constexpr float R1 = 10000.0f;
    constexpr float R2 = 1000000.0f;
    constexpr float C1 = (float) 0.1e-6;

    // analog coefficients
    const auto b0s = C1 * R2;
    const auto b1s = 0.0f;
    const auto a0s = C1 * (R1 + R2);
    const auto a1s = 1.0f;

    // bilinear transform
    const auto K = 2.0f * sr;
    const auto a0 = a0s * K + a1s;

    /* b0 */ coefficients[0] = ( b0s * K + b1s) / a0;
    /* b1 */ coefficients[1] = (-b0s * K + b1s) / a0;
    /* b2 */ coefficients[2] = 0.0;
    /* a1 */ coefficients[3] = (-a0s * K + a1s) / a0;
    /* a2 */ coefficients[4] = 0.0;
  }
  ~InputBuffer() {
    BiquadFilter::destroy(filter);
  }
  void process(FloatArray input, FloatArray output){
    filter->process(input, output);
  }
};

class SummingAmp : public SignalProcessor {
protected:
  BiquadFilter* filter;
public:
  SummingAmp(float sr) {
    filter = BiquadFilter::create(sr);
    float* coefficients = filter->getCoefficients();

    // component values
    constexpr float R20 = (float) 392e3;
    constexpr float C13 = (float) 820e-12;

    // analog coefficients
    const auto b0s = 0.0f;
    const auto b1s = R20;
    const auto a0s = C13 * R20;
    const auto a1s = 1.0f;

    const auto K = 2.0f * sr;
    const auto a0 = a0s * K + a1s;
        
    /* b0 */ coefficients[0] = ( b0s * K + b1s) / a0;
    /* b1 */ coefficients[1] = (-b0s * K + b1s) / a0;
    /* b2 */ coefficients[2] = 0.0;
    /* a1 */ coefficients[3] = (-a0s * K + a1s) / a0;
    /* a2 */ coefficients[4] = 0.0;
  }
  ~SummingAmp() {
    BiquadFilter::destroy(filter);
  }
  void process(FloatArray input, FloatArray output){
    filter->process(input, output);
  }
};

class AmpStage : public SignalProcessor {
protected:
  const float AUDIO_SAMPLE_RATE_EXACT;
  BiquadFilter* filter;
public:
  AmpStage(float sr) : AUDIO_SAMPLE_RATE_EXACT(sr) {
    filter = BiquadFilter::create(sr);
  }
    static inline float calcPoleFreq (float a, float b, float c)
    {
        auto radicand = b*b - 4.0f*a*c;
        if (radicand >= 0.0f)
            return 0.0f;

        return sqrtf (-radicand) / (2.0f * a);
    }
  void setGain(float gain){
    float* coefficients = filter->getCoefficients();

    // component values
    const float curR10b = (1.0f - gain) * 100000.0f + 2000.0f;
    constexpr float C7 = (float) 82e-9;
    constexpr float C8 = (float) 390e-12;
    constexpr float R11 = (float) 15e3;
    constexpr float R12 = (float) 422e3;

    // analog coeffs
    const float a0s = C7 * C8 * curR10b * R11 * R12;
    const float a1s = C7 * curR10b * R11 + C8 * R12 * (curR10b + R11);
    const float a2s = curR10b + R11;
    const float b0s = a0s;
    const float b1s = C7 * R11 * R12 + a1s;
    const float b2s = R12 + a2s;

    // frequency warping
    const float wc = calcPoleFreq (a0s, a1s, a2s);
    const auto K = wc == 0.0f ? 2.0f * AUDIO_SAMPLE_RATE_EXACT
      : wc / tan (wc / (2.0f * AUDIO_SAMPLE_RATE_EXACT));
    const auto KSq = K * K;

    // bilinear transform
    const float a0 = a0s * KSq + a1s * K + a2s;
    /* b0 */ coefficients[0] = (b0s * KSq + b1s * K + b2s) / a0;
    /* b1 */ coefficients[1] = 2.0f * (b2s - b0s * KSq) / a0;
    /* b2 */ coefficients[2] = (b0s * KSq - b1s * K + b2s) / a0;
    /* a1 */ coefficients[3] = 2.0f * (a2s - a0s * KSq) / a0;
    /* a2 */ coefficients[4] = (a0s * KSq - a1s * K + a2s) / a0;
  }
  ~AmpStage() {
    BiquadFilter::destroy(filter);
  }
  void process(FloatArray input, FloatArray output){
    filter->process(input, output);
  }
};

class OutputBuffer : public SignalProcessor {
protected:
  const float AUDIO_SAMPLE_RATE_EXACT;
  BiquadFilter* filter;
public:
  OutputBuffer(float sr) : AUDIO_SAMPLE_RATE_EXACT(sr) {
    filter = BiquadFilter::create(sr);
  }
  void setLevel(float level) {
    float* coefficients = filter->getCoefficients();

    const float R1 = 560.0f + (1.0f - level) * 10000.0f;
    const float R2 = level * 10000.0f + 1.0f;
    constexpr float C1 = (float) 4.7e-6;

    // analog coefficients
    const auto b0s = C1 * R2;
    const auto b1s = 0.0f;
    const auto a0s = C1 * (R1 + R2);
    const auto a1s = 1.0f;

    // bilinear transform
    const auto K = 2.0f * AUDIO_SAMPLE_RATE_EXACT;
    const auto a0 = a0s * K + a1s;

    /* b0 */ coefficients[0] = ( b0s * K + b1s) / a0;
    /* b1 */ coefficients[1] = (-b0s * K + b1s) / a0;
    /* b2 */ coefficients[2] = 0.0;
    /* a1 */ coefficients[3] = (-a0s * K + a1s) / a0;
    /* a2 */ coefficients[4] = 0.0;
  }
  ~OutputBuffer() {
    BiquadFilter::destroy(filter);
  }
  void process(FloatArray input, FloatArray output){
    filter->process(input, output);
  }
};

class ToneControl : public SignalProcessor {
protected:
  const float AUDIO_SAMPLE_RATE_EXACT;
  BiquadFilter* filter;
public:
  ToneControl(float sr) : AUDIO_SAMPLE_RATE_EXACT(sr) {
    filter = BiquadFilter::create(sr);
  }
  void setTreble (float treble) {
    float* coefficients = filter->getCoefficients();

    constexpr float Rpot = (float) 10e3;
    constexpr float C = (float) 3.9e-9;
    constexpr float G1 = 1.0f / (float) 100e3;
    const float G2 = 1.0f / ((float) 1.8e3 + (1.0f-treble)*Rpot);
    const float G3 = 1.0f / ((float) 4.7e3 + treble*Rpot);
    constexpr float G4 = 1.0f / (float) 100e3;

    constexpr float wc = G1 / C; // frequency to match
    const auto K = wc / tan (wc / (2.0f * AUDIO_SAMPLE_RATE_EXACT)); // frequency warp to match transition freq

    // analog coefficients
    const auto b0s = C * (G1 + G2);
    const auto b1s = G1 * (G2 + G3);
    const auto a0s = C * (G3 - G4);
    const auto a1s = -G4 * (G2 + G3);
        
    // bilinear transform
    const auto a0 = a0s * K + a1s;
    const float bU0 = ( b0s * K + b1s) / a0;
    const float bU1 = (-b0s * K + b1s) / a0;
    const float aU1 = (-a0s * K + a1s) / a0;

    // flip pole inside unit circle to ensure stability
    /* b0 */ coefficients[0] = bU0 / aU1;
    /* b1 */ coefficients[1] = bU1 / aU1;
    /* b2 */ coefficients[2] = 0.0;
    /* a1 */ coefficients[3] = 1.0 / aU1;
    /* a2 */ coefficients[4] = 0.0;
  }
  ~ToneControl() {
    BiquadFilter::destroy(filter);
  }
  void process(FloatArray input, FloatArray output){
    filter->process(input, output);
  }
};

// AudioConnection patchpreAmp (inputBuffer, 0, preAmpStage, 0);
// AudioConnection patchAmp (preAmpStage, 0, ampStage, 0);
// AudioConnection patchClipping (ampStage, 0, clippingStage, 0);
// AudioConnection patchFF2 (inputBuffer, 0, ff2, 0);
// AudioConnection sum0 (ff1, 0, mixer, 0);
// AudioConnection sum1 (clippingStage, 0, mixer, 1);
// AudioConnection sum2 (ff2, 0, mixer, 2);
// AudioConnection patchSummingAmp (mixer, 0, summingAmp, 0);
// AudioConnection patchTone (summingAmp, 0, toneControl, 0);

class ChowCentaurPatch : public Patch {
protected:
  const float AUDIO_SAMPLE_RATE_EXACT;
  // common processors
  InputBuffer* inputBuffer;
  ToneControl* toneControl;
  OutputBuffer* outputBuffer;

  // Gain stage processors
  PreAmpStage* preAmpStage; // outputs two channels: signal and ff1 current * 1000
  AmpStage* ampStage;
  ClippingStage* clippingStage;
  FeedForward2* ff2;
  SummingAmp* summingAmp;
  
  FloatArray buf1;
  FloatArray buf2;
  AudioBuffer* buf3;
public:
  ChowCentaurPatch() : AUDIO_SAMPLE_RATE_EXACT(getSampleRate()) {
    registerParameter(PARAMETER_A, "Gain");
    registerParameter(PARAMETER_B, "Treble");
    registerParameter(PARAMETER_C, "Level");
    setParameterValue(PARAMETER_A, 0.5);
    setParameterValue(PARAMETER_B, 0.5);
    setParameterValue(PARAMETER_C, 0.5);
    inputBuffer = new InputBuffer(getSampleRate());
    toneControl = new ToneControl(getSampleRate());
    outputBuffer = new OutputBuffer(getSampleRate());
    summingAmp = new SummingAmp(getSampleRate());
    ampStage = new AmpStage(getSampleRate());
    preAmpStage = new PreAmpStage(getSampleRate());
    clippingStage = new ClippingStage(getSampleRate());
    ff2 = new FeedForward2(getSampleRate());
    buf1 = FloatArray::create(getBlockSize());
    buf2 = FloatArray::create(getBlockSize());
    buf3 = AudioBuffer::create(2, getBlockSize());    
  }
  ~ChowCentaurPatch(){
    delete inputBuffer;
    delete toneControl;
    delete outputBuffer;
    delete summingAmp;
    delete ampStage;
    delete preAmpStage;
    delete clippingStage;
    delete ff2;
    FloatArray::destroy(buf1);
    FloatArray::destroy(buf2);
    AudioBuffer::destroy(buf3);
  }
  void processAudio(AudioBuffer &buffer){
    float gain = getParameterValue(PARAMETER_A);
    float treble = getParameterValue(PARAMETER_B);
    float level = getParameterValue(PARAMETER_C);
    toneControl->setTreble(treble);
    outputBuffer->setLevel(level);

    preAmpStage->setGain(gain);
    ampStage->setGain(gain);
    ff2->setGain(gain);

    FloatArray in = buffer.getSamples(LEFT_CHANNEL);
    inputBuffer->process(in, in);

    // inputBuffer > preAmpStage>ampStage > clippingStage > mixer
    preAmpStage->process(buffer, *buf3);
    ampStage->process(buf3->getSamples(0), buf1);
    clippingStage->process(buf1, buf1);
    // inputBuffer > ff2 > mixer
    ff2->process(in, buf2);
    buf1.add(buf2);
    // ff1 > mixer
    FloatArray ff1current = buf3->getSamples(1);
    buf1.add(ff1current);
    // mixer > summingAmp > toneControl
    summingAmp->process(buf1, buf1);
    toneControl->process(buf1, in);
    // buffer.getSamples(RIGHT_CHANNEL).copyFrom(in);
  }
};
