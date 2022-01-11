#pragma once

#include "Patch.h"
#include "OpenWareLibrary.h"

#include <memory>
#include "wdf.h"

class PreAmpStage : public MultiSignalProcessor
{
public:
  PreAmpStage(const float sampleRate)
    {
        C3 = std::make_unique<WaveDigitalFilter::Capacitor> (0.1e-6, sampleRate);
        C5 = std::make_unique<WaveDigitalFilter::Capacitor> (68.0e-9, sampleRate);
        Vbias.setVoltage (0.0);

        C16 = std::make_unique<WaveDigitalFilter::Capacitor> (1.0e-6, sampleRate);
        Vbias2.setVoltage (0.0);
        
        P1 = std::make_unique<WaveDigitalFilter::WDFParallel> (C5.get(), &R6);
        S1 = std::make_unique<WaveDigitalFilter::WDFSeries> (P1.get(), &Vbias);

        P2 = std::make_unique<WaveDigitalFilter::WDFParallel> (&Vbias2, C16.get());
        S2 = std::make_unique<WaveDigitalFilter::WDFSeries> (P2.get(), &R7);

        P3 = std::make_unique<WaveDigitalFilter::WDFParallel> (S1.get(), S2.get());
        S3 = std::make_unique<WaveDigitalFilter::WDFSeries> (P3.get(), C3.get());
        I1 = std::make_unique<WaveDigitalFilter::PolarityInverter> (S3.get());
        Vin.connectToNode (I1.get());
    }

    void setGain (float gain)
    {
        Vbias.setResistanceValue ((double) gain * 100.0e3);
    }

  inline float processSample(float x){
        Vin.setVoltage ((double) x);

        Vin.incident (I1->reflected());
        auto y = Vbias.voltage() + R6.voltage();
        I1->incident (Vin.reflected());

        return (float) y;
    }

  void process(AudioBuffer& input, AudioBuffer& output){
    FloatArray in = input.getSamples(0);
    FloatArray out = output.getSamples(0);
    FloatArray ff1Current = output.getSamples(1);
    for(size_t i=0; i<in.getSize(); ++i){
      out[i] = processSample(in[i]);
      ff1Current[i] = (float) (1000 * Vbias2.current());
    }
  }

private:
    WaveDigitalFilter::IdealVoltageSource Vin;
    std::unique_ptr<WaveDigitalFilter::Capacitor> C3;
    std::unique_ptr<WaveDigitalFilter::Capacitor> C5;
    WaveDigitalFilter::Resistor R6 { 10000.0 };
    WaveDigitalFilter::ResistiveVoltageSource Vbias;

    WaveDigitalFilter::Resistor R7 { 1500.0 };
    std::unique_ptr<WaveDigitalFilter::Capacitor> C16;
    WaveDigitalFilter::ResistiveVoltageSource Vbias2 { 15000.0 };

    std::unique_ptr<WaveDigitalFilter::PolarityInverter> I1;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S1;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S2;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S3;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S4;
    std::unique_ptr<WaveDigitalFilter::WDFParallel> P1;
    std::unique_ptr<WaveDigitalFilter::WDFParallel> P2;
    std::unique_ptr<WaveDigitalFilter::WDFParallel> P3;
};

class ClippingStage : public SignalProcessor
{
public:
  ClippingStage(const float sampleRate)
    {
        C9  = std::make_unique<WaveDigitalFilter::Capacitor> (1.0e-6, sampleRate);
        C10 = std::make_unique<WaveDigitalFilter::Capacitor> (1.0e-6, sampleRate);
        Vbias.setVoltage (4.5);

        I1 = std::make_unique<WaveDigitalFilter::PolarityInverter> (&Vin);
        S1 = std::make_unique<WaveDigitalFilter::WDFSeries> (I1.get(), C9.get());
        S2 = std::make_unique<WaveDigitalFilter::WDFSeries> (S1.get(), &R13);
        S3 = std::make_unique<WaveDigitalFilter::WDFSeries> (C10.get(), &Vbias);
        P1 = std::make_unique<WaveDigitalFilter::WDFParallel> (S2.get(), S3.get());
        D23.connectToNode (P1.get());
    }

    float process (float x)
    {
        Vin.setVoltage ((double) x + 4.5); // bias

        D23.incident (P1->reflected());
        P1->incident (D23.reflected());
        auto y = C10->current();

        return (float)(1000 *  y);
    }

  using SignalProcessor::process;
private:
    WaveDigitalFilter::ResistiveVoltageSource Vin;
    std::unique_ptr<WaveDigitalFilter::Capacitor> C9;
    WaveDigitalFilter::Resistor R13 { 1000.0 };
    WaveDigitalFilter::DiodePair D23 { 15e-6, 0.02585 };
    std::unique_ptr<WaveDigitalFilter::Capacitor> C10;
    WaveDigitalFilter::ResistiveVoltageSource Vbias { 47000.0 };

    std::unique_ptr<WaveDigitalFilter::PolarityInverter> I1;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S1;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S2;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S3;
    std::unique_ptr<WaveDigitalFilter::WDFParallel> P1;
};


class FeedForward2 : public SignalProcessor
{
public:
  FeedForward2(const float sampleRate)
    {
        C4 = std::make_unique<WaveDigitalFilter::Capacitor> (68e-9, sampleRate);
        C6 = std::make_unique<WaveDigitalFilter::Capacitor> (390e-9, sampleRate);
        C11 = std::make_unique<WaveDigitalFilter::Capacitor> (2.2e-9, sampleRate);
        C12 = std::make_unique<WaveDigitalFilter::Capacitor> (27e-9, sampleRate);
        Vbias.setVoltage (0.0);

        S1 = std::make_unique<WaveDigitalFilter::WDFSeries> (C12.get(), &R18);
        P1 = std::make_unique<WaveDigitalFilter::WDFParallel> (S1.get(), &R17);
        S2 = std::make_unique<WaveDigitalFilter::WDFSeries> (C11.get(), &R15);
        S3 = std::make_unique<WaveDigitalFilter::WDFSeries> (S2.get(), &R16);
        P2 = std::make_unique<WaveDigitalFilter::WDFParallel> (S3.get(), P1.get());
        P3 = std::make_unique<WaveDigitalFilter::WDFParallel> (P2.get(), &RVBot);

        S4 = std::make_unique<WaveDigitalFilter::WDFSeries> (P3.get(), &RVTop);
        S5 = std::make_unique<WaveDigitalFilter::WDFSeries> (C6.get(), &R9);
        P4 = std::make_unique<WaveDigitalFilter::WDFParallel> (S4.get(), S5.get());
        P5 = std::make_unique<WaveDigitalFilter::WDFParallel> (P4.get(), &R8);
        S6 = std::make_unique<WaveDigitalFilter::WDFSeries> (P5.get(), &Vbias);

        P6 = std::make_unique<WaveDigitalFilter::WDFParallel> (&R5, C4.get());
        S7 = std::make_unique<WaveDigitalFilter::WDFSeries> (P6.get(), S6.get());
        I1 = std::make_unique<WaveDigitalFilter::PolarityInverter> (S7.get());
        
        Vin.connectToNode (I1.get());
    }

    void setGain (float gain)
    {
        RVTop.setResistanceValue (std::max (gain * 100e3, 1.0));
        RVBot.setResistanceValue (std::max ((1.0 - gain) * 100e3, 1.0));
    }

    inline float processSample (float x)
    {
        Vin.setVoltage ((double) x);

        Vin.incident (I1->reflected());
        I1->incident (Vin.reflected());
        auto y = R16.current();

        return (float) y;
    }

private:
    WaveDigitalFilter::IdealVoltageSource Vin;
    WaveDigitalFilter::ResistiveVoltageSource Vbias;

    WaveDigitalFilter::Resistor R5 { 5100.0 };
    WaveDigitalFilter::Resistor R8 { 1500.0 };
    WaveDigitalFilter::Resistor R9 { 1000.0 };
    WaveDigitalFilter::Resistor RVTop { 50000.0 };
    WaveDigitalFilter::Resistor RVBot { 50000.0 };
    WaveDigitalFilter::Resistor R15 { 22000.0 };
    WaveDigitalFilter::Resistor R16 { 47000.0 };
    WaveDigitalFilter::Resistor R17 { 27000.0 };
    WaveDigitalFilter::Resistor R18 { 12000.0 };

    std::unique_ptr<WaveDigitalFilter::Capacitor> C4;
    std::unique_ptr<WaveDigitalFilter::Capacitor> C6;
    std::unique_ptr<WaveDigitalFilter::Capacitor> C11;
    std::unique_ptr<WaveDigitalFilter::Capacitor> C12;

    std::unique_ptr<WaveDigitalFilter::WDFSeries> S1;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S2;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S3;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S4;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S5;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S6;
    std::unique_ptr<WaveDigitalFilter::WDFSeries> S7;
    
    std::unique_ptr<WaveDigitalFilter::PolarityInverter> I1;
    std::unique_ptr<WaveDigitalFilter::WDFParallel> P1;
    std::unique_ptr<WaveDigitalFilter::WDFParallel> P2;
    std::unique_ptr<WaveDigitalFilter::WDFParallel> P3;
    std::unique_ptr<WaveDigitalFilter::WDFParallel> P4;
    std::unique_ptr<WaveDigitalFilter::WDFParallel> P5;
    std::unique_ptr<WaveDigitalFilter::WDFParallel> P6;
};

// PreAmpStage preAmpStage;
// AmpStage ampStage;
// ClippingStage clippingStage;
// FF1Current ff1 (preAmpStage);
// FeedForward2 ff2;
// AudioMixer4 mixer;
// SummingAmp summingAmp;

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
  BiquadFilter* inputBuffer;
  BiquadFilter* toneControl;
  BiquadFilter* outputBuffer;

// Gain stage processors
  PreAmpStage* preAmpStage; // outputs two channels: signal and ff1 current * 1000
  BiquadFilter* ampStage;
  ClippingStage* clippingStage;
  FeedForward2* ff2;
  BiquadFilter* summingAmp;
  
  FloatArray buf1;
  FloatArray buf2;
  AudioBuffer* buf3;
public:
  ChowCentaurPatch() : AUDIO_SAMPLE_RATE_EXACT(getSampleRate()) {
    registerParameter(PARAMETER_A, "Gain");
    registerParameter(PARAMETER_B, "Treble");
    registerParameter(PARAMETER_C, "Level");
    inputBuffer = BiquadFilter::create(getSampleRate(), 1);
    toneControl = BiquadFilter::create(getSampleRate(), 1);
    outputBuffer = BiquadFilter::create(getSampleRate(), 1);

    summingAmp = BiquadFilter::create(getSampleRate(), 1);
    ampStage = BiquadFilter::create(getSampleRate(), 1);
    buf1 = FloatArray::create(getBlockSize());
    buf2 = FloatArray::create(getBlockSize());
    buf3 = AudioBuffer::create(2, getBlockSize());

    preAmpStage = new PreAmpStage(getSampleRate());
    clippingStage = new ClippingStage(getSampleRate());
    ff2 = new FeedForward2(getSampleRate());
    
    setInputBuffer(inputBuffer);
    setSummingAmp(summingAmp);
  }
  ~ChowCentaurPatch(){
    BiquadFilter::destroy(inputBuffer);
    BiquadFilter::destroy(toneControl);
    BiquadFilter::destroy(outputBuffer);
    BiquadFilter::destroy(summingAmp);
    BiquadFilter::destroy(ampStage);
    FloatArray::destroy(buf1);
    FloatArray::destroy(buf2);
    AudioBuffer::destroy(buf3);
    delete preAmpStage;
    delete clippingStage;
    delete ff2;
  }
  void processAudio(AudioBuffer &buffer){
    float gain = getParameterValue(PARAMETER_A);
    float treble = getParameterValue(PARAMETER_B);
    float level = getParameterValue(PARAMETER_C);
    setTreble(treble, toneControl);
    setLevel(level, outputBuffer);

    preAmpStage->setGain(gain);
    setAmpStageGain(gain, ampStage);
    ff2->setGain (gain);

    FloatArray in = buffer.getSamples(LEFT_CHANNEL);
    inputBuffer->process(in, in);

    // inputBuffer->preAmpStage->ampStage->clippingStage->mixer
    preAmpStage->process(buffer, *buf3);
    ampStage->process(buf3->getSamples(0), buf1);
    clippingStage->process(buf1, buf1);
    // inputBuffer->ff2->mixer
    ff2->process(in, buf2);
    buf1.add(buf2);
    // ff1->mixer
    FloatArray ff1current = buf3->getSamples(1);
    buf1.add(ff1current);
    // mixer->summingAmp->toneControl
    summingAmp->process(buf1, buf1);
    toneControl->process(buf1, in);
  }

  
    static inline float calcPoleFreq (float a, float b, float c)
    {
        auto radicand = b*b - 4.0f*a*c;
        if (radicand >= 0.0f)
            return 0.0f;

        return sqrt (-radicand) / (2.0f * a);
    }

  void setAmpStageGain(float gain, BiquadFilter* filter)
    {
        // double coefficients[5];
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

        // setCoefficients(0, coefficients);
    }

  void setSummingAmp(BiquadFilter* filter)
    {
        // double coefficients[5];
      float* coefficients = filter->getCoefficients();

        // component values
        constexpr float R20 = (float) 392e3;
        constexpr float C13 = (float) 820e-12;

        // analog coefficients
        const auto b0s = 0.0f;
        const auto b1s = R20;
        const auto a0s = C13 * R20;
        const auto a1s = 1.0f;

        const auto K = 2.0f * AUDIO_SAMPLE_RATE_EXACT;
        const auto a0 = a0s * K + a1s;
        
        /* b0 */ coefficients[0] = ( b0s * K + b1s) / a0;
		/* b1 */ coefficients[1] = (-b0s * K + b1s) / a0;
		/* b2 */ coefficients[2] = 0.0;
		/* a1 */ coefficients[3] = (-a0s * K + a1s) / a0;
		/* a2 */ coefficients[4] = 0.0;

        // setCoefficients(0, coefficients);
    }

  void setLevel(float level, BiquadFilter* filter)
    {
        // double coefficients[5];
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

        // setCoefficients(0, coefficients);
    }

  void setInputBuffer(BiquadFilter* filter)
    {
        // double coefficients[5];
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
        const auto K = 2.0f * AUDIO_SAMPLE_RATE_EXACT;
        const auto a0 = a0s * K + a1s;

        /* b0 */ coefficients[0] = ( b0s * K + b1s) / a0;
		/* b1 */ coefficients[1] = (-b0s * K + b1s) / a0;
		/* b2 */ coefficients[2] = 0.0;
		/* a1 */ coefficients[3] = (-a0s * K + a1s) / a0;
		/* a2 */ coefficients[4] = 0.0;

        // setCoefficients(0, coefficients);
    }

  void setTreble (float treble, BiquadFilter* filter)
    {
        // double coefficients[5];
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

        // setCoefficients(0, coefficients);
    }
};

#if 0
    // static void addParameters (Parameters& params);
    void prepareToPlay (float sampleRate, int samplesPerBlock);
    // void releaseResources();
  void processAudio(AudioBuffer &buffer);

    // void processAudioBlock (AudioBuffer<float>& buffer) override;
    // void processInternalBuffer (AudioBuffer<float>& buffer);

    AudioProcessorEditor* createEditor() override;
    void setStateInformation (const void* data, int sizeInBytes) override;
    void getStateInformation (MemoryBlock& data) override;

private:
    std::atomic<float>* trebleParam = nullptr;
    std::atomic<float>* levelParam = nullptr;
    std::atomic<float>* mlParam = nullptr;
    std::atomic<float>* bypassParam = nullptr;
    std::atomic<float>* monoParam = nullptr;

    BypassProcessor bypass;
    InputBufferProcessor inProc[2];
    ToneFilterProcessor tone[2];
    OutputStageProc outProc[2];

    std::unique_ptr<GainStageProc> gainStageProc;
    GainStageMLProc gainStageMLProc;

    AudioBuffer<float> monoBuffer;
    bool useMonoPrev;

    AudioBuffer<float> fadeBuffer;
    bool useMLPrev = false;

    using StereoIIR = dsp::ProcessorDuplicator<dsp::IIR::Filter<float>, dsp::IIR::Coefficients<float>>;
    StereoIIR dcBlocker;
};


void ChowCentaurPatch::prepareToPlay (float sampleRate, int samplesPerBlock)
{
    gainStageProc = std::make_unique<GainStageProc> (vts, sampleRate);
    gainStageProc->reset (sampleRate, samplesPerBlock);
    gainStageMLProc.reset (sampleRate, samplesPerBlock);

    for (int ch = 0; ch < 2; ++ch)
    {
        inProc[ch].prepare ((float) sampleRate);
        tone[ch].prepare ((float) sampleRate);
        outProc[ch].prepare ((float) sampleRate);
    }

    scope->prepareToPlay (sampleRate, samplesPerBlock);

    useMLPrev = static_cast<bool> (*mlParam);
    fadeBuffer.setSize (getMainBusNumOutputChannels(), samplesPerBlock);

    useMonoPrev = static_cast<bool> (*monoParam);
    monoBuffer.setSize (1, samplesPerBlock);

    // set up DC blockers
    *dcBlocker.state = *dsp::IIR::Coefficients<float>::makeHighPass (sampleRate, 35.0f);
    dsp::ProcessSpec spec { sampleRate, static_cast<uint32> (samplesPerBlock), 2 };
    dcBlocker.prepare (spec);

    bypass.prepare (samplesPerBlock, ! bypass.toBool (bypassParam));
}

void ChowCentaurPatch::processInternalBuffer (AudioBuffer<float>& buffer)
{
    const auto numSamples = buffer.getNumSamples();
    for (int ch = 0; ch < buffer.getNumChannels(); ++ch)
    {
        auto* x = buffer.getWritePointer (ch);

        // Input buffer
        FloatVectorOperations::multiply (x, 0.5f, numSamples);
        inProc[ch].processBlock (x, numSamples);
        FloatVectorOperations::clip (x, x, -4.5f, 4.5f, numSamples); // op amp clipping
    }

    const bool useML = static_cast<bool> (*mlParam);
    if (useML == useMLPrev)
    {
        if (useML) // use rnn
            gainStageMLProc.processBlock (buffer);
        else // use circuit model
            gainStageProc->processBlock (buffer);
    }
    else
    {
        fadeBuffer.makeCopyOf (buffer, true);

        if (useML) // use rnn
        {
            gainStageMLProc.processBlock (buffer);
            gainStageProc->processBlock (fadeBuffer);
        }
        else // use circuit model
        {
            gainStageProc->processBlock (buffer);
            gainStageMLProc.processBlock (fadeBuffer);
        }

        buffer.applyGainRamp (0, numSamples, 0.0f, 1.0f);
        for (int ch = 0; ch < buffer.getNumChannels(); ++ch)
            buffer.addFromWithRamp (ch, 0, fadeBuffer.getReadPointer (ch), numSamples, 1.0f, 0.0f);

        useMLPrev = useML;
    }

    for (int ch = 0; ch < buffer.getNumChannels(); ++ch)
    {
        auto* x = buffer.getWritePointer (ch);

        // tone stage
        tone[ch].setTreble (*trebleParam);
        tone[ch].processBlock (x, numSamples);

        FloatVectorOperations::multiply (x, -1.0f, numSamples); // inverting amplifier
        FloatVectorOperations::clip (x, x, -13.1f, 11.7f, numSamples); // op-amp clipping

        outProc[ch].setLevel (*levelParam);
        outProc[ch].processBlock (x, numSamples);
    }
}

  void ChowCentaurPatch::processAudio(AudioBuffer &buffer);
// void ChowCentaurPatch::processAudioBlock (AudioBuffer<float>& buffer)
{
    // bypass if needed
    if (! bypass.processBlockIn (buffer, ! bypass.toBool (bypassParam)))
    {
        // DC Blocker
        dsp::AudioBlock<float> block (buffer);
        dsp::ProcessContextReplacing<float> context (block);
        dcBlocker.process (context);

        return;
    }

    // process mono or stereo buffer?
    auto numSamples = buffer.getNumSamples();
    auto useMono = static_cast<bool> (monoParam->load());

    if (useMono != useMonoPrev)
    {
        gainStageProc->reset (getSampleRate(), getBlockSize());
        gainStageMLProc.reset (getSampleRate(), getBlockSize());

        for (int ch = 0; ch < 2; ++ch)
        {
            inProc[ch].prepare ((float) getSampleRate());
            tone[ch].prepare ((float) getSampleRate());
            outProc[ch].prepare ((float) getSampleRate());
        }

        useMonoPrev = useMono;
    }

    if (useMono)
    {
        monoBuffer.setSize (1, buffer.getNumSamples(), false, false, true);
        monoBuffer.clear();
        monoBuffer.copyFrom (0, 0, buffer.getReadPointer (0), numSamples);

        for (int ch = 1; ch < buffer.getNumChannels(); ++ch)
            monoBuffer.addFrom (0, 0, buffer.getReadPointer (ch), numSamples);

        monoBuffer.applyGain (1.0f / (float) buffer.getNumChannels());
    }

    auto& processBuffer = useMono ? monoBuffer : buffer;

    // actual DSP processing
    processInternalBuffer (processBuffer);

    // go back from mono to stereo (if needed)
    if (useMono)
    {
        auto processedData = processBuffer.getReadPointer (0);
        for (int ch = 0; ch < buffer.getNumChannels(); ++ch)
            buffer.copyFrom (ch, 0, processedData, numSamples);
    }

    // bypass fade
    bypass.processBlockOut (buffer, ! bypass.toBool (bypassParam));

    // DC Blocker
    dsp::AudioBlock<float> block (buffer);
    dsp::ProcessContextReplacing<float> context (block);
    dcBlocker.process (context);
}
#endif
