#ifndef PROYECTO_ECHO_DSP_H
#define PROYECTO_ECHO_DSP_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>


namespace dsp {

constexpr uint32_t kSampleRate = 16000;
constexpr size_t kMaxDelaySamples = 8000;  
constexpr size_t kDefaultDelaySamples = 4000;  
constexpr float kDefaultWet = 0.5f;
constexpr float kDefaultGain = 0.5f;

enum class Mode {
    Loopback, 
    Delay,    
    Echo       
};

class Echo {
public:
    Echo()
        : writeIndex_(0), delaySamples_(kDefaultDelaySamples),
          wet_(kDefaultWet), gain_(kDefaultGain), mode_(Mode::Loopback) {
        reset();
    }

    
    void reset() {
        memset(delayLine_, 0, sizeof(delayLine_));
        writeIndex_ = 0;
    }

   
    float process(float x) {
        const size_t readIndex =
            (writeIndex_ + kMaxDelaySamples - delaySamples_) % kMaxDelaySamples;

        
        const float delayed = delayLine_[readIndex];
        delayLine_[writeIndex_] = x;
        writeIndex_ = (writeIndex_ + 1) % kMaxDelaySamples;

        switch (mode_) {
            case Mode::Loopback:
                return gain_ * x;
            case Mode::Delay:
                return gain_ * delayed;
            case Mode::Echo:
                return gain_ * (x + wet_ * delayed);
        }
        return gain_ * x;
    }

    void setGain(float gain) { gain_ = clampUnit(gain); }
    void setWet(float wet) { wet_ = clampUnit(wet); }

    void setDelaySamples(size_t delaySamples) {
        if (delaySamples < 1) {
            delaySamples_ = 1;
        } else if (delaySamples > kMaxDelaySamples) {
            delaySamples_ = kMaxDelaySamples;
        } else {
            delaySamples_ = delaySamples;
        }
    }

    void setMode(Mode mode) { mode_ = mode; }

    float gain() const { return gain_; }
    float wet() const { return wet_; }
    size_t delaySamples() const { return delaySamples_; }
    Mode mode() const { return mode_; }

private:
    static float clampUnit(float value) {

        if (!(value >= 0.0f)) return 0.0f;
        if (value > 1.0f) return 1.0f;
        return value;
    }

    float delayLine_[kMaxDelaySamples];
    size_t writeIndex_;
    size_t delaySamples_;
    float wet_;
    float gain_;
    Mode mode_;
};


} 

#endif
