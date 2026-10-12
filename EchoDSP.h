#ifndef PROYECTO_ECHO_DSP_H
#define PROYECTO_ECHO_DSP_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Este núcleo no depende de Arduino ni del controlador I2S. Recibe y entrega
// una muestra real por llamada; el programa principal maneja el hardware.
namespace dsp {

constexpr uint32_t kSampleRate = 16000;
constexpr size_t kMaxDelaySamples = 8000;  // 500 ms si Fs = 16 kHz.
constexpr size_t kDefaultDelaySamples = 4000;  // Ejemplo: 250 ms.
constexpr float kDefaultWet = 0.5f;
constexpr float kDefaultGain = 0.5f;

enum class Mode {
    Loopback,  // y[n] = g*x[n]
    Delay,     // y[n] = g*x[n-D]
    Echo       // y[n] = g*(x[n] + a*x[n-D])
};

class Echo {
public:
    Echo()
        : writeIndex_(0), delaySamples_(kDefaultDelaySamples),
          wet_(kDefaultWet), gain_(kDefaultGain), mode_(Mode::Loopback) {
        reset();
    }

    // Reinicia la memoria, pero conserva los parámetros elegidos.
    // El reposo inicial significa x[n] = 0 para n < 0.
    void reset() {
        memset(delayLine_, 0, sizeof(delayLine_));
        writeIndex_ = 0;
    }

    // Cada llamada representa exactamente un instante n de la señal.
    float process(float x) {
        const size_t readIndex =
            (writeIndex_ + kMaxDelaySamples - delaySamples_) % kMaxDelaySamples;

        // Leer ANTES de escribir también permite D = capacidad del buffer.
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
        // También convierte NaN a 0 para no contaminar todas las muestras.
        if (!(value >= 0.0f)) return 0.0f;
        if (value > 1.0f) return 1.0f;
        return value;
    }

    // No es static: cada instancia conserva su propia historia de muestras.
    // Ocupa 8000 * sizeof(float): 32 000 bytes con float de 32 bits en ESP32.
    float delayLine_[kMaxDelaySamples];
    size_t writeIndex_;
    size_t delaySamples_;
    float wet_;
    float gain_;
    Mode mode_;
};

// No hay recorte ni cuantización dentro del núcleo. Con parámetros fijos,
// las ecuaciones anteriores representan sistemas lineales e invariantes en
// el tiempo (LTI). El programa de audio debe limitar/convertir la salida al
// formato PCM; el recorte y la cuantización no son operaciones lineales.
// Los valores predeterminados son ejemplos provisionales para desarrollar
// el código: deben contrastarse con los cálculos del grupo.

}  // namespace dsp

#endif
