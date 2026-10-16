#ifndef PROYECTO_AUDIO_DIAGNOSTICS_H
#define PROYECTO_AUDIO_DIAGNOSTICS_H

#include <stdint.h>
#include <stddef.h>
#include <float.h>
#include <cmath>
#include "EchoDSP.h"

// Herramientas sin Arduino, I2S ni asignacion dinamica de memoria.
// No acceden al hardware ni imprimen datos: el sketch organiza esas tareas.
namespace audio_diag {

constexpr uint32_t kSampleRate = 16000;
constexpr size_t kCaptureCapacity = 8192;  // 0.512 s y 32768 bytes de float.
constexpr size_t kStage1DelaySamples = 4000;  // 250 ms a 16000 Hz.
constexpr float kPositiveFullScale24 = 8388607.0f / 8388608.0f;
constexpr double kPi = 3.14159265358979323846;

inline bool finite(float value) {
    return value >= -FLT_MAX && value <= FLT_MAX;
}

// I2S: los 24 bits validos del INMP441 ocupan bits 31..8 del int32 recibido.
// Los 8 bits inferiores se descartan: SD queda en alta impedancia tras el LSB.
// La extension de signo se hace por resta, sin desplazar enteros negativos
// ni convertir un uint32 fuera del rango de int32 a int32.
inline int32_t decodeRaw24(int32_t raw32) {
    const uint32_t code24 = static_cast<uint32_t>(raw32) >> 8;
    return (code24 & 0x00800000u)
        ? static_cast<int32_t>(code24) - 16777216
        : static_cast<int32_t>(code24);
}

inline float decodeNormalized24(int32_t raw32) {
    return static_cast<float>(decodeRaw24(raw32)) / 8388608.0f;
}

inline float decodeInmp441(int32_t raw32) { return decodeNormalized24(raw32); }

struct PcmCounters {
    uint32_t positiveClips;
    uint32_t negativeClips;
    uint32_t nonFinite;
    PcmCounters() : positiveClips(0), negativeClips(0), nonFinite(0) {}
};

// Convierte a PCM32 con redondeo al entero mas cercano; los empates se alejan
// de cero. +1 no cabe en int32; -1 si cabe. NaN/inf se convierten a silencio.
// El nucleo DSP sigue sin saturacion; esta conversion pertenece a la salida.
inline int32_t encodePcm32(float sample, PcmCounters* counters = NULL) {
    if (!finite(sample)) {
        if (counters) ++counters->nonFinite;
        return 0;
    }
    if (sample >= 1.0f) {
        if (counters) ++counters->positiveClips;
        return INT32_MAX;
    }
    if (sample <= -1.0f) {
        if (counters && sample < -1.0f) ++counters->negativeClips;
        return INT32_MIN;
    }
    const double scaled = static_cast<double>(sample) * 2147483648.0;
    const int64_t rounded = static_cast<int64_t>(scaled + (scaled >= 0 ? 0.5 : -0.5));
    // Las comprobaciones anteriores bastan para float IEEE de 32 bits;
    // mantener estas defensas hace explicito el contrato de la conversion.
    if (rounded > INT32_MAX) {
        if (counters) ++counters->positiveClips;
        return INT32_MAX;
    }
    if (rounded < INT32_MIN) {
        if (counters) ++counters->negativeClips;
        return INT32_MIN;
    }
    return static_cast<int32_t>(rounded);
}

// Adaptador para el contador de recortes existente en el sketch. Para contar
// tambien NaN/inf, usar encodePcm32 con un PcmCounters persistente.
inline int32_t toPcm32(float sample, uint32_t& clipCount) {
    PcmCounters counters;
    const int32_t pcm = encodePcm32(sample, &counters);
    clipCount += counters.positiveClips + counters.negativeClips;
    return pcm;
}

struct StatsSnapshot {
    uint64_t samples;         // Solo muestras finitas.
    uint32_t nonFinite;
    uint32_t positiveRail;    // Codigos en el extremo positivo de PCM24.
    uint32_t negativeRail;    // Codigos en el extremo negativo de PCM24.
    uint32_t nearFullScale;   // |x| >= 0.98; indicador, no prueba de recorte.
    uint32_t outOfRange;
    double mean;             // Componente DC media.
    double rms;              // Incluye DC.
    double rmsAc;            // RMS despues de restar la media.
    float minimum;
    float maximum;
    float peak;
};

class Stats {
public:
    Stats() { clear(); }

    void clear() {
        count_ = 0;
        nonFinite_ = positiveRail_ = negativeRail_ = nearFullScale_ = outOfRange_ = 0;
        sum_ = sumSquares_ = 0;
        minimum_ = maximum_ = peak_ = 0;
    }
    void reset() { clear(); }
    uint64_t count() const { return count_; }
    float peak() const { return peak_; }
    double mean() const { return count_ ? sum_ / static_cast<double>(count_) : 0; }
    double rms() const { return count_ ? std::sqrt(sumSquares_ / count_) : 0; }

    // Para diagnosticar el microfono, agregar ANTES de MIC_INPUT_GAIN.
    void addRaw24(int32_t raw32) { add(decodeNormalized24(raw32)); }

    void add(float sample) {
        if (!finite(sample)) { ++nonFinite_; return; }
        if (sample >= kPositiveFullScale24) ++positiveRail_;
        if (sample <= -1.0f) ++negativeRail_;
        if (std::fabs(sample) >= 0.98f) ++nearFullScale_;
        if (sample > kPositiveFullScale24 || sample < -1.0f) ++outOfRange_;
        if (count_ == 0) minimum_ = maximum_ = sample;
        if (sample < minimum_) minimum_ = sample;
        if (sample > maximum_) maximum_ = sample;
        const float magnitude = std::fabs(sample);
        if (magnitude > peak_) peak_ = magnitude;
        const double x = sample;
        sum_ += x;
        sumSquares_ += x * x;
        ++count_;
    }

    StatsSnapshot snapshot() const {
        StatsSnapshot out = {};
        out.samples = count_;
        out.nonFinite = nonFinite_;
        out.positiveRail = positiveRail_;
        out.negativeRail = negativeRail_;
        out.nearFullScale = nearFullScale_;
        out.outOfRange = outOfRange_;
        out.minimum = minimum_;
        out.maximum = maximum_;
        out.peak = peak_;
        if (count_ != 0) {
            out.mean = sum_ / static_cast<double>(count_);
            const double power = sumSquares_ / static_cast<double>(count_);
            out.rms = std::sqrt(power);
            const double variance = power - out.mean * out.mean;
            out.rmsAc = std::sqrt(variance > 0 ? variance : 0);
        }
        return out;
    }

private:
    uint64_t count_;
    uint32_t nonFinite_, positiveRail_, negativeRail_, nearFullScale_, outOfRange_;
    double sum_, sumSquares_;
    float minimum_, maximum_, peak_;
};

// Una instancia debe ser global/estatica en ESP32: su buffer ocupa 32 kB.
// Un solo productor escribe. Detener su escritura antes de leer/exportar.
class Capture {
public:
    Capture() : count_(0), target_(0), active_(false) {}

    size_t begin(size_t requested = kCaptureCapacity) {
        target_ = requested > kCaptureCapacity ? kCaptureCapacity : requested;
        count_ = 0;
        active_ = target_ != 0;
        return target_;
    }
    void reset() { begin(); }

    bool push(float sample) {
        if (!active_) return false;
        samples_[count_++] = sample;
        if (count_ == target_) active_ = false;
        return true;
    }

    void stop() { active_ = false; }
    bool active() const { return active_; }
    bool complete() const { return target_ != 0 && count_ == target_; }
    size_t size() const { return count_; }
    size_t target() const { return target_; }
    const float* data() const { return samples_; }
    const float& operator[](size_t index) const { return samples_[index]; }

private:
    float samples_[kCaptureCapacity];
    size_t count_, target_;
    bool active_;
};

// Vista sin copiar ni filtrar. Uso experimental: demostrar aliasing al
// submuestrear. NO sustituye un decimador con filtro antialias de produccion.
class RawDecimationView {
public:
    RawDecimationView(const float* input, size_t count, size_t factor = 2,
                      uint32_t inputRate = kSampleRate)
        : input_(input), count_(count), factor_(factor), inputRate_(inputRate) {}

    bool valid() const {
        return factor_ != 0 && inputRate_ != 0 && (input_ != NULL || count_ == 0);
    }
    size_t size() const {
        return valid() ? count_ / factor_ + (count_ % factor_ != 0) : 0;
    }
    double sampleRate() const {
        return valid() ? static_cast<double>(inputRate_) / factor_ : 0;
    }
    bool get(size_t n, size_t& sourceIndex, double& time, float& value) const {
        if (n >= size()) return false;
        sourceIndex = n * factor_;
        time = static_cast<double>(sourceIndex) / inputRate_;
        value = input_[sourceIndex];
        return true;
    }

private:
    const float* input_;
    size_t count_, factor_;
    uint32_t inputRate_;
};

// Acumulador de fase de 32 bits: no reiniciarlo en cada bloque de audio.
// La frecuencia es aproximada al incremento DDS mas cercano.
class ToneDDS {
public:
    ToneDDS(float frequency = 1000.0f, float amplitude = 0.02f,
            uint32_t sampleRate = kSampleRate)
        : phase_(0), increment_(0), amplitude_(0) {
        setFrequency(frequency, sampleRate);
        setAmplitude(amplitude);
    }

    bool setFrequency(float frequency, uint32_t sampleRate = kSampleRate) {
        if (!finite(frequency) || sampleRate == 0 || frequency < 0 ||
            static_cast<double>(frequency) > 0.5 * sampleRate) return false;
        increment_ = static_cast<uint32_t>(static_cast<double>(frequency) *
                                          4294967296.0 / sampleRate + 0.5);
        return true;  // Conserva la fase anterior.
    }

    void setAmplitude(float amplitude) {
        amplitude_ = !finite(amplitude) || amplitude < 0 ? 0 :
                     amplitude > 1 ? 1 : amplitude;
    }
    void resetPhase() { phase_ = 0; }
    float next() {
        const double angle = static_cast<double>(phase_) * (2 * kPi / 4294967296.0);
        const float sample = amplitude_ * static_cast<float>(std::sin(angle));
        phase_ += increment_;  // El desbordamiento unsigned esta definido.
        return sample;
    }

private:
    uint32_t phase_, increment_;
    float amplitude_;
};
typedef ToneDDS Tone;

struct ErrorMetrics {
    uint32_t samples;
    uint32_t failures;
    uint32_t nonFinite;
    float maxAbs;
    double sumSquared;
    ErrorMetrics() : samples(0), failures(0), nonFinite(0), maxAbs(0), sumSquared(0) {}

    void add(float actual, float expected, float absTolerance = 1e-6f,
             float relTolerance = 1e-6f) {
        if (!finite(actual) || !finite(expected)) { ++nonFinite; ++failures; return; }
        const float error = std::fabs(actual - expected);
        if (error > maxAbs) maxAbs = error;
        if (error > absTolerance + relTolerance * std::fabs(expected)) ++failures;
        sumSquared += static_cast<double>(error) * error;
        ++samples;
    }
    double rms() const { return samples ? std::sqrt(sumSquared / samples) : 0; }
    bool passed() const { return samples != 0 && failures == 0 && nonFinite == 0; }
};

enum class DigitalTestKind { Impulse, Additivity, Homogeneity };
struct DigitalRow {
    DigitalTestKind kind;
    size_t n;
    float input;
    float output;
    float reference;
    float residual;
};
typedef void (*DigitalRowSink)(const DigitalRow& row, void* context);

struct DigitalTestConfig {
    size_t samples;
    size_t delaySamples;
    float gain, wet, scale;
    DigitalTestConfig() : samples(kCaptureCapacity), delaySamples(kStage1DelaySamples),
                          gain(0.5f), wet(0.5f), scale(0.3f) {}
};
struct DigitalTestResult {
    bool valid;
    ErrorMetrics impulse, additivity, homogeneity;
    DigitalTestResult() : valid(false) {}
    bool passed() const {
        return valid && impulse.passed() && additivity.passed() && homogeneity.passed();
    }
};

inline float testInput(size_t n, unsigned which) {
    const double frequency = which == 1 ? 440.0 : 730.0;
    return static_cast<float>(0.1 * std::sin(2 * kPi * frequency *
                                           static_cast<double>(n) / kSampleRate));
}

inline void emitRow(DigitalRowSink sink, void* context, DigitalTestKind kind,
                    size_t n, float input, float output, float reference) {
    if (!sink) return;
    const DigitalRow row = {kind, n, input, output, reference, output - reference};
    sink(row, context);
}

// Pruebas sobre el nucleo float real, sin PCM, microfono ni amplificador.
// Requiere un workspace de N floats proporcionado por el llamador.
// NO ejecutar mientras otra tarea usa processor o workspace. El estado previo
// del retardo se pierde; se restauran parametros y se termina con memoria cero.
inline DigitalTestResult runDigitalTests(dsp::Echo& processor, float* workspace,
    size_t workspaceSamples, const DigitalTestConfig& config = DigitalTestConfig(),
    DigitalRowSink sink = NULL, void* context = NULL) {
    DigitalTestResult result;
    if (workspace == NULL || config.samples == 0 ||
        config.samples > workspaceSamples || config.delaySamples < 1 ||
        config.delaySamples > dsp::kMaxDelaySamples || config.samples <= config.delaySamples ||
        !finite(config.gain) || config.gain < 0 || config.gain > 1 ||
        !finite(config.wet) || config.wet < 0 || config.wet > 1 || !finite(config.scale)) {
        return result;
    }
    const dsp::Mode savedMode = processor.mode();
    const size_t savedDelay = processor.delaySamples();
    const float savedGain = processor.gain(), savedWet = processor.wet();
    processor.setMode(dsp::Mode::Echo);
    processor.setDelaySamples(config.delaySamples);
    processor.setGain(config.gain);
    processor.setWet(config.wet);
    processor.reset();
    for (size_t n = 0; n < config.samples; ++n) {
        const float x = n == 0 ? 1.0f : 0.0f;
        const float y = processor.process(x);
        const float reference = n == 0 ? config.gain :
            n == config.delaySamples ? config.gain * config.wet : 0;
        result.impulse.add(y, reference);
        emitRow(sink, context, DigitalTestKind::Impulse, n, x, y, reference);
    }

    processor.reset();
    for (size_t n = 0; n < config.samples; ++n)
        workspace[n] = processor.process(testInput(n, 1));
    processor.reset();
    for (size_t n = 0; n < config.samples; ++n)
        workspace[n] += processor.process(testInput(n, 2));
    processor.reset();
    for (size_t n = 0; n < config.samples; ++n) {
        const float x = testInput(n, 1) + testInput(n, 2);
        const float y = processor.process(x);
        result.additivity.add(y, workspace[n]);
        emitRow(sink, context, DigitalTestKind::Additivity, n, x, y, workspace[n]);
    }

    processor.reset();
    for (size_t n = 0; n < config.samples; ++n)
        workspace[n] = processor.process(testInput(n, 1));
    processor.reset();
    for (size_t n = 0; n < config.samples; ++n) {
        const float x = config.scale * testInput(n, 1);
        const float y = processor.process(x);
        const float reference = config.scale * workspace[n];
        result.homogeneity.add(y, reference);
        emitRow(sink, context, DigitalTestKind::Homogeneity, n, x, y, reference);
    }

    processor.setMode(savedMode);
    processor.setDelaySamples(savedDelay);
    processor.setGain(savedGain);
    processor.setWet(savedWet);
    processor.reset();
    result.valid = true;
    return result;
}

}  // namespace audio_diag

#endif
