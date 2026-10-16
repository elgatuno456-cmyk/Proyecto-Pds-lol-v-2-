#ifndef PROYECTO_MEDICIONES_APP_H
#define PROYECTO_MEDICIONES_APP_H

#include <Arduino.h>
#include <math.h>
#include "driver/i2s_std.h"
#include "esp_err.h"
#include "AudioDiagnostics.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Selecciona una placa ESP32-S3. Requiere Arduino-ESP32 3.x."
#endif

namespace mediciones {
constexpr int PIN_BCLK = 4, PIN_WS = 5, PIN_MIC_SD = 10, PIN_AMP_DIN = 7;
constexpr size_t BLOCK_FRAMES = 128;
constexpr size_t CAPTURE_N = 8192;
constexpr size_t TEST_TAIL = 1024;
constexpr size_t WORKSPACE_N = dsp::kMaxDelaySamples + TEST_TAIL;

struct Parameters {
  float gain = 0.05f;
  float wet = 0.5f;
  size_t delay = dsp::kDefaultDelaySamples;
  unsigned slot = 0;                 // L/R del INMP441 a GND: izquierdo.
};
struct TraceContext {
  int previous = -1;
  size_t rows = 0;
  size_t total = 0;
};
Parameters parameters;
i2s_chan_handle_t rxChannel = nullptr;
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t rxOverflows = 0;
int32_t rxBuffer[2 * BLOCK_FRAMES];
audio_diag::Capture recording;       // 32 kB; nunca en la pila.
dsp::Echo testEffect;                // Mismo nucleo del programa de audio.
float workspace[WORKSPACE_N];        // Memoria reutilizada entre ensayos.
char command[48];
size_t commandLength = 0;
bool commandOverflow = false;

bool IRAM_ATTR onRxOverflow(i2s_chan_handle_t, i2s_event_data_t *, void *) {
  portENTER_CRITICAL_ISR(&rxMux);
  ++rxOverflows;
  portEXIT_CRITICAL_ISR(&rxMux);
  return false;
}

uint32_t overflowCount() {
  portENTER_CRITICAL(&rxMux);
  const uint32_t value = rxOverflows;
  portEXIT_CRITICAL(&rxMux);
  return value;
}

void printHelp() {
  Serial.println("MEDICIONES: el parlante queda en silencio; no hay loopback.");
  Serial.println("c: captura real 8192 muestras (0.512 s); guardar bloque CSV completo");
  Serial.println("h: impulso CSV | t: resumen de pruebas DSP | v: pruebas DSP y trazas CSV");
  Serial.println("g 0.05: ganancia del modelo (0..0.5) | a 0.5: alpha (0..1)");
  Serial.println("d 250: retardo ms (1..500) | s 0: slot izquierdo | s 1: derecho | ?: ayuda");
  Serial.println("Para escuchar audio, volver a cargar esp32_etapa1.");
}

bool readBlock() {
  size_t received = 0;
  return i2s_channel_read(rxChannel, rxBuffer, sizeof(rxBuffer), &received, 100)
         == ESP_OK && received == sizeof(rxBuffer);
}

void captureMicrophone() {
  Serial.println("Captura: reproduce el tono externo antes de enviar c; ahora se vacia la cola RX.");
  // Se descartan mas bloques que los seis DMA para no exportar datos viejos.
  for (unsigned i = 0; i < 8; ++i) {
    if (!readBlock()) {
      Serial.println("ERROR: lectura incompleta antes de capturar; no se genero una captura valida.");
      return;
    }
  }
  const uint32_t dropsAtStart = overflowCount();
  uint32_t errors = 0;
  audio_diag::Stats stats;
  recording.begin(CAPTURE_N);
  while (recording.active()) {
    if (!readBlock()) {
      ++errors;
      recording.stop();              // No rellenar huecos con muestras inventadas.
      break;
    }
    for (size_t i = 0; i < BLOCK_FRAMES && recording.active(); ++i) {
      const float x = audio_diag::decodeInmp441(rxBuffer[2*i + parameters.slot]);
      stats.add(x);                  // Captura antes de ganancia, eco o PCM de salida.
      recording.push(x);
    }
  }
  const uint32_t drops = overflowCount() - dropsAtStart;
  const bool valid = recording.complete() && errors == 0 && drops == 0;
  const audio_diag::StatsSnapshot s = stats.snapshot();
  Serial.printf("BEGIN_CAPTURE fs=%lu n=%u origin=mic slot=%u valid=%u rx_drop=%lu io_errors=%lu\n",
      (unsigned long)dsp::kSampleRate, (unsigned)recording.size(), parameters.slot,
      unsigned(valid), (unsigned long)drops, (unsigned long)errors);
  Serial.println("n,x");
  for (size_t n = 0; n < recording.size(); ++n)
    Serial.printf("%u,%.9g\n", (unsigned)n, recording[n]);
  Serial.printf("END_CAPTURE n=%u\n", (unsigned)recording.size());
  Serial.printf("CAPTURE_STATS peak=%.6g mean=%.6g rms=%.6g min=%.6g max=%.6g rails=%lu nearFS=%lu\n",
      s.peak, s.mean, s.rms, s.minimum, s.maximum,
      (unsigned long)(s.positiveRail + s.negativeRail), (unsigned long)s.nearFullScale);
  Serial.println(valid ? "Captura completa, sin perdidas RX registradas durante la adquisicion."
                       : "CAPTURA INVALIDA: repetir antes de sacar conclusiones.");
  Serial.println("Los desbordamientos RX fuera de la adquisicion son esperables al imprimir o esperar comandos.");
}

void configureTestEffect() {
  testEffect.setMode(dsp::Mode::Echo);
  testEffect.setGain(parameters.gain);
  testEffect.setWet(parameters.wet);
  testEffect.setDelaySamples(parameters.delay);
  testEffect.reset();
}

void beginBlock(const char *kind, size_t total, const char *trace) {
  Serial.printf("BEGIN_%s fs=%lu D=%u g=%.9g alpha=%.9g n_total=%u origin=firmware_synthetic trace=%s f1=440 f2=730 scale=0.3 k=0.3\n",
      kind, (unsigned long)dsp::kSampleRate, (unsigned)parameters.delay,
      parameters.gain, parameters.wet, (unsigned)total, trace);
}

const char *kindName(int kind) {
  return kind == 0 ? "IMPULSE" : kind == 1 ? "ADDITIVITY" : "HOMOGENEITY";
}

void finishTrace(TraceContext &context) {
  if (context.previous >= 0)
    Serial.printf("END_%s rows=%u\n", kindName(context.previous), (unsigned)context.rows);
}

void emitDigitalRow(const audio_diag::DigitalRow &row, void *opaque) {
  TraceContext &context = *static_cast<TraceContext *>(opaque);
  const int kind = row.kind == audio_diag::DigitalTestKind::Impulse ? 0 :
      row.kind == audio_diag::DigitalTestKind::Additivity ? 1 : 2;
  if (kind != context.previous) {
    finishTrace(context);
    context.previous = kind;
    context.rows = 0;
    beginBlock(kindName(kind), context.total, kind == 0 ? "all" : "startup_and_tail");
    Serial.println(kind == 0 ? "n,h" : kind == 1 ?
        "n,x1,x2,xsum,actual,reference,error" : "n,x1,kx1,actual,reference,error");
  }
  // El ensayo procesa todas las muestras. Exportar dos ventanas acorta la espera.
  if (kind != 0 && row.n >= 64 && row.n < parameters.delay) return;
  if (kind == 0) Serial.printf("%u,%.9g\n", (unsigned)row.n, row.output);
  else if (kind == 1) {
    Serial.printf("%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", (unsigned)row.n,
        audio_diag::testInput(row.n, 1), audio_diag::testInput(row.n, 2),
        row.input, row.output, row.reference, row.residual);
  } else {
    Serial.printf("%u,%.9g,%.9g,%.9g,%.9g,%.9g\n", (unsigned)row.n,
        audio_diag::testInput(row.n, 1), row.input, row.output, row.reference, row.residual);
  }
  ++context.rows;
}

bool meaningfulParameters() {
  if (parameters.gain > 0 && parameters.wet > 0) return true;
  Serial.println("Para este ensayo del eco usa g>0 y a>0; por ejemplo g 0.05 y a 0.5.");
  return false;
}

void printImpulse() {
  if (!meaningfulParameters()) return;
  configureTestEffect();
  const size_t total = parameters.delay + 257;
  beginBlock("IMPULSE", total, "all");
  Serial.println("n,h");
  for (size_t n = 0; n < total; ++n)
    Serial.printf("%u,%.9g\n", (unsigned)n, testEffect.process(n == 0 ? 1.0f : 0.0f));
  Serial.printf("END_IMPULSE rows=%u\n", (unsigned)total);
  testEffect.reset();
}

void printMetric(const char *name, const audio_diag::ErrorMetrics &metric) {
  Serial.printf("%s: %s max_error=%.9g rms_error=%.9g muestras=%lu fallos=%lu\n",
      name, metric.passed() ? "PASS" : "FAIL", metric.maxAbs, metric.rms(),
      (unsigned long)metric.samples, (unsigned long)metric.failures);
}

void runTests(bool exportRows) {
  if (!meaningfulParameters()) return;
  audio_diag::DigitalTestConfig config;
  config.samples = parameters.delay + TEST_TAIL;
  config.delaySamples = parameters.delay;
  config.gain = parameters.gain;
  config.wet = parameters.wet;
  config.scale = 0.3f;
  TraceContext context;
  context.total = config.samples;
  const audio_diag::DigitalTestResult result = audio_diag::runDigitalTests(
      testEffect, workspace, WORKSPACE_N, config,
      exportRows ? emitDigitalRow : nullptr, exportRows ? &context : nullptr);
  if (exportRows) finishTrace(context);
  Serial.println("PRUEBAS DSP: entradas generadas en ESP32; no prueban la cadena acustica.");
  Serial.printf("Fs=%lu D=%u g=%.6g alpha=%.6g N=%u; tonos440/730Hz; k=0.3\n",
      (unsigned long)dsp::kSampleRate, (unsigned)parameters.delay, parameters.gain,
      parameters.wet, (unsigned)config.samples);
  Serial.println("Tolerancia: 1e-6 absoluto + 1e-6 * abs(referencia), antes de PCM/recorte.");
  if (!result.valid) { Serial.println("FAIL: parametros o workspace invalidos."); return; }
  printMetric("Impulso", result.impulse);
  printMetric("Aditividad", result.additivity);
  printMetric("Homogeneidad", result.homogeneity);
}

void handleCommand(const char *line) {
  float value = 0;
  const bool hasValue = sscanf(line + 1, "%f", &value) == 1 && isfinite(value);
  switch (line[0]) {
    case '?': printHelp(); return;
    case 'c': captureMicrophone(); return;
    case 'h': printImpulse(); return;
    case 't': runTests(false); return;
    case 'v': runTests(true); return;
    case 'g': if (hasValue && value >= 0 && value <= 0.5f) { parameters.gain=value; break; } return;
    case 'a': if (hasValue && value >= 0 && value <= 1) { parameters.wet=value; break; } return;
    case 'd': if (hasValue && value >= 1 && value <= 500) {
        parameters.delay=size_t(lroundf(value*dsp::kSampleRate/1000.0f)); break;
      } return;
    case 's': if (hasValue && (value == 0 || value == 1)) { parameters.slot=unsigned(value); break; } return;
    default: Serial.println("Comando desconocido. Escribe ?."); return;
  }
  Serial.printf("Modelo: g=%.6g alpha=%.6g D=%u slot=%u\n",
      parameters.gain, parameters.wet, (unsigned)parameters.delay, parameters.slot);
}

void begin() {
  Serial.begin(115200);
  delay(1000);
  // Relojes I2S van a ambos modulos. DIN=0 mantiene datos de audio de salida nulos.
  pinMode(PIN_AMP_DIN, OUTPUT);
  digitalWrite(PIN_AMP_DIN, LOW);
  i2s_chan_config_t channel = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  channel.dma_desc_num=6;
  channel.dma_frame_num=BLOCK_FRAMES;
  ESP_ERROR_CHECK(i2s_new_channel(&channel, nullptr, &rxChannel));
  i2s_std_config_t config = {};
  config.clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(dsp::kSampleRate);
  config.slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  config.gpio_cfg.mclk=I2S_GPIO_UNUSED;
  config.gpio_cfg.bclk=gpio_num_t(PIN_BCLK);
  config.gpio_cfg.ws=gpio_num_t(PIN_WS);
  config.gpio_cfg.dout=I2S_GPIO_UNUSED;
  config.gpio_cfg.din=gpio_num_t(PIN_MIC_SD);
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rxChannel, &config));
  i2s_event_callbacks_t events = {};
  events.on_recv_q_ovf=onRxOverflow;
  ESP_ERROR_CHECK(i2s_channel_register_event_callback(rxChannel, &events, nullptr));
  ESP_ERROR_CHECK(i2s_channel_enable(rxChannel));
  delay(350);  // Estabilizar el microfono con SCK/WS ya activos antes de ofrecer c.
  Serial.println("ETAPA1 MEDICIONES - GPIO10 - Fs nominal16000Hz - INMP44124/slots32");
  printHelp();
}

void tick() {
  while (Serial.available()) {
    const char ch=char(Serial.read());
    if (ch=='\r') continue;
    if (ch=='\n') {
      command[commandLength]='\0';
      if (commandOverflow) Serial.println("Comando demasiado largo.");
      else if (commandLength) handleCommand(command);
      commandLength=0;
      commandOverflow=false;
    } else if (commandLength+1<sizeof(command)) command[commandLength++]=ch;
    else commandOverflow=true;
  }
  delay(1);
}
}  // namespace mediciones
#endif
