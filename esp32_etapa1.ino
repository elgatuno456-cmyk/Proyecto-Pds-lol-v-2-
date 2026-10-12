#ifndef MODO_SIN_HARDWARE
#define MODO_SIN_HARDWARE 1  // Cambiar a 0 para usar microfono y parlante.
#endif

#include <Arduino.h>
#include <math.h>
#include "EchoDSP.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Selecciona una placa ESP32-S3 en Arduino IDE."
#endif

#if !MODO_SIN_HARDWARE
#include <Wire.h>
#include "driver/i2s_std.h"
#include "esp_err.h"
#endif

constexpr size_t BLOCK_FRAMES = 128;  
constexpr float MAX_OUTPUT_GAIN = 0.5f;
constexpr float MAX_DELAY_MS = 1000.0f * dsp::kMaxDelaySamples / dsp::kSampleRate;
constexpr float MIC_INPUT_GAIN = 1.0f; 
constexpr int PIN_BCLK = 4;
constexpr int PIN_WS = 5;
constexpr int PIN_MIC_SD = 6;
constexpr int PIN_AMP_DIN = 7;
constexpr int PIN_SDA = 8;
constexpr int PIN_SCL = 9;
constexpr uint8_t ADS_ADDRESS = 0x48;  
constexpr float POT_SUPPLY_V = 3.3f;  

struct Controls {
  dsp::Mode mode = dsp::Mode::Loopback;
  float gain = 0.5f;
  float wet = 0.5f;
  size_t delaySamples = dsp::kDefaultDelaySamples;
  bool reset = false;
};
Controls controls;
dsp::Echo effect;                     
bool potEnabled = false;             
char commandBuffer[48];
size_t commandLength = 0;
bool commandOverflow = false;

#if !MODO_SIN_HARDWARE
portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;
i2s_chan_handle_t txChannel = nullptr;
i2s_chan_handle_t rxChannel = nullptr;
int32_t rxBuffer[2 * BLOCK_FRAMES];    
int32_t txBuffer[2 * BLOCK_FRAMES];
bool adsAvailable = false;
struct Telemetry {
  float peakIn = 0;
  float peakOut = 0;
  uint32_t clips = 0;
  uint32_t ioErrors = 0;
  uint32_t blocks = 0;
  uint32_t maxDspUs = 0;             
};
Telemetry telemetry;
#endif


Controls readControls(bool consumeReset = false);
void writeControls(const Controls &value);
void configureEffect(const Controls &c);

Controls readControls(bool consumeReset) {
#if !MODO_SIN_HARDWARE
  portENTER_CRITICAL(&controlMux);
#endif
  Controls copy = controls;
  if (consumeReset) controls.reset = false;
  else copy.reset = false;             
#if !MODO_SIN_HARDWARE
  portEXIT_CRITICAL(&controlMux);
#endif
  return copy;
}

void writeControls(const Controls &value) {
#if !MODO_SIN_HARDWARE
  portENTER_CRITICAL(&controlMux);
#endif
 
  const bool pendingReset = controls.reset;
  controls = value;
  controls.reset = pendingReset || value.reset;
#if !MODO_SIN_HARDWARE
  portEXIT_CRITICAL(&controlMux);
#endif
}

void configureEffect(const Controls &c) {
  effect.setMode(c.mode);
  effect.setGain(c.gain);
  effect.setWet(c.wet);
  effect.setDelaySamples(c.delaySamples);
  if (c.reset) effect.reset();
}

void printHelp() {
  Serial.println("b: loopback | e: eco | r: solo retardo");
  Serial.printf("d 250: retardo ms (1..%.1f) | a 0.5: alpha (0..1)\n", MAX_DELAY_MS);
  Serial.println("g 0.5: ganancia (0..0.5); desactiva control por pot");
  Serial.println("p 1: control ADS1115 | p 0: ganancia manual | ?: ayuda");
#if MODO_SIN_HARDWARE
  Serial.println("t: pruebas DSP | h: impulso con parametros actuales");
#endif
}

#if MODO_SIN_HARDWARE
void printImpulse() {
  Controls c = readControls();
  c.mode = dsp::Mode::Echo;
  c.reset = true;
  configureEffect(c);
  Serial.println("SIMULACION DSP: impulso, modo eco. Solo filas no nulas.");
  Serial.println("n,h");
  for (size_t n = 0; n <= c.delaySamples + 1; ++n) {
    float y = effect.process(n == 0 ? 1.0f : 0.0f);
    if (y != 0.0f) Serial.printf("%u,%.7f\n", (unsigned)n, y);
  }
  effect.reset();
}

float testSignal(size_t n, int which) {
  const float frequency = which == 1 ? 440.0f : 730.0f;
  return 0.1f * sinf(2.0f * PI * frequency * n / dsp::kSampleRate);
}

void runSyntheticTests() {
  
  Controls c = readControls();
  c.mode = dsp::Mode::Echo;
  c.reset = true;
  configureEffect(c);
  size_t wrongImpulse = 0;
  for (size_t n = 0; n <= c.delaySamples + 2; ++n) {
    const float actual = effect.process(n == 0 ? 1.0f : 0.0f);
    const float expected = n == 0 ? c.gain :
                           (n == c.delaySamples ? c.gain*c.wet : 0.0f);
    if (fabsf(actual - expected) > 1e-6f) ++wrongImpulse;
  }
  
  static float sumOfOutputs[1024];
  effect.setDelaySamples(37);         
  effect.reset();
  for (size_t n = 0; n < 1024; ++n)
    sumOfOutputs[n] = effect.process(testSignal(n, 1));
  effect.reset();
  for (size_t n = 0; n < 1024; ++n)
    sumOfOutputs[n] += effect.process(testSignal(n, 2));
  effect.reset();
  float addError = 0;
  for (size_t n = 0; n < 1024; ++n) {
    const float together = effect.process(testSignal(n,1) + testSignal(n,2));
    addError = fmaxf(addError, fabsf(together - sumOfOutputs[n]));
  }
  effect.reset();
  for (size_t n = 0; n < 1024; ++n)
    sumOfOutputs[n] = effect.process(testSignal(n, 1));
  effect.reset();
  float scaleError = 0;
  for (size_t n = 0; n < 1024; ++n) {
    const float scaled = effect.process(0.3f * testSignal(n,1));
    scaleError = fmaxf(scaleError, fabsf(scaled - 0.3f*sumOfOutputs[n]));
  }
  Serial.println("PRUEBAS SINTETICAS: no prueban I2S, ADS ni audio fisico.");
  Serial.printf("Impulso usa D=%u; linealidad usa D=37 muestras.\n", (unsigned)c.delaySamples);
  Serial.printf("Impulso: %s\n", wrongImpulse == 0 ? "PASS" : "FAIL");
  Serial.printf("Aditividad error max: %.9g (%s)\n", addError,
                addError < 1e-6f ? "PASS" : "FAIL");
  Serial.printf("Homogeneidad error max: %.9g (%s)\n", scaleError,
                scaleError < 1e-6f ? "PASS" : "FAIL");
  c.reset = true;
  configureEffect(c);                
}
#else
bool adsWriteRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(ADS_ADDRESS);
  Wire.write(reg);
  Wire.write(uint8_t(value >> 8));
  Wire.write(uint8_t(value & 0xff));
  return Wire.endTransmission() == 0;
}

bool adsReadRegister(uint8_t reg, int16_t &value) {
  Wire.beginTransmission(ADS_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADS_ADDRESS, uint8_t(2)) != 2) return false;
  const uint16_t high = uint16_t(Wire.read());
  const uint16_t low = uint16_t(Wire.read());
  value = int16_t((high << 8) | low);
  return true;
}

void updatePot() {
  static uint32_t lastReadMs = 0;
  if (!potEnabled || !adsAvailable || millis() - lastReadMs < 25) return;
  lastReadMs = millis();
  int16_t raw;
  if (!adsReadRegister(0x00, raw)) {
    adsAvailable = false;
    potEnabled = false;
    Serial.println("ADS1115 dejo de responder; se mantiene la ultima ganancia.");
    return;
  }
  
  const float voltage = fmaxf(0.0f, float(raw) * 0.000125f);
  const float fraction = fminf(1.0f, voltage / POT_SUPPLY_V);
  Controls c = readControls();
  c.gain = fraction * MAX_OUTPUT_GAIN;
  writeControls(c);
}

int32_t toPcm32(float sample, uint32_t &clipCount) {
  
  if (sample > 0.99999988f) { ++clipCount; return INT32_MAX; }
  if (sample < -1.0f) { ++clipCount; return INT32_MIN; }
  return int32_t(double(sample) * 2147483648.0);
}

void audioTask(void *) {
  Telemetry local;
  while (true) {
    size_t bytesRead = 0;
    const esp_err_t readError = i2s_channel_read(
        rxChannel, rxBuffer, sizeof(rxBuffer), &bytesRead, 100);
    if (readError != ESP_OK || bytesRead != sizeof(rxBuffer)) {
      ++local.ioErrors;
      portENTER_CRITICAL(&controlMux);
      telemetry = local;
      portEXIT_CRITICAL(&controlMux);
      vTaskDelay(1);                    
      continue;                      
    }
    Controls c = readControls(true);
    configureEffect(c);
    float peakIn = 0, peakOut = 0;
    uint32_t startUs = micros();
    for (size_t i = 0; i < BLOCK_FRAMES; ++i) {
      const float x = float(rxBuffer[2*i]) * (1.0f / 2147483648.0f)
                    * MIC_INPUT_GAIN;
      const float y = effect.process(x);
      peakIn = fmaxf(peakIn, fabsf(x));
      peakOut = fmaxf(peakOut, fabsf(y));
      const int32_t pcm = toPcm32(y, local.clips);
      txBuffer[2*i] = pcm;
      txBuffer[2*i+1] = pcm;           
    }
    local.maxDspUs = max(local.maxDspUs, uint32_t(micros() - startUs));
    size_t bytesWritten = 0;
    const esp_err_t writeError = i2s_channel_write(
        txChannel, txBuffer, sizeof(txBuffer), &bytesWritten, 100);
    if (writeError != ESP_OK || bytesWritten != sizeof(txBuffer))
      ++local.ioErrors;
    local.peakIn = peakIn;
    local.peakOut = peakOut;
    ++local.blocks;
    portENTER_CRITICAL(&controlMux);
    telemetry = local;
    portEXIT_CRITICAL(&controlMux);
  }
}

void startAudio() {
  i2s_chan_config_t channelConfig =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  channelConfig.dma_desc_num = 6;
  channelConfig.dma_frame_num = BLOCK_FRAMES;
  channelConfig.auto_clear = true;     
  ESP_ERROR_CHECK(i2s_new_channel(&channelConfig, &txChannel, &rxChannel));

  i2s_std_config_t config = {};
  config.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(dsp::kSampleRate);
  config.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  config.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  config.gpio_cfg.bclk = gpio_num_t(PIN_BCLK);
  config.gpio_cfg.ws = gpio_num_t(PIN_WS);
  config.gpio_cfg.din = gpio_num_t(PIN_MIC_SD);
  config.gpio_cfg.dout = gpio_num_t(PIN_AMP_DIN);
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(txChannel, &config));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rxChannel, &config));
  
  size_t loaded = 0;
  ESP_ERROR_CHECK(i2s_channel_preload_data(
      txChannel, txBuffer, sizeof(txBuffer), &loaded));
  if (loaded != sizeof(txBuffer)) Serial.println("Aviso: precarga parcial TX.");
  ESP_ERROR_CHECK(i2s_channel_enable(txChannel));
  ESP_ERROR_CHECK(i2s_channel_enable(rxChannel));
  BaseType_t ok = xTaskCreatePinnedToCore(audioTask, "audio", 4096,
                                        nullptr, 5, nullptr, 0);
  if (ok != pdPASS) {
    Serial.println("ERROR: no se pudo crear la tarea de audio.");
    ESP_ERROR_CHECK(i2s_channel_disable(rxChannel));
    ESP_ERROR_CHECK(i2s_channel_disable(txChannel));
    while (true) delay(1000);
  }
}
#endif

void handleCommand(const char *line) {
  Controls c = readControls();
  float value = 0;
  const bool hasValue = sscanf(line + 1, "%f", &value) == 1 && isfinite(value);
  switch (line[0]) {
    case '?': printHelp(); return;
    case 'b': c.mode = dsp::Mode::Loopback; c.reset = true; break;
    case 'e': c.mode = dsp::Mode::Echo; c.reset = true; break;
    case 'r': c.mode = dsp::Mode::Delay; c.reset = true; break;
    case 'd':
      if (!hasValue || value < 1 || value > MAX_DELAY_MS) break;
      c.delaySamples = size_t(lroundf(value * dsp::kSampleRate / 1000.0f));
      c.reset = true;
      writeControls(c); Serial.println("Retardo actualizado; memoria vaciada."); return;
    case 'a':
      if (!hasValue || value < 0 || value > 1) break;
      c.wet = value; writeControls(c); Serial.println("Alpha actualizado."); return;
    case 'g':
      if (!hasValue || value < 0 || value > MAX_OUTPUT_GAIN) break;
      potEnabled = false; c.gain = value;
      writeControls(c); Serial.println("Ganancia manual actualizada."); return;
    case 'p':
#if MODO_SIN_HARDWARE
      Serial.println("Modo sin hardware: ADS1115 no se lee; usa g 0.25.");
#else
      if (!hasValue || (value != 0 && value != 1)) break;
      potEnabled = value == 1 && adsAvailable;
      Serial.println(potEnabled ? "Control por pot activo." : "Control por pot inactivo.");
#endif
      return;
#if MODO_SIN_HARDWARE
    case 't': runSyntheticTests(); return;
    case 'h': printImpulse(); return;
#endif
    default: Serial.println("Comando desconocido; escribe ?."); return;
  }
  if (line[0] != 'b' && line[0] != 'e' && line[0] != 'r') {
    Serial.println("Parametro fuera de rango o incompleto; escribe ?."); return;
  }
  writeControls(c);
  Serial.println("Modo actualizado; memoria vaciada.");
}

void pollSerial() {
  
  while (Serial.available()) {
    char ch = char(Serial.read());
    if (ch == '\r') continue;
    if (ch == '\n') {
      commandBuffer[commandLength] = '\0';
      if (commandOverflow) Serial.println("Comando demasiado largo.");
      else if (commandLength != 0) handleCommand(commandBuffer);
      commandLength = 0;
      commandOverflow = false;
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = ch;
    } else commandOverflow = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);                       
  Serial.println("PDS etapa 1 - ESP32-S3 - parametros provisionales.");
  printHelp();
#if MODO_SIN_HARDWARE
  Serial.println("MODO SIN HARDWARE: no se inicializan I2S ni I2C.");
  runSyntheticTests();
  printImpulse();
#else
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(10);
  
  adsAvailable = adsWriteRegister(0x01, 0xC283);
  Serial.println(adsAvailable ? "ADS1115 configurado. Control pot aun inactivo."
                              : "ADS1115 no encontrado. Se usa ganancia manual.");
  startAudio();
  Serial.printf("I2S activo: %lu Hz, stereo, slots 32 bits; microfono 24 bits.\n",
                (unsigned long)dsp::kSampleRate);
#endif
}

void loop() {
  pollSerial();
#if !MODO_SIN_HARDWARE
  updatePot();                         
  static uint32_t lastReportMs = 0;
  if (Serial && millis() - lastReportMs >= 1000) {
    lastReportMs = millis();
    portENTER_CRITICAL(&controlMux);
    const Telemetry t = telemetry;
    portEXIT_CRITICAL(&controlMux);
    const Controls c = readControls();
    Serial.printf("g=%.3f D=%u peakX=%.4f peakY=%.4f clips=%lu errores=%lu DSPmax=%luus bloques=%lu\n",
      c.gain, (unsigned)c.delaySamples, t.peakIn, t.peakOut,
      (unsigned long)t.clips, (unsigned long)t.ioErrors,
      (unsigned long)t.maxDspUs, (unsigned long)t.blocks);
  }
#endif
  delay(1);                           
}
