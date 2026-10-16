/*
  Diagnostico independiente: ESP32-S3 -> MAX98357 I2S -> parlante.
  Arduino-ESP32 3.x. Seno de 1 kHz, amplitud 0.02, Fs = 16000 Hz.
  TX solo: GPIO4 BCLK, GPIO5 LRC/WS, GPIO7 DIN del amplificador.
  No captura microfono, no usa ADS1115 y no aplica eco.
*/
#include <Arduino.h>
#include <math.h>
#include "driver/i2s_std.h"
#include "esp_err.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Selecciona una placa ESP32-S3 en Arduino IDE."
#endif

constexpr uint32_t SAMPLE_RATE = 16000;
constexpr uint32_t TONE_HZ = 1000;
constexpr size_t PERIOD_FRAMES = SAMPLE_RATE / TONE_HZ;
constexpr size_t BLOCK_FRAMES = 128;
constexpr size_t DMA_BUFFERS = 6;
constexpr float AMPLITUDE = 0.02f;
constexpr int PIN_BCLK = 4;
constexpr int PIN_WS = 5;
constexpr int PIN_AMP_DIN = 7;
static_assert(SAMPLE_RATE % TONE_HZ == 0, "El periodo debe ser entero.");
static_assert(BLOCK_FRAMES % PERIOD_FRAMES == 0, "El bloque debe tener periodos completos.");

// Una trama contiene L y R. El bloque contiene ocho periodos completos.
int32_t tone[2 * BLOCK_FRAMES];
i2s_chan_handle_t txChannel = nullptr;
portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
struct Status {
  uint32_t blocks = 0;
  uint32_t errors = 0;
  uint32_t incompleteWrites = 0;
  esp_err_t lastError = ESP_OK;
  bool stopped = false;
};
Status status;

// Prototipos explicitos para el preprocesador de Arduino.
void makeTone();
void audioTask(void *);
void preloadTone();

void makeTone() {
  for (size_t n = 0; n < BLOCK_FRAMES; ++n) {
    const double phase = 2.0 * PI * (n % PERIOD_FRAMES) / PERIOD_FRAMES;
    const int32_t pcm = int32_t(lround(double(AMPLITUDE) * sin(phase) * 2147483648.0));
    tone[2 * n] = pcm;
    tone[2 * n + 1] = pcm;  // Misma senal en L y R.
  }
}

void preloadTone() {
  // Cargar las seis memorias DMA antes de producir relojes I2S.
  const uint8_t *data = reinterpret_cast<const uint8_t *>(tone);
  for (size_t block = 0; block < DMA_BUFFERS; ++block) {
    size_t offset = 0;
    while (offset < sizeof(tone)) {
      size_t loaded = 0;
      ESP_ERROR_CHECK(i2s_channel_preload_data(
          txChannel, data + offset, sizeof(tone) - offset, &loaded));
      if (loaded == 0) {
        Serial.println("ERROR: la precarga DMA quedo incompleta; no se inicia el tono.");
        while (true) delay(1000);
      }
      offset += loaded;
    }
  }
}

void audioTask(void *) {
  Status local;
  local.lastError = i2s_channel_enable(txChannel);
  if (local.lastError != ESP_OK) {
    local.errors = 1;
    local.stopped = true;
    portENTER_CRITICAL(&statusMux);
    status = local;
    portEXIT_CRITICAL(&statusMux);
    vTaskDelete(nullptr);
    return;
  }

  const uint8_t *data = reinterpret_cast<const uint8_t *>(tone);
  size_t offset = 0;
  while (true) {
    const size_t remaining = sizeof(tone) - offset;
    size_t written = 0;
    const esp_err_t result = i2s_channel_write(
        txChannel, data + offset, remaining, &written, 100);
    if (written != remaining) ++local.incompleteWrites;
    if (result != ESP_OK) {
      ++local.errors;
      local.lastError = result;
    }
    offset += written;  // Reintentar solo lo pendiente: no saltar muestras.
    if (offset == sizeof(tone)) {
      offset = 0;
      ++local.blocks;
    }

    // Un timeout ya cedio CPU al esperar. Un fallo inmediato sin progreso
    // detiene la prueba para evitar un bucle que bloquee el equipo.
    if (written == 0 && result != ESP_ERR_TIMEOUT) {
      if (result == ESP_OK) {
        ++local.errors;
        local.lastError = ESP_FAIL;
      }
      local.stopped = true;
    }
    portENTER_CRITICAL(&statusMux);
    status = local;
    portEXIT_CRITICAL(&statusMux);
    if (local.stopped) {
      i2s_channel_disable(txChannel);
      vTaskDelete(nullptr);
      return;
    }
    // Sin delay ni mensajes: la escritura espera al ritmo del reloj I2S.
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);  // Solo al arrancar, antes de iniciar el audio.
  Serial.println("PRUEBA DE TONO: 1000 Hz, amplitud 0.02, sin microfono ni eco.");
  Serial.println("Confirmar cableado: GPIO4->BCLK, GPIO5->LRC, GPIO7->DIN.");
  makeTone();

  i2s_chan_config_t channelConfig =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  channelConfig.dma_desc_num = DMA_BUFFERS;
  channelConfig.dma_frame_num = BLOCK_FRAMES;
  channelConfig.auto_clear = true;
  ESP_ERROR_CHECK(i2s_new_channel(&channelConfig, &txChannel, nullptr));

  i2s_std_config_t config = {};
  config.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  config.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  config.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  config.gpio_cfg.bclk = gpio_num_t(PIN_BCLK);
  config.gpio_cfg.ws = gpio_num_t(PIN_WS);
  config.gpio_cfg.dout = gpio_num_t(PIN_AMP_DIN);
  config.gpio_cfg.din = I2S_GPIO_UNUSED;
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(txChannel, &config));
  preloadTone();
  Serial.println("Precarga completa: 6 bloques, 6144 bytes. Iniciando audio.");
  if (xTaskCreatePinnedToCore(audioTask, "tono", 4096, nullptr, 5, nullptr, 0) != pdPASS) {
    Serial.println("ERROR: no se pudo crear la tarea de audio.");
    while (true) delay(1000);
  }
}

void loop() {
  static uint32_t lastReport = 0;
  if (Serial && millis() - lastReport >= 1000) {
    lastReport = millis();
    portENTER_CRITICAL(&statusMux);
    const Status copy = status;
    portEXIT_CRITICAL(&statusMux);
    Serial.printf("bloques=%lu errores=%lu incompletas=%lu ultimo=%s estado=%s\n",
        (unsigned long)copy.blocks, (unsigned long)copy.errors,
        (unsigned long)copy.incompleteWrites, esp_err_to_name(copy.lastError),
        copy.stopped ? "DETENIDO" : "en marcha");
  }
  delay(1);  // Solo la tarea de informes; el audio corre en otra tarea.
}
