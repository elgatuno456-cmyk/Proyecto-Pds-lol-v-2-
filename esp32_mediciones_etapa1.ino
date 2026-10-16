/*
  Etapa 1 - MEDICIONES. Arduino-ESP32 3.x, ESP32-S3.
  Programa separado: no reemplazar el proyecto de audio con este archivo.
  Captura I2S real con c; pruebas del mismo EchoDSP con h/t/v.
  Durante estas mediciones DIN del MAX98357 permanece en cero (silencio).
*/
#include "MedicionesApp.h"

void setup() { mediciones::begin(); }
void loop() { mediciones::tick(); }
