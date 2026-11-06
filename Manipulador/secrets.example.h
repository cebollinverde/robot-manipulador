#pragma once

// Copiar este archivo como "secrets.h" y reemplazar los valores de ejemplo.
// "secrets.h" esta ignorado por Git y no debe subirse al repositorio.

#define BLYNK_AUTH_TOKEN "PEGA_AQUI_EL_TOKEN_DE_BLYNK"

// El ESP32 intentara conectarse a cualquiera de estas redes.
// Agrega o quita filas segun sea necesario. Solo admite WiFi de 2.4 GHz.
const char* REDES[][2] = {
  {"NOMBRE_WIFI", "CLAVE_WIFI"},
  {"OTRA_WIFI",   "OTRA_CLAVE"},
};

const int N_REDES = sizeof(REDES) / sizeof(REDES[0]);
