/*
  Manipulador robotico - ESP32 + PCA9685 + HC-SR04 + Blynk
  ---------------------------------------------------------
  Flujo: App Blynk <-> ESP32. Las rutinas generales siguen pudiendo usar
  Python por USB; la secuencia validada de dos cajas corre dentro del ESP32.

  Cambios respecto a la version anterior:
   1. Blynk/WiFi corre en el NUCLEO 0 (tarea propia). El nucleo 1 (loop) solo
      mueve servos y atiende al PC, asi una reconexion WiFi ya no congela el brazo.
   2. Blynk ya no escribe al Serial ni a Blynk desde otro nucleo: deja "pedidos"
      que el loop atiende, y el loop deja "publicaciones" que la tarea Blynk envia.
   3. DIST entrega la mediana de 5 lecturas (deteccion mas confiable).
   4. Vigilancia: si se pidio una rutina y el PC no responde en 15 s, se libera
      el control manual (antes quedaba bloqueado para siempre).
   5. REPORT con un estado no final marca la rutina como activa (bloquea los
      sliders de la app mientras Python mueve el brazo).
  El protocolo serial con Python es el mismo de antes.
*/
#define BLYNK_TEMPLATE_ID "TMPL24p6M39YQ"
#define BLYNK_TEMPLATE_NAME "Manipulador Robotico"

// Token de Blynk y redes WiFi. Este archivo es local y esta ignorado por Git.
// La primera vez, copiar secrets.example.h como secrets.h y completar los datos.
#include "secrets.h"

#include <WiFi.h>
#include <WiFiMulti.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <math.h>

// ==================================================
// 1. CONFIGURACION
// ==================================================

// Redes WiFi: se conecta a la que encuentre (casa o laboratorio).
// El ESP32 solo usa 2.4 GHz: si el router separa 2.4/5 GHz, usar la de 2.4.
// Las redes se configuran en secrets.h como pares { "nombre", "clave" }.
// Solo sirven redes de 2.4 GHz y sin pagina de inicio de sesion.
WiFiMulti redes;

Adafruit_PWMServoDriver pca(0x40);

const int SDA_PCA = 26;
const int SCL_PCA = 27;
const int TRIG = 25;
const int ECHO = 33;   // OJO: el ECHO del HC-SR04 sale a 5V -> divisor 1k/2k a 3.3V

// Servo:          1 base, 2 hombro, 3 codo, 4 giro muneca, 5 muneca, 6 pinza
const int canales[6] = {10, 11, 12, 13, 14, 15};
const int PULSO_MIN = 1000;
const int PULSO_MAX = 2000;

// Porcentajes ORIGINALES de los limites comprobados
const float minimo[6] = {5, 15, 0, 10, 25, 20};
const float maximo[6] = {80, 93, 80, 78, 70, 85};

// Mantener la postura fisica de calibracion (antiguo 50%)
// volatile: los escribe Blynk (nucleo 0) y el loop (nucleo 1)
float actual[6] = {50, 50, 50, 50, 50, 50};
volatile float destino[6] = {50, 50, 50, 50, 50, 50};

// Pinza: porcentajes ORIGINALES. Se ajustan con el slider V16 de la app
// (o con el comando PINZA,abierta,cerrada) y quedan guardados en el ESP32.
float pinzaAbierta = 110.0f * 100.0f / 180.0f;   // valor inicial (~61 %)
float pinzaCerrada = 65.0f * 100.0f / 180.0f;    // valor inicial (~36 %)
Preferences memoria;

// Mensajes de diagnostico (WiFi/Blynk) que el nucleo 0 deja y el loop imprime
char infoSerial[200] = "";
volatile bool hayInfo = false;
volatile bool pedidoEscaneo = false;

const unsigned long VIGILANCIA_MS = 15000;   // sin noticias del PC -> liberar

// Coincide con la velocidad de la rutina enseñada y con el valor
// predeterminado de V12 en Blynk.
volatile int velocidad = 30;
volatile int rutina = 1;      // si la app aun no manda V8: Apilar
volatile bool rutinaActiva = false;
volatile bool bloqueado = false;

// La rutina 4 (dos cajas A->B) se ejecuta sin PC. Cada estado espera que el
// movimiento anterior termine, de modo que Blynk y STOP siguen respondiendo.
enum EstadoApiladoBlynk : uint8_t {
  AP_INACTIVO,
  AP_INICIO_A, AP_INICIO_B, AP_SENSOR, AP_ESPERA_CAJA, AP_SEGURIDAD,
  AP_ABRIR_A, AP_APROX_AGARRE, AP_AGARRE, AP_CERRAR_A,
  AP_ELEVACION_1, AP_ELEVACION_2, AP_ELEVACION_3, AP_A_ALTA, AP_B_ALTA,
  AP_B1_APROX, AP_B1_DEPOSITO, AP_B1_ABRIR, AP_B1_RETIRAR, AP_B1_ALTA,
  AP_VOLVER_A,
  AP_B2_APROX, AP_B2_PRECOLOCAR, AP_B2_SUELTA, AP_B2_ABRIR,
  AP_B2_RETIRAR, AP_B2_ALTA
};

EstadoApiladoBlynk estadoApilado = AP_INACTIVO;
bool esperandoPC = false;
int cajaApilado = 0;
int confirmacionesCaja = 0;
unsigned long inicioEstadoApilado = 0;
unsigned long inicioEsperaCaja = 0;
unsigned long ultimaMedicionCaja = 0;

const unsigned long ESPERA_CAJA_MS = 120000;
const unsigned long SEGURIDAD_MANOS_MS = 2000;
const float UMBRAL_CAJA_CM = 12.0f;
const float PINZA_ABIERTA_RUTINA = 95.0f;
const float PINZA_CERRADA_RUTINA = 60.0f;

// Posturas normalizadas ensenadas y validadas fisicamente el 2026-11-06.
const float P_A_SENSOR[5] = {14.169f, 45.161f, 89.474f, 58.824f, 3.800f};
const float P_A_APROX[5] = {14.167f, 42.189f, 92.145f, 58.824f, 9.733f};
const float P_A_AGARRE[5] = {14.167f, 24.217f, 84.822f, 58.824f, 19.930f};
const float P_A_ELEV1[5] = {14.167f, 31.144f, 86.543f, 58.824f, 10.983f};
const float P_A_ELEV2[5] = {14.167f, 36.625f, 89.559f, 58.824f, 10.813f};
const float P_A_ELEV3[5] = {14.167f, 42.189f, 92.145f, 58.824f, 9.733f};
const float P_A_ALTA[5] = {14.169f, 53.458f, 95.982f, 58.824f, 4.958f};
const float P_B_ALTA[5] = {84.189f, 35.857f, 68.231f, 58.824f, 2.005f};
const float P_B1_APROX[5] = {84.1891f, 25.8766f, 65.7015f, 58.8235f, 10.8387f};
const float P_B1_DEPOSITO[5] = {84.1891f, 8.3154f, 54.8611f, 58.8235f, 14.070f};
const float P_B2_APROX[5] = {84.189f, 31.765f, 65.844f, 58.824f, 0.886f};
const float P_B2_PRE[5] = {84.189f, 23.254f, 62.675f, 58.824f, 6.035f};
const float P_B2_SUELTA[5] = {84.190f, 21.880f, 59.630f, 58.820f, 3.010f};

// Pedidos de Blynk (nucleo 0) que atiende el loop (nucleo 1)
volatile int pedidoRutina = 0;
volatile bool pedidoStop = false;
volatile bool pedidoReanudar = false;
volatile int pedidoGuardarPinza = 0;    // 1 = abierta, 2 = cerrada (botones V17/V18)

unsigned long ultimoPaso = 0;
unsigned long ultimoSensor = 0;
unsigned long ultimoSerial = 0;

// Publicaciones pendientes hacia la app (las envia la tarea Blynk)
portMUX_TYPE candado = portMUX_INITIALIZER_UNLOCKED;
struct Publicaciones {
  bool estado = false;
  char texto[32] = "";
  bool xyz = false;
  float x = 0, y = 0, z = 0;
  bool sliders = false;
  bool interruptor = false;
  bool vel = false;
  bool dist = false;
  float d = -1;
} pend;

void publicarEstado(const char* texto);
void publicarXYZ(float x, float y, float z);
void actualizarSliders();
void publicarInterruptor();
void publicarVelocidad();
void publicarDistancia(float d);
void iniciarBlynk();
void iniciarApiladoBlynk();
void atenderApiladoBlynk();


// ==================================================
// 2. MOVIMIENTO DE SERVOS
// ==================================================

// La app y Python usan 0-100% NORMALIZADO.
// Internamente el ESP32 guarda porcentajes ORIGINALES.
float convertir(int i, float nuevo) {
  return minimo[i] + nuevo * (maximo[i] - minimo[i]) / 100.0f;
}

float normalizar(int i, float antiguo) {
  return 100.0f * (antiguo - minimo[i]) / (maximo[i] - minimo[i]);
}

// Numero de servo: 1 a 6
bool fijarServo(int numero, float nuevo) {
  if (numero < 1 || numero > 6 || !isfinite(nuevo) ||
      nuevo < 0 || nuevo > 100) return false;

  int i = numero - 1;
  destino[i] = convertir(i, nuevo);
  return true;
}

void escribirServo(int i, float antiguo) {
  int pulso = PULSO_MIN + (PULSO_MAX - PULSO_MIN) * antiguo / 100.0f;
  pca.writeMicroseconds(canales[i], pulso);
}

void mover() {
  if (millis() - ultimoPaso < 20) return;
  ultimoPaso = millis();

  float paso = 0.1f + velocidad * 0.012f;

  for (int i = 0; i < 6; i++) {
    float diferencia = destino[i] - actual[i];
    if (fabs(diferencia) < 0.01f) continue;

    if (fabs(diferencia) <= paso) actual[i] = destino[i];
    else if (diferencia > 0) actual[i] += paso;
    else actual[i] -= paso;

    escribirServo(i, actual[i]);
  }
}

bool enMovimiento() {
  for (int i = 0; i < 6; i++) {
    if (fabs(destino[i] - actual[i]) > 0.01f) return true;
  }
  return false;
}

void detener() {
  for (int i = 0; i < 6; i++) destino[i] = actual[i];
  estadoApilado = AP_INACTIVO;
  esperandoPC = false;
  rutinaActiva = false;
  bloqueado = true;
  publicarEstado("DETENIDO");
  publicarInterruptor();
}


// ==================================================
// 3. SENSOR ULTRASONICO (montado en la pinza)
// ==================================================

float lecturaUnica() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long tiempo = pulseIn(ECHO, HIGH, 25000);
  if (tiempo == 0) return -1;

  float cm = tiempo * 0.0343f / 2.0f;
  return (cm >= 2 && cm <= 400) ? cm : -1;
}

// Mediana de 5 lecturas validas (descarta ecos sueltos)
float distancia() {
  float v[5];
  int n = 0;
  for (int k = 0; k < 5; k++) {
    float d = lecturaUnica();
    if (d > 0) v[n++] = d;
    delay(35);                      // dejar que se apague el eco anterior
  }
  if (n == 0) return -1;
  for (int a = 1; a < n; a++) {     // orden por insercion
    float x = v[a];
    int b = a - 1;
    while (b >= 0 && v[b] > x) { v[b + 1] = v[b]; b--; }
    v[b + 1] = x;
  }
  return v[n / 2];
}


// ==================================================
// 4. RUTINA BLYNK AUTONOMA: DOS CAJAS A -> B
// ==================================================

void irPosturaApilado(EstadoApiladoBlynk siguiente, const float postura[5],
                      const char* texto = nullptr) {
  for (int i = 0; i < 5; i++) fijarServo(i + 1, postura[i]);
  estadoApilado = siguiente;
  inicioEstadoApilado = millis();
  actualizarSliders();
  if (texto != nullptr) publicarEstado(texto);
}

void terminarApiladoBlynk(const char* texto) {
  estadoApilado = AP_INACTIVO;
  esperandoPC = false;
  rutinaActiva = false;
  actualizarSliders();
  publicarEstado(texto);
  Serial.print("APILADO,");
  Serial.println(texto);
}

void esperarCajaApilado() {
  estadoApilado = AP_ESPERA_CAJA;
  inicioEstadoApilado = millis();
  inicioEsperaCaja = millis();
  ultimaMedicionCaja = 0;
  confirmacionesCaja = 0;
  char texto[32];
  snprintf(texto, sizeof(texto), "COLOQUE CAJA %d EN A", cajaApilado);
  publicarEstado(texto);
}

void iniciarApiladoBlynk() {
  rutinaActiva = true;
  esperandoPC = false;
  cajaApilado = 1;
  confirmacionesCaja = 0;

  // Evita que un valor antiguo de V12 deje la rutina practicamente inmovil.
  // Se conserva cualquier ajuste del usuario que sea mas rapido que 30.
  if (velocidad < 30) velocidad = 30;
  publicarVelocidad();

  // Igual que la rutina Python: primero sube por el punto alto mas cercano.
  float distanciaA = 0, distanciaB = 0;
  for (int i = 0; i < 5; i++) {
    float ahora = normalizar(i, actual[i]);
    distanciaA += fabs(ahora - P_A_ALTA[i]);
    distanciaB += fabs(ahora - P_B_ALTA[i]);
  }
  if (distanciaB < distanciaA) {
    irPosturaApilado(AP_INICIO_B, P_B_ALTA, "INICIANDO DESDE B");
  } else {
    irPosturaApilado(AP_INICIO_A, P_A_ALTA, "INICIANDO DESDE A");
  }
  Serial.println("APILADO,INICIO AUTONOMO");
}

void atenderApiladoBlynk() {
  if (estadoApilado == AP_INACTIVO || bloqueado) return;

  switch (estadoApilado) {
    case AP_INICIO_B:
      if (!enMovimiento()) irPosturaApilado(AP_INICIO_A, P_A_ALTA, "YENDO A");
      break;
    case AP_INICIO_A:
    case AP_VOLVER_A:
      if (!enMovimiento()) irPosturaApilado(AP_SENSOR, P_A_SENSOR, "BUSCANDO CAJA");
      break;
    case AP_SENSOR:
      if (!enMovimiento()) esperarCajaApilado();
      break;
    case AP_ESPERA_CAJA:
      if (millis() - inicioEsperaCaja >= ESPERA_CAJA_MS) {
        terminarApiladoBlynk("CAJA NO DETECTADA");
        break;
      }
      if (millis() - ultimaMedicionCaja >= 300) {
        ultimaMedicionCaja = millis();
        float d = distancia();
        publicarDistancia(d);
        if (d >= 2.0f && d <= UMBRAL_CAJA_CM) confirmacionesCaja++;
        else confirmacionesCaja = 0;
        if (confirmacionesCaja >= 2) {
          estadoApilado = AP_SEGURIDAD;
          inicioEstadoApilado = millis();
          char texto[32];
          snprintf(texto, sizeof(texto), "CAJA %d: RETIRE MANOS", cajaApilado);
          publicarEstado(texto);
        }
      }
      break;
    case AP_SEGURIDAD:
      if (millis() - inicioEstadoApilado >= SEGURIDAD_MANOS_MS) {
        fijarServo(6, PINZA_ABIERTA_RUTINA);
        estadoApilado = AP_ABRIR_A;
        actualizarSliders();
        publicarEstado(cajaApilado == 1 ? "TOMANDO CAJA 1" : "TOMANDO CAJA 2");
      }
      break;
    case AP_ABRIR_A:
      if (!enMovimiento()) irPosturaApilado(AP_APROX_AGARRE, P_A_APROX);
      break;
    case AP_APROX_AGARRE:
      if (!enMovimiento()) irPosturaApilado(AP_AGARRE, P_A_AGARRE);
      break;
    case AP_AGARRE:
      if (!enMovimiento()) {
        fijarServo(6, PINZA_CERRADA_RUTINA);
        estadoApilado = AP_CERRAR_A;
        actualizarSliders();
      }
      break;
    case AP_CERRAR_A:
      if (!enMovimiento()) {
        publicarEstado(cajaApilado == 1 ? "TRASLADANDO CAJA 1" : "TRASLADANDO CAJA 2");
        irPosturaApilado(AP_ELEVACION_1, P_A_ELEV1);
      }
      break;
    case AP_ELEVACION_1:
      if (!enMovimiento()) irPosturaApilado(AP_ELEVACION_2, P_A_ELEV2);
      break;
    case AP_ELEVACION_2:
      if (!enMovimiento()) irPosturaApilado(AP_ELEVACION_3, P_A_ELEV3);
      break;
    case AP_ELEVACION_3:
      if (!enMovimiento()) irPosturaApilado(AP_A_ALTA, P_A_ALTA);
      break;
    case AP_A_ALTA:
      if (!enMovimiento()) irPosturaApilado(AP_B_ALTA, P_B_ALTA);
      break;
    case AP_B_ALTA:
      if (!enMovimiento()) {
        if (cajaApilado == 1)
          irPosturaApilado(AP_B1_APROX, P_B1_APROX, "DEPOSITANDO CAJA 1");
        else
          irPosturaApilado(AP_B2_APROX, P_B2_APROX, "APILANDO CAJA 2");
      }
      break;
    case AP_B1_APROX:
      if (!enMovimiento()) irPosturaApilado(AP_B1_DEPOSITO, P_B1_DEPOSITO);
      break;
    case AP_B1_DEPOSITO:
      if (!enMovimiento()) {
        fijarServo(6, PINZA_ABIERTA_RUTINA);
        estadoApilado = AP_B1_ABRIR;
        actualizarSliders();
      }
      break;
    case AP_B1_ABRIR:
      if (!enMovimiento()) irPosturaApilado(AP_B1_RETIRAR, P_B1_APROX);
      break;
    case AP_B1_RETIRAR:
      if (!enMovimiento()) irPosturaApilado(AP_B1_ALTA, P_B_ALTA);
      break;
    case AP_B1_ALTA:
      if (!enMovimiento()) {
        cajaApilado = 2;
        irPosturaApilado(AP_VOLVER_A, P_A_ALTA, "VOLVIENDO POR CAJA 2");
      }
      break;
    case AP_B2_APROX:
      if (!enMovimiento()) irPosturaApilado(AP_B2_PRECOLOCAR, P_B2_PRE);
      break;
    case AP_B2_PRECOLOCAR:
      if (!enMovimiento()) irPosturaApilado(AP_B2_SUELTA, P_B2_SUELTA);
      break;
    case AP_B2_SUELTA:
      if (!enMovimiento()) {
        fijarServo(6, PINZA_ABIERTA_RUTINA);
        estadoApilado = AP_B2_ABRIR;
        actualizarSliders();
      }
      break;
    case AP_B2_ABRIR:
      if (!enMovimiento()) irPosturaApilado(AP_B2_RETIRAR, P_B2_APROX);
      break;
    case AP_B2_RETIRAR:
      if (!enMovimiento()) irPosturaApilado(AP_B2_ALTA, P_B_ALTA);
      break;
    case AP_B2_ALTA:
      if (!enMovimiento()) terminarApiladoBlynk("DOS CAJAS APILADAS");
      break;
    default:
      terminarApiladoBlynk("ERROR DE RUTINA");
      break;
  }
}


void avisarSerial(const char* texto) {
  portENTER_CRITICAL(&candado);
  strncpy(infoSerial, texto, sizeof(infoSerial) - 1);
  infoSerial[sizeof(infoSerial) - 1] = '\0';
  hayInfo = true;
  portEXIT_CRITICAL(&candado);
}

void guardarPinza(float abierta, float cerrada) {
  pinzaAbierta = abierta;
  pinzaCerrada = cerrada;
  memoria.putFloat("abierta", abierta);
  memoria.putFloat("cerrada", cerrada);
}


// ==================================================
// 4. COMUNICACION SERIAL CON PYTHON
// ==================================================

void comando(String dato) {
  if (dato != "STATE") ultimoSerial = millis();

  // WIFI -> estado de la conexion (para revisar en el Monitor serie)
  if (dato == "WIFI") {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("INFO,WIFI conectado a ");
      Serial.print(WiFi.SSID());
      Serial.print(" senal ");
      Serial.print(WiFi.RSSI());
      Serial.print(" dBm IP ");
      Serial.print(WiFi.localIP());
    } else {
      Serial.print("INFO,WIFI sin conexion");
    }
    Serial.println(Blynk.connected() ? " | BLYNK conectado" : " | BLYNK desconectado");
  }

  // REDES -> lista las redes de 2.4 GHz que ve el ESP32
  else if (dato == "REDES") {
    pedidoEscaneo = true;
    Serial.println("INFO,buscando redes (unos segundos)...");
  }

  else if (dato == "PING") {
    Serial.println("LISTO,PONG");
  }

  else if (dato == "STATE") {
    Serial.print(enMovimiento() ? "STATE,MOVING" : "STATE,IDLE");
    for (int i = 0; i < 6; i++) {
      Serial.print(",");
      Serial.print(normalizar(i, actual[i]), 2);
    }
    Serial.println();
  }

  else if (dato == "STOP") {
    detener();
    Serial.println("LISTO,STOP");
  }

  else if (dato == "RESUME") {
    bloqueado = false;
    publicarInterruptor();
    publicarEstado("CONECTADO");
    Serial.println("LISTO,RESUME");
  }

  else if (dato == "HOME") {
    if (bloqueado || rutinaActiva) {
      Serial.println("ERROR,STOP");
      return;
    }
    for (int i = 0; i < 6; i++) destino[i] = 50;
    actualizarSliders();
    Serial.println("LISTO,HOME");
  }

  else if (dato == "OPEN" || dato == "CLOSE") {
    if (bloqueado) {
      Serial.println("ERROR,STOP");
      return;
    }
    destino[5] = (dato == "OPEN") ? pinzaAbierta : pinzaCerrada;
    Serial.print(dato == "OPEN" ? "LISTO,OPEN," : "LISTO,CLOSE,");
    Serial.println(normalizar(5, destino[5]), 2);   // Python sabe donde quedo
    actualizarSliders();
  }

  // PINZA,90,20 -> abierta 90 %, cerrada 20 % (escala 0-100 de la app). Se guarda.
  else if (dato.startsWith("PINZA,")) {
    float a, c;
    char extra;
    if (sscanf(dato.c_str(), "PINZA,%f,%f%c", &a, &c, &extra) != 2 ||
        !isfinite(a) || !isfinite(c) || a < 0 || a > 100 || c < 0 || c > 100) {
      Serial.println("ERROR,PINZA");
      return;
    }
    guardarPinza(convertir(5, a), convertir(5, c));
    Serial.println("LISTO,PINZA");
  }

  else if (dato == "PINZA?") {
    Serial.print("PINZA,");
    Serial.print(normalizar(5, pinzaAbierta), 2);
    Serial.print(",");
    Serial.println(normalizar(5, pinzaCerrada), 2);
  }

  else if (dato.startsWith("SPEED,")) {
    int v;
    char extra;
    if (sscanf(dato.c_str(), "SPEED,%d%c", &v, &extra) != 1 ||
        v < 1 || v > 100) {
      Serial.println("ERROR,SPEED");
      return;
    }
    velocidad = v;
    publicarVelocidad();
    Serial.println("LISTO,SPEED");
  }

  else if (dato == "DIST" || dato == "DETECT") {
    float d = distancia();
    if (dato == "DIST") Serial.print("DIST,");
    else {
      Serial.print("DETECT,");
      Serial.print((d >= 5 && d <= 20) ? 1 : 0);
      Serial.print(",");
    }
    if (d < 0) Serial.println("NA");
    else Serial.println(d, 2);
    publicarDistancia(d);
  }

  // SET,1,60 -> Servo 1 al nuevo 60%
  else if (dato.startsWith("SET,")) {
    int numero;
    float p;
    char extra;
    int n = sscanf(dato.c_str(), "SET,%d,%f%c", &numero, &p, &extra);
    if (bloqueado || n != 2 || !fijarServo(numero, p)) {
      Serial.println("ERROR,SET");
      return;
    }
    actualizarSliders();
    Serial.println("LISTO,SET");
  }

  // ALL,60,44.87,62.5,58.82,55.56,46.15
  else if (dato.startsWith("ALL,")) {
    float p[6];
    char extra;
    int n = sscanf(dato.c_str(), "ALL,%f,%f,%f,%f,%f,%f%c",
                   &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &extra);
    if (bloqueado || n != 6) {
      Serial.println("ERROR,ALL");
      return;
    }
    for (int i = 0; i < 6; i++) {
      if (!isfinite(p[i]) || p[i] < 0 || p[i] > 100) {
        Serial.println("ERROR,LIMITE");
        return;
      }
    }
    for (int i = 0; i < 6; i++) fijarServo(i + 1, p[i]);
    actualizarSliders();
    Serial.println("LISTO,ALL");
  }

  // Python envia XYZ calculado mediante D-H
  else if (dato.startsWith("XYZ,")) {
    float x, y, z;
    char extra;
    int n = sscanf(dato.c_str(), "XYZ,%f,%f,%f%c", &x, &y, &z, &extra);
    if (n != 3 || !isfinite(x) || !isfinite(y) || !isfinite(z)) {
      Serial.println("ERROR,XYZ");
      return;
    }
    publicarXYZ(x, y, z);
    Serial.println("LISTO,XYZ");
  }

  // REPORT,TOMANDO A  -> texto que se muestra en la app
  else if (dato.startsWith("REPORT,")) {
    String estado = dato.substring(7);
    estado.trim();
    if (estado == "TERMINADA" || estado == "DETENIDA" || estado == "ERROR") {
      rutinaActiva = false;
      esperandoPC = false;
    } else if (!bloqueado) {
      rutinaActiva = true;            // Python esta ejecutando: bloquear sliders
    }
    publicarEstado(bloqueado ? "DETENIDO" : estado.c_str());
    Serial.println("LISTO,REPORT");
  }

  else {
    Serial.println("ERROR,COMMAND");
  }
}

// Pedidos que llegaron desde Blynk (se atienden aqui, en el nucleo del brazo)
void atenderPedidos() {
  if (pedidoStop) {
    pedidoStop = false;
    detener();
    Serial.println("STOP_REQUEST");
  }
  if (pedidoReanudar) {
    pedidoReanudar = false;
    bloqueado = false;
    publicarEstado("CONECTADO");
  }
  if (pedidoGuardarPinza != 0) {
    float actualPinza = destino[5];
    if (pedidoGuardarPinza == 1) guardarPinza(actualPinza, pinzaCerrada);
    else guardarPinza(pinzaAbierta, actualPinza);
    publicarEstado(pedidoGuardarPinza == 1 ? "ABIERTA GUARDADA" : "CERRADA GUARDADA");
    pedidoGuardarPinza = 0;
  }
  int r = pedidoRutina;
  if (r != 0) {
    pedidoRutina = 0;
    if (!rutinaActiva && !bloqueado) {
      if (r == 4) {
        iniciarApiladoBlynk();
      } else {
        rutinaActiva = true;
        esperandoPC = true;
        ultimoSerial = millis();
        Serial.print("RUTINA,");
        Serial.println(r);
        publicarEstado("ESPERANDO PC");
      }
    }
  }
  // Vigilancia: el PC no contesto -> devolver el control a la app
  if (esperandoPC && rutinaActiva && millis() - ultimoSerial > VIGILANCIA_MS) {
    esperandoPC = false;
    rutinaActiva = false;
    publicarEstado("PC SIN RESPUESTA");
  }
}


// ==================================================
// 5. PROGRAMA PRINCIPAL (nucleo 1)
// ==================================================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(30);

  memoria.begin("brazo", false);
  pinzaAbierta = memoria.getFloat("abierta", pinzaAbierta);
  pinzaCerrada = memoria.getFloat("cerrada", pinzaCerrada);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Wire.begin(SDA_PCA, SCL_PCA);

  if (!pca.begin()) {
    Serial.println("ERROR,PCA");
    while (true) delay(1000);
  }
  pca.setPWMFreq(50);
  delay(10);

  // Todos al antiguo 50% (postura de referencia)
  for (int i = 0; i < 6; i++) escribirServo(i, actual[i]);

  iniciarBlynk();
  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String dato = Serial.readStringUntil('\n');
    dato.trim();
    if (dato.length() > 0) comando(dato);
  }
  mover();
  atenderPedidos();
  atenderApiladoBlynk();

  if (hayInfo) {                       // mensajes de WiFi/Blynk para el Monitor serie
    char copia[200];
    portENTER_CRITICAL(&candado);
    strcpy(copia, infoSerial);
    hayInfo = false;
    portEXIT_CRITICAL(&candado);
    Serial.println(copia);
  }

  // Distancia para la app cada 1 s (solo con el brazo quieto: no frena el movimiento)
  if (millis() - ultimoSensor >= 1000 && !enMovimiento() && !rutinaActiva) {
    ultimoSensor = millis();
    publicarDistancia(lecturaUnica());
  }
}


// ==================================================
// 6. BLYNK (tarea separada en el nucleo 0)
// ==================================================

// --- publicaciones: se llaman desde el loop, solo dejan el dato pendiente
void publicarEstado(const char* texto) {
  portENTER_CRITICAL(&candado);
  strncpy(pend.texto, texto, sizeof(pend.texto) - 1);
  pend.texto[sizeof(pend.texto) - 1] = '\0';
  pend.estado = true;
  portEXIT_CRITICAL(&candado);
}

void publicarXYZ(float x, float y, float z) {
  portENTER_CRITICAL(&candado);
  pend.x = x; pend.y = y; pend.z = z;
  pend.xyz = true;
  portEXIT_CRITICAL(&candado);
}

void actualizarSliders() {
  portENTER_CRITICAL(&candado);
  pend.sliders = true;
  portEXIT_CRITICAL(&candado);
}

void publicarInterruptor() {
  portENTER_CRITICAL(&candado);
  pend.interruptor = true;
  portEXIT_CRITICAL(&candado);
}

void publicarVelocidad() {
  portENTER_CRITICAL(&candado);
  pend.vel = true;
  portEXIT_CRITICAL(&candado);
}

void publicarDistancia(float d) {
  portENTER_CRITICAL(&candado);
  pend.d = d;
  pend.dist = true;
  portEXIT_CRITICAL(&candado);
}

// --- envio real (solo desde la tarea Blynk)
void enviarPendientes() {
  Publicaciones p;
  portENTER_CRITICAL(&candado);
  p = pend;
  pend.estado = pend.xyz = pend.sliders = false;
  pend.interruptor = pend.vel = pend.dist = false;
  portEXIT_CRITICAL(&candado);

  if (p.estado) Blynk.virtualWrite(V11, p.texto);
  if (p.xyz) {
    Blynk.virtualWrite(V13, p.x);
    Blynk.virtualWrite(V14, p.y);
    Blynk.virtualWrite(V15, p.z);
  }
  if (p.sliders) {
    for (int i = 0; i < 5; i++) Blynk.virtualWrite(i, normalizar(i, destino[i]));
    Blynk.virtualWrite(V16, normalizar(5, destino[5]));
  }
  if (p.interruptor) Blynk.virtualWrite(V10, bloqueado ? 1 : 0);
  if (p.vel) Blynk.virtualWrite(V12, (int)velocidad);
  if (p.dist) Blynk.virtualWrite(V7, p.d);
}

void tareaBlynk(void* parametro) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  for (int i = 0; i < N_REDES; i++) {
    if (strlen(REDES[i][0]) > 0) redes.addAP(REDES[i][0], REDES[i][1]);
  }
  Blynk.config(BLYNK_AUTH_TOKEN);
  unsigned long ultimoIntento = 0;
  unsigned long ultimoAviso = 0;
  bool wifiAntes = false, blynkAntes = false;
  int fallosBlynk = 0;
  char txt[200];

  for (;;) {
    if (pedidoEscaneo) {                       // comando REDES
      pedidoEscaneo = false;
      int n = WiFi.scanNetworks();
      int largo = snprintf(txt, sizeof(txt), "INFO,REDES 2.4GHz (%d):", n < 0 ? 0 : n);
      for (int i = 0; i < n && largo < (int)sizeof(txt) - 40; i++) {
        largo += snprintf(txt + largo, sizeof(txt) - largo, " %s(%d)",
                          WiFi.SSID(i).c_str(), (int)WiFi.RSSI(i));
      }
      WiFi.scanDelete();
      avisarSerial(txt);
    }

    // run(): si ya hay conexion vuelve al tiro; si no, busca una de las redes
    bool wifi = redes.run(5000) == WL_CONNECTED;
    if (wifi && !wifiAntes) {
      snprintf(txt, sizeof(txt), "INFO,WIFI conectado a %s (%d dBm)",
               WiFi.SSID().c_str(), (int)WiFi.RSSI());
      avisarSerial(txt);
    }
    if (!wifi && millis() - ultimoAviso >= 15000) {
      ultimoAviso = millis();
      avisarSerial("INFO,WIFI no encuentra ninguna red guardada (2.4 GHz?, clave?). "
                   "Escriban REDES para ver las que hay");
    }
    wifiAntes = wifi;

    if (wifi) {
      if (Blynk.connected()) {
        if (!blynkAntes) avisarSerial("INFO,BLYNK conectado");
        fallosBlynk = 0;
        Blynk.run();
        enviarPendientes();
      } else if (millis() - ultimoIntento >= 5000) {
        ultimoIntento = millis();
        if (!Blynk.connect(3000) && ++fallosBlynk == 3) {   // puede tardar: no frena al brazo
          avisarSerial("INFO,BLYNK no conecta: la red no tiene internet o el token no es valido");
        }
      }
    }
    blynkAntes = wifi && Blynk.connected();
    vTaskDelay(pdMS_TO_TICKS(5)); // cede el nucleo (evita el watchdog)
  }
}

void iniciarBlynk() {
  xTaskCreatePinnedToCore(tareaBlynk, "blynk", 8192, NULL, 1, NULL, 0);
}

// --- entradas desde la app (corren en la tarea Blynk)
void controlManual(int servo, float porcentaje) {
  if (rutinaActiva || bloqueado) return;
  if (!fijarServo(servo, porcentaje)) Blynk.virtualWrite(V11, "LIMITE");
}

BLYNK_WRITE(V0) { controlManual(1, param.asFloat()); }
BLYNK_WRITE(V1) { controlManual(2, param.asFloat()); }
BLYNK_WRITE(V2) { controlManual(3, param.asFloat()); }
BLYNK_WRITE(V3) { controlManual(4, param.asFloat()); }
BLYNK_WRITE(V4) { controlManual(5, param.asFloat()); }

BLYNK_WRITE(V5) {
  if (param.asInt() == 1 && !rutinaActiva && !bloqueado)
    destino[5] = pinzaAbierta;
  actualizarSliders();
}

BLYNK_WRITE(V6) {
  if (param.asInt() == 1 && !rutinaActiva && !bloqueado)
    destino[5] = pinzaCerrada;
  actualizarSliders();
}

// V8: menu de Blynk. La app manda 0, 1, 2, 3 (posicion del item):
//   0 = Apilar, 1 = Ordenar, 2 = Escanear, 3 = Apilado ensenado -> rutina 1..4
// V16: slider de la pinza (0-100 %), para probar cuanto abre/cierra
BLYNK_WRITE(V16) { controlManual(6, param.asFloat()); }

// V17 / V18: botones "guardar abierta" / "guardar cerrada" (la posicion actual del V16)
BLYNK_WRITE(V17) { if (param.asInt() == 1) pedidoGuardarPinza = 1; }
BLYNK_WRITE(V18) { if (param.asInt() == 1) pedidoGuardarPinza = 2; }

BLYNK_WRITE(V8) { rutina = constrain(param.asInt(), 0, 3) + 1; }

// V9: boton "iniciar rutina". La rutina 4 corre en el ESP32; 1..3 conservan
// compatibilidad con el programa Python por USB.
BLYNK_WRITE(V9) {
  if (param.asInt() == 1 && rutina >= 1 && rutina <= 4 &&
      !rutinaActiva && !bloqueado) {
    pedidoRutina = rutina;
  }
}

// V10: interruptor (1 = detener, 0 = reanudar)
BLYNK_WRITE(V10) {
  if (param.asInt() == 1) pedidoStop = true;
  else pedidoReanudar = true;
}

BLYNK_WRITE(V12) { velocidad = constrain(param.asInt(), 1, 100); }

BLYNK_CONNECTED() {
  // Recuperar la rutina elegida y la velocidad guardadas en Blynk. No se debe
  // escribir V12 inmediatamente después: eso pisaría el valor del servidor
  // antes de que BLYNK_WRITE(V12) alcance a restaurarlo.
  Blynk.syncVirtual(V8, V12);
  for (int i = 0; i < 5; i++) Blynk.virtualWrite(i, normalizar(i, destino[i]));
  Blynk.virtualWrite(V16, normalizar(5, destino[5]));
  Blynk.virtualWrite(V10, bloqueado ? 1 : 0);
  Blynk.virtualWrite(V11, bloqueado ? "DETENIDO" :
                          rutinaActiva ? "EJECUTANDO" : "CONECTADO");
}
