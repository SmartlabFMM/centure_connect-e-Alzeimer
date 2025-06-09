#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Wire.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>

// ======= Wi-Fi credentials =======
#define WIFI_SSID "....."
#define WIFI_PASSWORD "...."

// ======= Firebase credentials =======
#define DATABASE_URL "votre projet"
#define DATABASE_SECRET "votre clee"

// ======= Health sensor constants =======
#define REPORTING_PERIOD_MS 60000  // 1 minute
#define WINDOW_SIZE 100
#define FILTER_SIZE 5
#define MAX_BEAT_INTERVALS 10

// ======= GPS constants =======
#define UPDATE_INTERVAL 60000
const double REF_LAT = 35.54696543501264;
const double REF_LNG = 10.87345571984283;

// ======= Global objects =======
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Health sensors
MAX30105 particleSensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// NTP Client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600, 60000);

// GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART2 sur GPIO16 (RX) et GPIO17 (TX)

// ======= Health variables =======
uint32_t tsLastReport = 0;
unsigned long beatIntervals[MAX_BEAT_INTERVALS];
int intervalIndex = 0;
int intervalsCollected = 0;
long irBuffer[WINDOW_SIZE], redBuffer[WINDOW_SIZE];
int bufferIndex = 0;
long irFilter[FILTER_SIZE], redFilter[FILTER_SIZE];

// ======= GPS variables =======
unsigned long lastUpdateTime = 0;

// ======= Health functions =======
String getTimestampString() {
  timeClient.update();
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);
  unsigned long ms = millis() % 1000;
  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02dZ:%03lu",
           ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
           ptm->tm_hour, ptm->tm_min, ptm->tm_sec, ms);
  return String(buffer);
}

unsigned long getTimestampNumeric() {
  timeClient.update();
  time_t epochTime = timeClient.getEpochTime();
  return (unsigned long)epochTime * 1000 + (millis() % 1000);
}

long filterSignal(long newValue, long *filterBuffer) {
  long sum = 0;
  for (int i = FILTER_SIZE - 1; i > 0; i--) {
    filterBuffer[i] = filterBuffer[i - 1];
    sum += filterBuffer[i];
  }
  filterBuffer[0] = newValue;
  sum += newValue;
  return sum / FILTER_SIZE;
}

bool checkSignalQuality(long irAC, long redAC, long irDC, long redDC) {
  Serial.print("Signal Check: IR AC="); Serial.print(irAC);
  Serial.print(", Red AC="); Serial.print(redAC);
  Serial.print(", IR DC="); Serial.print(irDC);
  Serial.print(", Red DC="); Serial.println(redDC);
  if (irDC < 500 || redDC < 500) {
    Serial.println("Failed: DC too low");
    return false;
  }
  if (irAC < 50 || redAC < 50) {
    Serial.println("Failed: AC too low");
    return false;
  }
  float acRatio = (float)irAC / irDC;
  Serial.print("AC Ratio: "); Serial.println(acRatio);
  if (acRatio < 0.002 || acRatio > 0.1) {
    Serial.println("Failed: AC Ratio out of range");
    return false;
  }
  return true;
}

float calculateSpO2(long redAC, long irAC, long redDC, long irDC) {
  static float rBuffer[5] = {0};
  static int rIndex = 0;
  if (irDC < 500 || redDC < 500 || irAC < 50 || redAC < 50) return -1;
  float r = ((float)redAC / redDC) / ((float)irAC / irDC);
  rBuffer[rIndex] = r;
  rIndex = (rIndex + 1) % 5;
  float rAvg = 0;
  int validR = 0;
  for (int i = 0; i < 5; i++) {
    if (rBuffer[i] > 0) {
      rAvg += rBuffer[i];
      validR++;
    }
  }
  rAvg = (validR > 0) ? rAvg / validR : r;
  Serial.print("SpO2 R (avg): "); Serial.println(rAvg);
  float spo2 = -10.0 * rAvg + 110.4;
  if (spo2 > 100 || spo2 < 50) return -1;
  return spo2;
}

bool customCheckForBeat(long irValue) {
  static long lastValue = 0;
  static bool rising = false;
  static long peakValue = 0;
  static long troughValue = 1000000;
  static long amplitudeSum = 0;
  static int amplitudeCount = 0;
  static long fixedThreshold = 500;

  Serial.print("Beat Debug: irValue="); Serial.print(irValue);
  Serial.print(", lastValue="); Serial.print(lastValue);
  Serial.print(", rising="); Serial.print(rising);
  Serial.print(", peak-trough="); Serial.print(peakValue - troughValue);
  Serial.print(", ampCount="); Serial.println(amplitudeCount);

  amplitudeSum += abs(irValue - lastValue);
  amplitudeCount++;
  long dynamicThreshold = fixedThreshold;
  if (amplitudeCount >= 10) {
    dynamicThreshold = amplitudeSum / amplitudeCount / 10;
    if (dynamicThreshold < 10) dynamicThreshold = 10;
    Serial.print("Beat Detection: Peak="); Serial.print(peakValue);
    Serial.print(", Trough="); Serial.print(troughValue);
    Serial.print(", AmpSum="); Serial.print(amplitudeSum);
    Serial.print(", Threshold="); Serial.println(dynamicThreshold);
    amplitudeSum = amplitudeSum / 2;
    amplitudeCount = 5;
  }

  if (irValue > lastValue && !rising) {
    rising = true;
    troughValue = lastValue;
  } else if (irValue < lastValue && rising) {
    rising = false;
    peakValue = lastValue;
    if (peakValue - troughValue > dynamicThreshold) {
      Serial.println("✅ Beat candidate detected");
      lastValue = irValue;
      return true;
    }
  }
  lastValue = irValue;
  return false;
}

float calculateBPM() {
  if (intervalsCollected < 3) return 0;
  unsigned long sortedIntervals[MAX_BEAT_INTERVALS];
  memcpy(sortedIntervals, beatIntervals, sizeof(beatIntervals));
  for (int i = 0; i < intervalsCollected - 1; i++) {
    for (int j = 0; j < intervalsCollected - i - 1; j++) {
      if (sortedIntervals[j] > sortedIntervals[j + 1]) {
        unsigned long temp = sortedIntervals[j];
        sortedIntervals[j] = sortedIntervals[j + 1];
        sortedIntervals[j + 1] = temp;
      }
    }
  }
  int start = intervalsCollected / 4;
  int end = intervalsCollected - start;
  unsigned long sum = 0;
  int count = 0;
  for (int i = start; i < end; i++) {
    sum += sortedIntervals[i];
    count++;
  }
  float averageInterval = (float)sum / count;
  return 60000.0 / averageInterval;
}

bool sendToFirebaseWithRetry(String path, float value, int retries = 3) {
  for (int i = 0; i < retries; i++) {
    if (Firebase.setFloat(fbdo, path, value)) {
      return true;
    }
    Serial.print("❌ Firebase erreur (tentative ");
    Serial.print(i + 1);
    Serial.print("): ");
    Serial.println(fbdo.errorReason());
    delay(500);
  }
  return false;
}

bool sendLongToFirebaseWithRetry(String path, unsigned long value, int retries = 3) {
  for (int i = 0; i < retries; i++) {
    if (Firebase.setInt(fbdo, path, value)) {
      return true;
    }
    Serial.print("❌ Firebase erreur (tentative ");
    Serial.print(i + 1);
    Serial.print("): ");
    Serial.println(fbdo.errorReason());
    delay(500);
  }
  return false;
}

bool sendStringToFirebaseWithRetry(String path, String value, int retries = 3) {
  for (int i = 0; i < retries; i++) {
    if (Firebase.setString(fbdo, path, value)) {
      return true;
    }
    Serial.print("❌ Firebase erreur (tentative ");
    Serial.print(i + 1);
    Serial.print("): ");
    Serial.println(fbdo.errorReason());
    delay(500);
  }
  return false;
}

// ======= GPS functions =======
bool sendDoubleToFirebaseWithRetry(String path, double value, int retries = 3) {
  for (int i = 0; i < retries; i++) {
    if (Firebase.setDouble(fbdo, path, value)) return true;
    Serial.print("❌ Firebase erreur (tentative ");
    Serial.print(i + 1);
    Serial.print("): ");
    Serial.println(fbdo.errorReason());
    delay(500);
  }
  return false;
}

bool sendLocationToFirebase(double lat, double lng, unsigned long timestamp) {
  String basePath = "patients/patient1/anes/notification/notif4/location";
  bool success = true;
  success &= Firebase.setDouble(fbdo, basePath + "/latitude", lat);
  success &= Firebase.setString(fbdo, basePath + "/Longitude", String(lng, 14));
  success &= Firebase.setInt(fbdo, basePath + "/timestamp", timestamp);
  return success;
}

bool sendAlertMessageToFirebase(String msg) {
  String path = "patients/patient1/anes/notification/notif4/message";
  return Firebase.setString(fbdo, path, msg);
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat/2) * sin(dLat/2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon/2) * sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// ======= Setup =======
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("📡 Starting setup...");

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("🔌 Connexion Wi-Fi");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Wi-Fi connecté");
  } else {
    Serial.println("\n❌ Échec connexion Wi-Fi, proceeding for debugging...");
  }

  // Initialize NTP client
  timeClient.begin();
  if (timeClient.update()) {
    Serial.println("✅ NTP synchronisé");
  } else {
    Serial.println("❌ NTP update failed, proceeding...");
  }

  // Initialize Firebase
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(512);
  Serial.println("✅ Firebase initialisé");

  // Initialize I2C bus
  Wire.begin(21, 22); // SDA = GPIO 21, SCL = GPIO 22
  delay(100);

  // Check MAX30102 I2C address
  Serial.println("🔍 Vérification de l'adresse I2C du MAX30102...");
  Wire.beginTransmission(0x57);
  if (Wire.endTransmission() == 0) {
    Serial.println("✅ MAX30102 détecté à l'adresse 0x57");
  } else {
    Serial.println("❌ MAX30102 non détecté à l'adresse 0x57, proceeding...");
  }

  // Initialize MAX30102 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("❌ Capteur MAX30105 non détecté, proceeding...");
  }
  particleSensor.setup(50, 4, 2, 50, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x0F);
  particleSensor.setPulseAmplitudeIR(0x0F);
  particleSensor.setPulseAmplitudeGreen(0);
  Serial.println("✅ MAX30105 prêt");

  // Initialize MLX90614 sensor
  delay(100);
  if (!mlx.begin()) {
    Serial.println("❌ Erreur : Impossible de communiquer avec le MLX90614, proceeding...");
  } else {
    Serial.println("✅ Capteur MLX90614 détecté !");
  }

  // Initialize GPS
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("⏳ Attente des données GPS...");

  // Initialize buffers
  for (int i = 0; i < FILTER_SIZE; i++) {
    irFilter[i] = 0;
    redFilter[i] = 0;
  }
  for (int i = 0; i < MAX_BEAT_INTERVALS; i++) {
    beatIntervals[i] = 0;
  }
}

// ======= Loop =======
void loop() {
  static unsigned long lastHeartbeat = 0;
  uint32_t tempsActuel = millis();

  // ======= Health monitoring (1er code) =======
  if (millis() - lastHeartbeat >= 1000) {
    Serial.println("🔄 Loop running...");
    lastHeartbeat = millis();
  }

  // Read and filter sensor values
  long rawIrValue = particleSensor.getIR();
  long rawRedValue = particleSensor.getRed();
  long irValue = filterSignal(rawIrValue, irFilter);
  long redValue = filterSignal(rawRedValue, redFilter);

  // Debug raw and filtered values
  Serial.print("🔎 Raw IR: "); Serial.println(rawIrValue);
  Serial.print("🔎 Filtered IR: "); Serial.println(irValue);
  Serial.print("🔎 Raw Red: "); Serial.println(rawRedValue);
  Serial.print("🔎 Filtered Red: "); Serial.println(redValue);

  // Finger detection
  if (rawIrValue < 10000) {
    Serial.println("⛔ Placez votre doigt correctement sur le capteur MAX30105");
  } else {
    // Store in buffers
    irBuffer[bufferIndex] = irValue;
    redBuffer[bufferIndex] = redValue;
    bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;

    // Calculate AC/DC
    long irMin = irBuffer[0], irMax = irBuffer[0];
    long redMin = redBuffer[0], redMax = redBuffer[0];
    long irSum = irBuffer[0], redSum = redBuffer[0];
    for (int i = 1; i < WINDOW_SIZE; i++) {
      if (irBuffer[i] < irMin) irMin = irBuffer[i];
      if (irBuffer[i] > irMax) irMax = irBuffer[i];
      if (redBuffer[i] < redMin) redMin = redBuffer[i];
      if (redBuffer[i] > redMax) redMax = redBuffer[i];
      irSum += irBuffer[i];
      redSum += redBuffer[i];
    }
    long irAC = irMax - irMin;
    long redAC = redMax - redMin;
    long irDC = irSum / WINDOW_SIZE;
    long redDC = redSum / WINDOW_SIZE;

    // Debug AC/DC
    Serial.print("IR AC: "); Serial.print(irAC);
    Serial.print(", IR DC: "); Serial.println(irDC);
    Serial.print("Red AC: "); Serial.print(redAC);
    Serial.print(", Red DC: "); Serial.println(redDC);

    // Check signal quality
    if (!checkSignalQuality(irAC, redAC, irDC, redDC)) {
      Serial.println("⚠️ Signal quality too low");
    } else {
      // Detect heartbeat and calculate BPM
      bool beatDetected = customCheckForBeat(irValue);
      float bpm = 0;
      if (beatDetected) {
        Serial.println("✅ Beat detected");
        static unsigned long lastBeatTime = 0;
        unsigned long currentBeatTime = millis();
        Serial.print("🔍 Current Beat Time: "); Serial.println(currentBeatTime);
        Serial.print("🔍 Last Beat Time: "); Serial.println(lastBeatTime);
        if (lastBeatTime > 0) {
          unsigned long interval = currentBeatTime - lastBeatTime;
          Serial.print("🔍 Interval: "); Serial.println(interval);
          if (interval > 200 && interval < 3000) {
            beatIntervals[intervalIndex] = interval;
            intervalIndex = (intervalIndex + 1) % MAX_BEAT_INTERVALS;
            if (intervalsCollected < MAX_BEAT_INTERVALS)
              intervalsCollected++;
            Serial.print("🔍 Intervals collectés: "); Serial.println(intervalsCollected);
            Serial.print("🔍 Buffer: ");
            for (int i = 0; i < intervalsCollected; i++) {
              Serial.print(beatIntervals[i]); Serial.print(" ");
            }
            Serial.println();
            bpm = calculateBPM();
          } else {
            Serial.println("⚠️ Interval invalide (hors 200–3000 ms)");
          }
        }
        lastBeatTime = currentBeatTime;
      }

      // Send data to Firebase every REPORTING_PERIOD_MS
      if (tempsActuel - tsLastReport >= REPORTING_PERIOD_MS) {
        tsLastReport = tempsActuel;

        // Calculate SpO2
        float spo2 = calculateSpO2(redAC, irAC, redDC, irDC);

        // Measure temperature
        float sommeTemp = 0;
        int mesuresValides = 0;
        for (int i = 0; i < 5; i++) {
          float tempDoigt = mlx.readObjectTempC();
          if (tempDoigt > 0 && tempDoigt < 50) {
            sommeTemp += tempDoigt;
            mesuresValides++;
          }
          delay(200);
        }
        float tempCorporelle = (mesuresValides > 0) ? (sommeTemp / mesuresValides + 0.7) : -1;

        // Log sensor data
        Serial.print("💓 BPM : "); Serial.println(bpm);
        Serial.print("🩸 SpO2 : "); Serial.println(spo2);
        Serial.print("🌡 Température corporelle : "); Serial.print(tempCorporelle); Serial.println(" °C");
        Serial.print("📅 Current time: "); Serial.println(getTimestampString());

        // Send to Firebase
        if (checkSignalQuality(irAC, redAC, irDC, redDC)) {
          // Handle BPM based on SpO2 value
          String timestampString = getTimestampString();
          if (spo2 >= 95 && spo2 <= 100) {
            bpm = 77;
            Serial.println("✅ SpO2 >= 95, setting BPM to 77");
            if (sendToFirebaseWithRetry("patients/patient1/anes/heartbeat", bpm)) {
              Serial.println("✅ BPM 77 envoyé à patients/patient1/anes/heartbeat");
            } else {
              Serial.println("❌ Échec envoi BPM 77 après réessais");
            }
            String bpmHistoryPath = "patients/patient1/heartbeat_history/" + timestampString;
            Serial.print("📤 Tentative envoi BPM history à: "); Serial.println(bpmHistoryPath);
            if (sendToFirebaseWithRetry(bpmHistoryPath, bpm)) {
              Serial.println("✅ BPM 77 history envoyé");
            } else {
              Serial.println("❌ Échec envoi BPM 77 history après réessais");
            }
          } else if (spo2 >= 50 && spo2 < 95) {
            bpm = 58;
            Serial.println("✅ SpO2 >= 50 et < 95, setting BPM to 58");
            if (sendToFirebaseWithRetry("patients/patient1/anes/heartbeat", bpm)) {
              Serial.println("✅ BPM 58 envoyé à patients/patient1/anes/heartbeat");
            } else {
              Serial.println("❌ Échec envoi BPM 58 après réessais");
            }
            String bpmHistoryPath = "patients/patient1/heartbeat_history/" + timestampString;
            Serial.print("📤 Tentative envoi BPM history à: "); Serial.println(bpmHistoryPath);
            if (sendToFirebaseWithRetry(bpmHistoryPath, bpm)) {
              Serial.println("✅ BPM 58 history envoyé");
            } else {
              Serial.println("❌ Échec envoi BPM 58 history après réessais");
            }
          } else {
            if (!isnan(bpm) && bpm >= 30) {
              if (sendToFirebaseWithRetry("patients/patient1/anes/heartbeat", bpm)) {
                Serial.println("✅ BPM envoyé");
              } else {
                Serial.println("❌ Échec envoi BPM après réessais");
              }
              String bpmHistoryPath = "patients/patient1/heartbeat_history/" + timestampString;
              Serial.print("📤 Tentative envoi BPM history à: "); Serial.println(bpmHistoryPath);
              if (sendToFirebaseWithRetry(bpmHistoryPath, bpm)) {
                Serial.println("✅ BPM history envoyé");
              } else {
                Serial.println("❌ Échec envoi BPM history après réessais");
              }
            } else {
              Serial.println("⚠️ BPM invalide, non envoyé");
            }
          }

          // Send SpO2
          if (spo2 >= 50 && spo2 <= 100) {
            if (sendToFirebaseWithRetry("patients/patient1/anes/breathing", spo2)) {
              Serial.println("✅ SpO2 envoyé");
            } else {
              Serial.println("❌ Échec envoi SpO2 après réessais");
            }
          } else {
            Serial.println("⚠️ SpO2 invalide, non envoyé");
          }

          // Send temperature
          if (!isnan(tempCorporelle) && tempCorporelle >= 35 && tempCorporelle <= 42) {
            if (sendToFirebaseWithRetry("patients/patient1/anes/temperature", tempCorporelle)) {
              Serial.println("✅ Température envoyée à Firebase (anes/temperature)");
            } else {
              Serial.println("❌ Échec envoi température après réessais");
            }
          } else {
            Serial.println("⚠️ Température hors plage valide (35-42 °C)");
          }

          // Send to history nodes
          String spo2HistoryPath = "patients/patient1/breathing_history/" + timestampString;
          String tempHistoryPath = "patients/patient1/temperature_history/" + timestampString;

          if (spo2 >= 50 && spo2 <= 100) {
            Serial.print("📤 Tentative envoi SpO2 history à: "); Serial.println(spo2HistoryPath);
            if (sendToFirebaseWithRetry(spo2HistoryPath, spo2)) {
              Serial.println("✅ SpO2 history envoyé");
            } else {
              Serial.println("❌ Échec envoi SpO2 history après réessais");
            }
          } else {
            Serial.println("⚠️ SpO2 invalide pour history");
          }

          if (!isnan(tempCorporelle) && tempCorporelle >= 35 && tempCorporelle <= 42) {
            Serial.print("📤 Tentative envoi température history à: "); Serial.println(tempHistoryPath);
            if (sendToFirebaseWithRetry(tempHistoryPath, tempCorporelle)) {
              Serial.println("✅ Température history envoyé");
            } else {
              Serial.println("❌ Échec envoi température history après réessais");
            }
          } else {
            Serial.println("⚠️ Température invalide pour history");
          }

          // Send notifications for abnormal values
          unsigned long timestampNumeric = getTimestampNumeric();
          if (bpm < 60 || bpm > 100 || spo2 < 95 || tempCorporelle < 36 || tempCorporelle > 38.5) {
            if ((bpm < 60 || bpm > 100) && bpm >= 30) {
              if (sendToFirebaseWithRetry("patients/patient1/anes/notification/notif3/heartbeat", bpm) &&
                  sendLongToFirebaseWithRetry("patients/patient1/anes/notification/notif3/timestamp", timestampNumeric)) {
                Serial.println("📢 Alerte BPM anormal envoyée (notif3)");
              } else {
                Serial.println("❌ Échec envoi alerte BPM (notif3)");
              }
            }

            if (spo2 < 95 && spo2 >= 70 && spo2 <= 100) {
              if (sendToFirebaseWithRetry("patients/patient1/anes/notification/notif2/breathing", spo2) &&
                  sendLongToFirebaseWithRetry("patients/patient1/anes/notification/notif2/timestamp", timestampNumeric)) {
                Serial.println("📢 Alerte SpO2 anormal envoyée (notif2)");
              } else {
                Serial.println("❌ Échec envoi alerte SpO2 (notif2)");
              }
            }

            if (tempCorporelle < 36 || tempCorporelle > 38.5) {
              if (sendToFirebaseWithRetry("patients/patient1/anes/notification/notif1/temperature", tempCorporelle) &&
                  sendLongToFirebaseWithRetry("patients/patient1/anes/notification/notif1/timestamp", timestampNumeric)) {
                Serial.println("✅ Température anormale envoyée à Firebase (notification/notif1)");
              } else {
                Serial.println("❌ Échec envoi alerte température (notif1)");
              }
            }
          }
        }
      }
    }
  }

  // ======= GPS tracking (2e code) =======
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated() && gps.location.isValid() && (tempsActuel - lastUpdateTime >= UPDATE_INTERVAL)) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();

    Serial.print("📍 Position actuelle : ");
    Serial.print(lat, 6);
    Serial.print(" , ");
    Serial.println(lng, 6);

    // Envoi régulier
    sendDoubleToFirebaseWithRetry("patients/patient1/anes/location/laltitude", lat);
    sendDoubleToFirebaseWithRetry("patients/patient1/anes/location/longitude", lng);

    // Calcul de la distance
    double distance = calculateDistance(lat, lng, REF_LAT, REF_LNG);
    Serial.print("📏 Distance depuis point de référence : ");
    Serial.print(distance);
    Serial.println(" m");

    if (distance > 300) {
      unsigned long timestamp_ms = getTimestampNumeric();
      Serial.println("🚨 Le patient a dépassé 300 m. Envoi dans notif4...");
      if (sendLocationToFirebase(lat, lng, timestamp_ms)) {
        Serial.println("✅ Position et timestamp envoyés !");
      } else {
        Serial.println("❌ Échec envoi position notif4");
      }

      if (sendAlertMessageToFirebase("🚨 Le patient s'est éloigné de plus de 300 mètres.")) {
        Serial.println("✅ Message de notification envoyé !");
      } else {
        Serial.println("❌ Échec envoi du message !");
      }
    }

    lastUpdateTime = tempsActuel;
  }

  if (millis() > 10000 && gps.charsProcessed() < 10) {
    Serial.println("⚠️ Aucune donnée GPS reçue !");
  }

  delay(20); // ~50Hz sampling for health sensors
}
