#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <arduinoFFT.h>
#include "esp_sleep.h"
#include <heltec.h>
#include "LoRaWan_APP.h"
#include <time.h>
#include <sys/time.h>

// ***********************
// Global Variables for NTP Sync Reference
// ***********************
time_t g_ntpEpoch = 0;           // Epoch time (seconds) at last sync
unsigned long g_syncMillis = 0;  // The value of millis() at sync

// ***********************
// Static network configuration
// ***********************
IPAddress local_IP(192, 168, 137, 100);
IPAddress gateway(192, 168, 137, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   // Google's primary DNS
IPAddress secondaryDNS(8, 8, 4, 4); // Google's secondary DNS

// ***********************
// NTP Server Settings
// ***********************
const char* ntpServer = "time.google.com";
const long  gmtOffset_sec = 3600;       // Adjust as needed (+1 hour)
const int   daylightOffset_sec = 3600;  // Additional offset for DST

// ***********************
// LoRaWAN Parameters
// ***********************
/* OTAA parameters (replace with your actual values) */
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xFA, 0xCE };
uint8_t appEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x03, 0xD0, 0x00 };
uint8_t appKey[] = { 0x5D, 0x81, 0x15, 0x42, 0xC7, 0x32, 0x82, 0xC6, 0x3E, 0x36, 0x8C, 0x65, 0x1C, 0x9E, 0x37, 0x22 };

/* ABP parameters (if using ABP, not used here) */
uint8_t nwkSKey[] = { 0x15, 0xB1, 0xD0, 0xEF, 0xA4, 0x63, 0xDF, 0xBE, 0x3D, 0x11, 0x18, 0x1E, 0x1E, 0xC7, 0xDA, 0x85 };
uint8_t appSKey[] = { 0xD7, 0x2C, 0x78, 0x75, 0x8C, 0xDC, 0xCA, 0xBF, 0x55, 0xEE, 0x4A, 0x77, 0x8D, 0x16, 0xEF, 0x67 };
uint32_t devAddr = (uint32_t)0x007e6ae1;

/* Channels mask: default channels 0-7 */
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/* LoRaWAN region and class */
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868;
DeviceClass_t loraWanClass = CLASS_A;

/* Application transmission duty cycle (ms) */
uint32_t appTxDutyCycle = 15000;

/* OTAA or ABP */
bool overTheAirActivation = true;

/* ADR enable */
bool loraWanAdr = true;

/* Confirmed transmission flag */
bool isTxConfirmed = true;

/* Number of confirmed transmission trials */
uint8_t confirmedNbTrials = 4;

/* Application port */
uint8_t appPort = 2;

// ***********************
// WiFi & MQTT Parameters
// ***********************
const char *ssid_wifi = "ali";
const char *password_wifi = "12345678";
const char *mqtt_server = "192.168.1.5"; // Your Mosquitto broker's IP
const int mqtt_port = 1883;
const char *mqtt_topic = "test/esp32";
const char *client_id = "esp32-local-client";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// ***********************
// Sampling & Aggregation Parameters
// ***********************
#define WINDOW_SIZE         128            // Must be a power of 2
#define MAX_SAMPLING_FREQ   1e6            // Upper bound for adaptation (not used directly)
#define TASK_DELAY_MS       5000           // Delay between adaptation iterations (ms)
#define ADAPT_ITERATIONS    4              // Number of adaptation iterations
#define AGGREGATE_WINDOW_MS 10000          // Aggregation window duration (10 seconds)

// ***********************
// RTC Memory Variables (Preserved Across Deep Sleep)
// ***********************
RTC_DATA_ATTR bool freqLocked = false;              
RTC_DATA_ATTR double lockedAdaptiveFreq = MAX_SAMPLING_FREQ;   

// ***********************
// Global Variables for Adaptive Sampling
// ***********************
double currentSamplingFreq = MAX_SAMPLING_FREQ;  
double vReal[WINDOW_SIZE];
double vImag[WINDOW_SIZE];
ArduinoFFT<double> FFT(vReal, vImag, WINDOW_SIZE, MAX_SAMPLING_FREQ, true);

// ***********************
// Bonus: Input Signal Selection
// ***********************
enum SignalType {
  SIGNAL_LOW_FREQ,   
  SIGNAL_MIXED,      
  SIGNAL_HIGH_FREQ   
};

SignalType currentSignal = SIGNAL_MIXED;  // Set desired signal type

double inputSignal(double t) {
  switch (currentSignal) {
    case SIGNAL_LOW_FREQ:
      return 1.0 + 2 * sin(2 * PI * 2 * t);
    case SIGNAL_HIGH_FREQ:
      return 1.0 + 1 * sin(2 * PI * 40 * t);
    case SIGNAL_MIXED:
    default:
      return 1.0 + 2 * sin(2 * PI * 3 * t) + 4 * sin(2 * PI * 30 * t);
  }
}

// ***********************
// MQTT Callback Function
// ***********************
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("MQTT Message arrived on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// ***********************
// WiFi & MQTT Reconnect Functions
// ***********************
void connectWiFi() {
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
    Serial.println("Failed to configure static IP");
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid_wifi, password_wifi);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP:");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(client_id))
      Serial.println(" connected!");
    else {
      Serial.print("MQTT connect failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

// ***********************
// Measure Maximum Sampling Frequency Function
// ***********************
int measureMaxSamplingFrequency() {
  unsigned long startTime = micros();
  int sampleCount = 0;
  while (micros() - startTime < 1000000) {
    volatile double val = inputSignal((micros() - startTime) / 1000000.0);
    sampleCount++;
  }
  Serial.print("Approx. Maximum Sampling Frequency measured: ");
  Serial.print(sampleCount);
  Serial.println(" Hz");
  return sampleCount;
}

// ***********************
// Sample the Signal Function
// ***********************
void sampleSignal(double samplingFreq, int samples) {
  unsigned long samplingPeriodUs = 1000000 / samplingFreq;
  unsigned long t0 = micros();
  for (int i = 0; i < samples; i++) {
    unsigned long t = micros();
    double timeElapsed = (t - t0) / 1000000.0;
    vReal[i] = inputSignal(timeElapsed);
    vImag[i] = 0;
    while (micros() - t < samplingPeriodUs) {
      vTaskDelay(1);
    }
  }
}

// ***********************
// Compute FFT and Adjust Sampling Frequency Function
// ***********************
void computeFFTandAdjustSampling() {
  sampleSignal(currentSamplingFreq, WINDOW_SIZE);
  FFT.windowing(vReal, WINDOW_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, WINDOW_SIZE, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, WINDOW_SIZE);
  double frequencyResolution = currentSamplingFreq / double(WINDOW_SIZE);
  double maxMagnitude = 0;
  int maxIndex = 0;
  for (int i = 1; i < WINDOW_SIZE / 2; i++) {
    if (vReal[i] > maxMagnitude) {
      maxMagnitude = vReal[i];
      maxIndex = i;
    }
  }
  double maxFrequency = maxIndex * frequencyResolution;
  Serial.print("Maximum frequency component detected: ");
  Serial.print(maxFrequency, 2);
  Serial.println(" Hz");
  double optimalSamplingFrequency = 2 * maxFrequency;
  Serial.print("Optimal sampling frequency (Nyquist): ");
  Serial.print(optimalSamplingFrequency, 2);
  Serial.println(" Hz");
  if (optimalSamplingFrequency < currentSamplingFreq)
    currentSamplingFreq = optimalSamplingFrequency;
  else
    currentSamplingFreq = MAX_SAMPLING_FREQ;
  Serial.print("Adaptive Sampling Frequency set to: ");
  Serial.print(currentSamplingFreq, 2);
  Serial.println(" Hz");
}

// ***********************
// LoRaWAN Uplink: Prepare Payload Function
// ***********************
void prepareTxFrame(uint8_t port, float aggValue) {
  appDataSize = 4;
  memcpy(appData, &aggValue, sizeof(aggValue));
  Serial.print("LoRaWAN payload prepared on port ");
  Serial.print(port);
  Serial.print(": ");
  for (int i = 0; i < appDataSize; i++) {
    Serial.print(appData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// ***********************
// FreeRTOS Task: Aggregate Sensor Readings Over 10 Seconds & Transmit via MQTT and LoRaWAN
// ***********************
void taskAggregate(void * parameter) {
  while (1) {
    unsigned long aggStartTime = millis();
    double sum = 0;
    unsigned long count = 0;
    while (millis() - aggStartTime < AGGREGATE_WINDOW_MS) {
      double t = (millis() - aggStartTime) / 1000.0;
      double sampleVal = inputSignal(t);
      sum += sampleVal;
      count++;
      unsigned long samplingPeriodUs = 1000000 / currentSamplingFreq;
      unsigned long samplingPeriodMs = samplingPeriodUs / 1000;
      vTaskDelay(samplingPeriodMs / portTICK_PERIOD_MS);
    }
    double average = (count > 0) ? (sum / count) : 0;
    Serial.print("Aggregate (average) over 10 seconds: ");
    Serial.println(average, 4);
    
    // Capture sendMillis immediately before publishing.
    unsigned long sendMillis = millis();
    // Compute current time from the stored NTP sync reference.
    time_t currentEpoch = g_ntpEpoch + ((sendMillis - g_syncMillis) / 1000);
    struct tm currentTime;
    localtime_r(&currentEpoch, &currentTime);
    char timeStr[64];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &currentTime);
    unsigned long msecPart = (sendMillis - g_syncMillis) % 1000;
    Serial.printf("ESP sending time (calculated at publish): %s.%03lu\n", timeStr, msecPart);
    
    // Publish only the aggregated average value (as a string)
    if (!client.connected())
      reconnectMQTT();
    char payload[50];
    snprintf(payload, sizeof(payload), "%.4f", average);
    client.publish(mqtt_topic, payload);
    Serial.print("Published MQTT aggregate value: ");
    Serial.println(payload);
    
    // Prepare LoRaWAN payload with the aggregate value.
    prepareTxFrame(appPort, (float)average);
    deviceState = DEVICE_STATE_SEND;  // Trigger LoRaWAN sending.
    
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// ***********************
// FreeRTOS Task: Continuous Sampling (Optional)
// ***********************
void taskContinuousSampling(void * parameter) {
  while (1) {
    sampleSignal(currentSamplingFreq, WINDOW_SIZE);
    Serial.print("First sample value: ");
    Serial.println(vReal[0], 4);
    vTaskDelay(TASK_DELAY_MS / portTICK_PERIOD_MS);
  }
}

// ***********************
// LoRaWAN State Machine (Based on Heltec LoRaWan_APP Library)
// ***********************
void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
    {
#if (LORAWAN_DEVEUI_AUTO)
      LoRaWAN.generateDeveuiByChipID();
#endif
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(3);
      deviceState = DEVICE_STATE_JOIN;
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep(loraWanClass);
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
  client.loop();  // Maintain MQTT connection.
}

// ***********************
// setup() Function
// ***********************
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== Adaptive IoT Sampling with Dual Transmission ===");
  
  // Adaptive Phase:
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    measureMaxSamplingFrequency();
    for (int i = 0; i < ADAPT_ITERATIONS; i++) {
      computeFFTandAdjustSampling();
      delay(TASK_DELAY_MS);
    }
    lockedAdaptiveFreq = currentSamplingFreq;
    freqLocked = true;
    Serial.print("Locked adaptive sampling frequency: ");
    Serial.print(lockedAdaptiveFreq, 2);
    Serial.println(" Hz");
    Serial.println("Adaptation complete. Entering deep sleep for 1 second...");
    esp_deep_sleep(1000000);  // Deep sleep for 1 second
  }
  else {
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
      Serial.println("Failed to configure static IP");
    connectWiFi();
    
    // Wait until time is synchronized via NTP.
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeInfo;
    Serial.println("Waiting for NTP time sync...");
    while (!getLocalTime(&timeInfo)) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nNTP time synchronized.");
    
    // Store the synchronized epoch time and current millis() for later calculations.
    struct timeval tv;
    gettimeofday(&tv, NULL);
    g_ntpEpoch = tv.tv_sec;
    g_syncMillis = millis();
    Serial.print("System epoch time: ");
    Serial.println(g_ntpEpoch);
    
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    
    Serial.print("Using previously locked adaptive sampling frequency: ");
    Serial.print(lockedAdaptiveFreq, 2);
    Serial.println(" Hz");
    currentSamplingFreq = lockedAdaptiveFreq;
  }
  
  // Initialize LoRaWAN using Heltec's library.
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  
  // Create tasks.
  xTaskCreate(taskAggregate, "AggregateTask", 4096, NULL, 1, NULL);
  xTaskCreate(taskContinuousSampling, "ContinuousSampling", 4096, NULL, 1, NULL);
}
