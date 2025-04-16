# Adaptive IoT Sampling and Aggregation System

This repository implements an energy-efficient IoT system using an ESP32 board with FreeRTOS. The device collects sensor data, performs local FFT analysis to determine the dominant frequency, and then adapts its sampling frequency accordingly. It computes an average over a 10-second window and transmits this aggregated value via two channels: MQTT over WiFi to an edge server, and LoRaWAN (via TTN) to the cloud.

The primary goal is to conserve energy by reducing unnecessary high-frequency sampling while ensuring a constant and minimal communication load over the network.

---

## Project Overview

### Objectives

- **Efficient Signal Analysis:**  
  The system initially samples the sensor signal at a very high rate to perform Fast Fourier Transform (FFT) analysis. This process identifies the dominant frequencies present in the signal. Based on the Nyquist theorem, the device then computes an optimal sampling frequency (set to twice the maximum significant frequency), thereby avoiding oversampling and saving energy.

- **Adaptive Sampling & Power Management:**  
  After the FFT phase, the adaptive sampling frequency is “locked in” and the device switches from the high-frequency sampling mode to a lower, optimal frequency. It enters deep sleep mode between transmission intervals to significantly reduce energy consumption.

- **Data Aggregation and Consistent Payload:**  
  The system aggregates sensor readings over a fixed 10-second window. Regardless of whether the device is oversampling or using the adaptive sampling strategy, it transmits only one aggregated value. This results in a consistent 6-byte payload transmitted every 10 seconds, ensuring that network load is constant.

- **Dual Communication Channels:**  
  The aggregated result is sent over MQTT (using WiFi) to an edge server and via LoRaWAN to the cloud (using OTAA activation on TTN).

---

## Key System Components and Important Variables

### Network and Time Synchronization
- **NTP Sync Variables:**  
  - `g_ntpEpoch`: Epoch time at the last NTP sync.
  - `g_syncMillis`: The system uptime (millis) at synchronization.
- **IP Configuration:**  
  Variables such as `local_IP`, `gateway`, `subnet`, `primaryDNS`, and `secondaryDNS` ensure stable network connections.
- **NTP Server:**  
  The system uses `ntpServer` (e.g., "time.google.com") with offsets (`gmtOffset_sec` and `daylightOffset_sec`) for accurate time stamping.

### LoRaWAN and MQTT Configurations
- **LoRaWAN Settings:**  
  Parameters like `devEui`, `appEui`, `appKey`, and the channel mask ensure proper cloud communication via the LoRaWAN network.  
- **WiFi/MQTT Parameters:**  
  WiFi is set up using credentials (`ssid_wifi` and `password_wifi`), and the MQTT client is configured with `mqtt_server`, `mqtt_port`, `mqtt_topic`, and `client_id`.

### Sampling and FFT Analysis
- **Sampling Constants:**  
  - `WINDOW_SIZE` defines the number of samples used for the FFT (must be a power of 2).  
  - `MAX_SAMPLING_FREQ` is the maximum rate at which the device is capable of sampling.
- **Adaptive Variables:**  
  - `currentSamplingFreq` holds the active sampling frequency.
  - `lockedAdaptiveFreq` (saved in RTC memory) preserves the optimal frequency across deep sleep cycles.
- **FFT Computation:**  
  The code uses the `arduinoFFT` library to perform FFT on `vReal` and `vImag` arrays, identifying the maximum frequency component and setting the optimal sampling frequency accordingly.

### Data Aggregation and Transmission
- **Aggregation Window:**  
  Sensor data is aggregated over a 10-second interval (`AGGREGATE_WINDOW_MS`).
- **Communication Load:**  
  After aggregation, the sensor’s average reading (formatted to four decimal places) is transmitted. This message is fixed at 6 bytes regardless of the internal sampling strategy.

---

## Performance Evaluation

### Energy Consumption

The adaptive sampling method is designed to significantly reduce energy use compared to continuous high-rate sampling. This is achieved by performing a brief period of high-frequency sampling for FFT analysis, then switching to an optimal, lower sampling rate with deep sleep between transmissions.

- **Software-Based Integration:**  
  Energy savings are computed by integrating power data over time using methods like trapezoidal integration. These calculations reveal that the adaptive system consumes far less energy during its sleep and low-power operation phases compared to an oversampled system.
  
- **Real-World Measurements:**  
  Power consumption was also verified with an INA219 current sensor. The sensor’s readings clearly demonstrate that the adaptive sampling mode produces lower current draw during deep sleep and transmission bursts. This experimental evidence provides additional confidence in the energy savings claims.

*Visuals:*  
- [Energy Savings Graph](./images/energy_savings.png)  
- [Power Consumption Comparison](./images/power_comparison.png)

### Data Transmission Volume

Despite the significant differences in internal sampling strategies, both systems transmit the same volume of data over the network. This is because only one aggregated value is sent every 10 seconds. The payload is always a fixed-size message (6 bytes), regardless of whether the system is using adaptive sampling or oversampling.

- **Network Load Consistency:**  
  The fixed interval and constant payload size mean that the network load is predictable and minimal—approximately 36 bytes per minute. This predictable load is beneficial for network stability and is especially important in bandwidth-constrained environments.

*Visual:*  
- [Payload Size Screenshot](./images/payload_size.png)

### End-to-End Latency

End-to-end latency in the system—measured from the time a sensor reading is generated at the ESP32 until the aggregated message is received at the MQTT broker—has been consistently around 215 milliseconds. 

- **Latency Measurement Approach:**  
  The system uses NTP synchronization to accurately timestamp messages at the source and on the receiving edge server. This allows precise computation of the latency.
  
- **Reliability:**  
  A consistent latency of around 215 ms is acceptable for many real-time IoT applications, ensuring that the system is both responsive and reliable.

*Visual:*  
- [Latency Measurements](./images/latency.png)

### Real-World Power Measurements

In addition to software-simulated measurements, actual power usage was recorded using an INA219 sensor. These measurements show the actual current draw during various operating modes, such as during the FFT, the deep sleep periods, and the transmission bursts. The data validates that adaptive sampling yields substantial energy savings.

*Visuals:*  
- [INA219 Reading 1](./images/ina219_1.png)  
- [INA219 Reading 2](./images/ina219_2.png)  
- [INA219 Reading 3](./images/ina219_3.png)

---

## Setup and Usage

### Prerequisites
- **Hardware:**  
  - ESP32 board (e.g., Heltec WiFi LoRa V3)  
  - INA219 current sensor (for power measurement validation)
- **Network Requirements:**  
  - A WiFi network with a configured MQTT broker (e.g., Mosquitto)  
  - A LoRaWAN gateway and a TTN account for cloud transmissions
- **Software:**  
  - Arduino IDE or PlatformIO for compiling and flashing the firmware  
  - Python for running the MQTT monitoring script

### Installation Steps

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/your-username/adaptive-iot-sampling.git
   cd adaptive-iot-sampling
