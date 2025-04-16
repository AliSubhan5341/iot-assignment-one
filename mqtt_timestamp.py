#!/usr/bin/env python3
import subprocess
import sys
from datetime import datetime, timedelta
import ntplib
import time

# Configuration
broker = "192.168.1.5"
topic = "test/esp32"

# Get NTP time offset for synchronized timestamps
def get_ntp_offset(ntp_server="time.google.com"):
    try:
        client = ntplib.NTPClient()
        response = client.request(ntp_server, version=3)
        return response.tx_time - time.time()
    except Exception as e:
        print("Failed to get NTP time:", e)
        return 0

ntp_offset = get_ntp_offset()

# Start mosquitto_sub as subprocess
process = subprocess.Popen(["mosquitto_sub", "-h", broker, "-t", topic, "-v"], stdout=subprocess.PIPE)

try:
    while True:
        line = process.stdout.readline()
        if not line:
            break

        # Decode the line
        decoded_line = line.decode().strip()

        # Get current synchronized time
        current_time = datetime.now() + timedelta(seconds=ntp_offset)
        formatted_time = current_time.strftime("[%Y-%m-%d %H:%M:%S.%f]")[:-3]

        # Extract message payload (everything after the topic name)
        if " " in decoded_line:
            _, payload = decoded_line.split(" ", 1)
            payload_size = len(payload.encode("utf-8"))  # count bytes, not characters
        else:
            payload = decoded_line
            payload_size = len(payload.encode("utf-8"))

        # Print formatted output
        print(f"{formatted_time} {decoded_line} | Payload size: {payload_size} bytes")

except KeyboardInterrupt:
    process.kill()
