import mysql.connector
import serial
import time
import geocoder
from datetime import datetime, timedelta

# Serial connection
ser = serial.Serial('COM6', 115200, timeout=1)
time.sleep(2)  # Wait for ESP32 to initialize

# MySQL connection setup
db = mysql.connector.connect(
    host="localhost",
    user="esp_user",
    password="ESPtesting123!!!",
    database="esp_user"
)
cursor = db.cursor()

def send_time_and_location():
    offset = timedelta(hours=3)
    now = datetime.utcnow() + offset
    time_string = f"TIME:{now.hour},{now.minute},{now.second},{now.day},{now.month},{now.year}\n"

    g = geocoder.ip('me')
    if g.ok and g.latlng:
        latitude, longitude = g.latlng
    else:
        latitude, longitude = 56.5047, 21.0108  # Fallback if geolocation fails
        print("⚠️  Using fallback coordinates.")

    location_string = f"LOC:{latitude},{longitude}\n"

    print("Sending to ESP32:")
    print("  Time:", time_string.strip())
    print("  Location:", location_string.strip())

    ser.write(time_string.encode())
    time.sleep(0.1)
    ser.write(location_string.encode())

    print("  Status: Sent\n")

def insert_data_to_mysql(timestamp, latitude, longitude, azimuth, ldr_left, ldr_right, base_pos, state):
    query = """
        INSERT INTO sun_tracking_data (
            timestamp, latitude, longitude, sun_azimuth,
            ldr_left, ldr_right, base_position, system_state
        ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
    """
    values = (timestamp, latitude, longitude, azimuth, ldr_left, ldr_right, base_pos, state)
    cursor.execute(query, values)
    db.commit()

def read_esp_data():
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        if line.startswith("DATA:"):
            data = line[len("DATA:"):]
            print("[ESP DATA]", data)
            parts = data.split(',')
            if len(parts) == 8:
                timestamp, lat, lon, az, ldr_l, ldr_r, base, state = parts
                try:
                    insert_data_to_mysql(timestamp, float(lat), float(lon), float(az),
                                         int(ldr_l), int(ldr_r), int(base), state)
                except Exception as e:
                    print("Error inserting to DB:", e)

# === Main loop ===
last_sent = time.time()

while True:
    now = time.time()

    # Send time/location update every 5 seconds
    if now - last_sent >= 5:
        send_time_and_location()
        last_sent = now

    # Always listen for ESP responses
    read_esp_data()

    time.sleep(0.9)
