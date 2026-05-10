#!/usr/bin/env python3
"""
sensor_logger.py

Reads binary sensor data from Arduino via Serial and logs to CSV.

Packet format (64 bytes, little-endian):
  Bytes  0-3:   Sync marker (0x55 0xAA 0x55 0xAA)
  Bytes  4-7:   PPG Red
  Bytes  8-11:  PPG IR
  Bytes 12-15:  PPG Green
  Bytes 16-19:  accX (scaled by 0.01)
  Bytes 20-23:  accY (scaled by 0.01)
  Bytes 24-27:  accZ (scaled by 0.01)
  Bytes 28-31:  gyrX
  Bytes 32-35:  gyrY
  Bytes 36-39:  gyrZ
  Bytes 40-43:  magX
  Bytes 44-47:  magY
  Bytes 48-51:  magZ
  Bytes 52-55:  temperature
  Bytes 56-59:  timestamp (millis() as float)
  Bytes 60-63:  padding (zeros)

Usage:
  python sensor_logger.py [--port COM_PORT] [--baud BAUD_RATE]
"""

import serial
import struct
import csv
import time
import argparse
import os
import sys
from datetime import datetime

# ===================== Constants =====================
PACKET_SIZE = 64
NUM_FLOATS = 14
SYNC_MARKER = bytes([0x55, 0xAA, 0x55, 0xAA])
SYNC_LEN = 4

# Column names matching the 14 float values in the packet + aligned PC timestamp
COLUMN_NAMES = [
    'Red', 'IR', 'Green',
    'accX', 'accY', 'accZ',
    'gyrX', 'gyrY', 'gyrZ',
    'magX', 'magY', 'magZ',
    'temp', 'timestamp', 'pc_timestamp'
]

# ===================== Sync Detection =====================

def find_sync(ser):
    """Scan the serial stream byte-by-byte until the 4-byte sync marker is found.
    
    After finding the sync marker, the next PACKET_SIZE - SYNC_LEN bytes
    form the payload.
    
    Returns True if sync found, False on timeout/error.
    """
    match_pos = 0
    start_time = time.time()
    timeout = 5.0  # seconds

    while match_pos < SYNC_LEN:
        byte = ser.read(1)
        if not byte:
            # Timeout check
            if time.time() - start_time > timeout:
                return False
            continue

        if byte[0] == SYNC_MARKER[match_pos]:
            match_pos += 1
        else:
            # Reset: check if this byte could be the start of a new sync
            if byte[0] == SYNC_MARKER[0]:
                match_pos = 1
            else:
                match_pos = 0

    return True


def read_packet(ser):
    """Read one complete packet from serial.
    
    First finds the sync marker, then reads the remaining payload.
    Returns the 14-float data as a list, or None on error.
    """
    # Find sync marker
    if not find_sync(ser):
        return None

    # Read payload (60 bytes = 14 floats + 4 bytes padding)
    payload_size = PACKET_SIZE - SYNC_LEN
    payload = ser.read(payload_size)

    if len(payload) < payload_size:
        print(f"Warning: Incomplete payload ({len(payload)}/{payload_size} bytes)")
        return None

    # Unpack 14 floats (little-endian)
    try:
        values = struct.unpack('<14f', payload[:NUM_FLOATS * 4])
    except struct.error as e:
        print(f"Warning: Unpack error: {e}")
        return None

    return list(values)


# ===================== Data Validation =====================

def validate_packet(values):
    """Validate parsed packet data. Returns True if data looks reasonable."""
    if values is None:
        return False

    # Check for excessive NaN values
    nan_count = sum(1 for v in values if v != v)  # NaN != NaN
    if nan_count > 3:
        return False

    # Check timestamp is reasonable (0 to ~50 days in millis)
    timestamp = values[13]
    if timestamp == timestamp and (timestamp < 0 or timestamp > 4.3e9):
        return False

    return True


# ===================== Timestamp Synchronization =====================

def sync_timestamps(ser, rounds=20):
    """Synchronize Arduino millis() with PC wall-clock time via round-trip measurement.

    Sends multiple 't' commands and selects the round with minimum RTT to minimize
    asymmetric delay error. Returns the offset in seconds:
        pc_time = arduino_millis / 1000.0 + offset

    Expected accuracy: ≤1 ms over USB serial (typical min RTT ~1-2 ms).
    """
    best_rtt = float('inf')
    best_offset = 0.0
    success_count = 0

    for _ in range(rounds):
        ser.reset_input_buffer()
        t1 = time.perf_counter()
        ser.write(b't\n')
        ser.flush()
        response = ser.readline()
        t2 = time.perf_counter()

        if not response or not response.startswith(b'T'):
            continue

        try:
            arduino_ms = int(response[1:].strip())
        except ValueError:
            continue

        rtt = t2 - t1
        # Offset = PC midpoint time - Arduino time
        offset = (t1 + t2) / 2.0 - arduino_ms / 1000.0

        if rtt < best_rtt:
            best_rtt = rtt
            best_offset = offset
        success_count += 1

    if success_count == 0:
        print("Warning: Timestamp sync failed, using offset=0")
        return 0.0

    print(f"Timestamp sync: offset={best_offset * 1000:.3f} ms "
          f"(min RTT={best_rtt * 1000:.3f} ms, {success_count}/{rounds} rounds)")
    return best_offset


# ===================== Main Logger =====================

def main():
    parser = argparse.ArgumentParser(description='Sensor Data Logger')
    parser.add_argument('--port', type=str, default='COM5',
                        help='Serial port (default: COM5)')
    parser.add_argument('--baud', type=int, default=1000000,
                        help='Baud rate (default: 1000000)')
    args = parser.parse_args()

    # Generate output filename with timestamp
    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_dir = os.path.join(os.path.dirname(os.getcwd()), 'log')
    os.makedirs(log_dir, exist_ok=True)
    output_file = os.path.join(log_dir, f'sensor_data_{timestamp_str}.csv')

    # Open serial port
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Error opening serial port {args.port}: {e}")
        sys.exit(1)

    print(f"Connected to {args.port} at {args.baud} baud")

    # Wait for Arduino to reset after serial connection
    time.sleep(2.0)

    # Flush any stale data in the input buffer
    ser.reset_input_buffer()

    # Synchronize Arduino millis() with PC wall-clock time
    ts_offset = sync_timestamps(ser, rounds=20)

    # Send 's' command to start data collection
    ser.write(b's\n')
    ser.flush()
    print("Sent 's' command - data collection started on Arduino")

    # Wait briefly for Arduino to process the command and start sending
    time.sleep(0.5)

    print(f"Output file: {output_file}")
    print("Logging data... (Press Ctrl+C to stop)")

    # Open CSV file for writing
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(COLUMN_NAMES)

        packet_count = 0
        invalid_count = 0
        start_time = time.time()
        last_progress_time = start_time
        last_progress_count = 0

        try:
            while True:
                values = read_packet(ser)

                if values is None:
                    continue

                if not validate_packet(values):
                    invalid_count += 1
                    continue

                # Write to CSV (Red, IR, Green, timestamp as int; rest as float)
                row = []
                for i, v in enumerate(values):
                    if i in (0, 1, 2, 13):  # Red, IR, Green, timestamp
                        row.append(str(int(v)))
                    else:
                        row.append(f'{v:.6f}')
                # Append PC-aligned timestamp (Unix epoch in seconds, 6 decimal places)
                pc_ts = values[13] / 1000.0 + ts_offset
                row.append(f'{pc_ts:.6f}')
                writer.writerow(row)
                packet_count += 1

                # Print progress every 100 packets (real-time rate)
                if packet_count % 100 == 0:
                    now = time.time()
                    interval = now - last_progress_time
                    rate = (packet_count - last_progress_count) / interval if interval > 0 else 0
                    last_progress_time = now
                    last_progress_count = packet_count
                    print(f"Packets: {packet_count} | Rate: {rate:.1f} Hz | "
                          f"Invalid: {invalid_count}", end='\r')

        except KeyboardInterrupt:
            # Send 'e' command to stop data collection on Arduino
            print()
            print("Stopping data collection...")
            ser.write(b'e\n')
            ser.flush()
            time.sleep(0.5)  # Wait for Arduino to send summary

            elapsed = time.time() - start_time
            print("===== Logging Summary =====")
            print(f"Total packets: {packet_count}")
            print(f"Invalid packets: {invalid_count}")
            if elapsed > 0:
                print(f"Average rate: {packet_count / elapsed:.1f} Hz")
            print(f"Duration: {elapsed:.1f} s")
            print(f"Data saved to: {output_file}")
            print("===========================")

        finally:
            ser.close()


if __name__ == '__main__':
    main()