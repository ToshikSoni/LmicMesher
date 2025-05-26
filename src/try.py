import serial
import time
from collections import deque

# Replace with actual ports
COM_IN = '/dev/cu.usbserial-3'
COM_OUT = '/dev/cu.usbserial-0001'
BAUD_RATE = 115200
CHUNK_SIZE = 51

orange =  '\x1b[33m'
pink =  '\x1b[31m'
green = '\x1b[32m'
reset = '\x1b[0m'

# Queue to hold outgoing data chunks
message_queue = deque()

try:
    ser_in = serial.Serial(COM_IN, BAUD_RATE, timeout=1)
    ser_out = serial.Serial(COM_OUT, BAUD_RATE, timeout=1)

    print(f"Listening on {COM_IN} and writing to {COM_OUT}...")

    while True:
        # Handle input from MESHER
        if ser_in.in_waiting:
            line = ser_in.readline().decode('utf-8', errors='ignore').strip()
            print(f"{orange}MESHER:{reset} {line}")

            if line.startswith("DATA:"):
                print(f"{green}Processing DATA line:{reset} {line}")
                try:
                    # Format: DATA:<long_payload> <dst_hex>
                    data_content = line[5:]
                    d, dst = data_content.rsplit(' ', 1)  # Split from right
                    dst_int = int(dst, 16)
                    a = dst_int // 256
                    b = dst_int % 256

                    # Convert the string of space-separated numbers to int
                    d_bytes = [int(val) for val in d.strip().split()]
                    d_chunks = [d_bytes[i:i+CHUNK_SIZE-2] for i in range(0, len(d_bytes), CHUNK_SIZE-2)]

                    for chunk in d_chunks:
                        full_chunk = chunk + [a, b]  # Append destination bytes
                        enc_data = bytes(full_chunk)
                        message_queue.append(enc_data)
                        print(f"{green}Queued chunk:{reset} {full_chunk}")

                except Exception as e:
                    print(f"{pink}Parsing error:{reset} {e}")

        # Forward queued chunks to LMIC
        if message_queue:
            to_send = message_queue.popleft()
            ser_out.write(to_send + b'\n')
            print(f"{green}Sent chunk to LMIC:{reset} {list(to_send)}")
            time.sleep(1.2)  # Optional: Throttle for LoRaWAN

        # Read response from LMIC
        if ser_out.in_waiting:
            line = ser_out.readline().decode('utf-8', errors='ignore').strip()
            print(f"{pink}LMIC:{reset} {line}")

            if line.startswith("RETURN:"):
                data_str = line[7:]
                print(f"{green}Returning to MESHER:{reset} {data_str}")
                try:
                    data = [int(i) for i in data_str.split()]
                    enc_data = bytes(data)
                    ser_in.write(enc_data + b'\n')
                except Exception as e:
                    print(f"{pink}Return parse error:{reset} {e}")

except serial.SerialException as e:
    print(f"Serial error: {e}")

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    try:
        ser_in.close()
        ser_out.close()
    except:
        pass
