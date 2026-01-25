import serial
import sys
import time

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM2'
print(f"🦎 Attempting to wake Lizard on {port}...")

try:
    # We must set DTR and RTS to True to allow the S3 application to run
    ser = serial.Serial(port, 115200, timeout=0.5)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.1)
    
    # Send multiple newlines to clear any garbage and trigger the prompt
    ser.write(b"\n\ncore.version()\n")
    ser.flush()
    
    print("📡 Listening (Ctrl+C to stop)...")
    while True:
        if ser.in_waiting:
            line = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
            print(line, end='', flush=True)
        time.sleep(0.01)
except Exception as e:
    print(f"❌ Error: {e}")
finally:
    if 'ser' in locals(): ser.close()
