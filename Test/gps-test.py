import serial
import time

# ─── SETTINGS ───────────────────────────────────────
PORT = '/dev/ttyUSB0'
BAUD = 115200
# ────────────────────────────────────────────────────

def checksum(msg):
    ck_a, ck_b = 0, 0
    for byte in msg:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def build_ubx(cls, id, payload):
    msg = bytes([cls, id]) + len(payload).to_bytes(2, 'little') + payload
    return b'\xB5\x62' + msg + checksum(msg)

def send_ubx(ser, msg, label):
    ser.write(msg)
    time.sleep(0.1)
    print(f"Sent: {label}")

def build_cfg_valset(key, value):
    # layers = 0x07 means RAM + BBR + Flash (permanent)
    header = bytes([
        0x00,        # version
        0x07,        # layers - RAM + BBR + Flash
        0x00, 0x00   # reserved
    ])
    payload = header + key + bytes([value])
    return build_ubx(0x06, 0x8A, payload)

# CFG keys for each signal
SIGNALS = {
    # GPS
    'GPS L5':      (bytes([0x01, 0x00, 0x31, 0x10]), 0x01),
    # Galileo E5a (L5 band)
    'GAL E5a':     (bytes([0x01, 0x00, 0x56, 0x10]), 0x01),
    # BeiDou B2a (L5 band)
    'BDS B2a':     (bytes([0x01, 0x00, 0x76, 0x10]), 0x01),
    # QZSS L5
    'QZSS L5':     (bytes([0x01, 0x00, 0x91, 0x10]), 0x01),
    # NavIC L5 (entire system)
    'NavIC L5':    (bytes([0x01, 0x00, 0xA1, 0x10]), 0x01),
}

def main():
    print(f"Connecting to {PORT} at {BAUD} baud...")
    
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(1)
        
        print("\nEnabling L5 signals...\n")
        
        for label, (key, value) in SIGNALS.items():
            msg = build_cfg_valset(key, value)
            send_ubx(ser, msg, label)
            time.sleep(0.2)
        
        print("\nAll L5 signals enabled and saved to flash.")
        print("Waiting for receiver to settle...\n")
        time.sleep(3)
        
        # Verify by reading NMEA and checking for NavIC
        print("Reading NMEA output — checking for NavIC ($GIGSV)...")
        print("Wait up to 2 minutes for NavIC satellites to appear.\n")
        
        start = time.time()
        navic_seen = False
        
        while time.time() - start < 120:
            line = ser.readline().decode('ascii', errors='replace').strip()
            
            if '$GIGSV' in line:
                print(f"✅ NavIC detected: {line}")
                navic_seen = True
            
            elif '$GNGGA' in line:
                parts = line.split(',')
                if len(parts) > 7:
                    fix  = parts[6]
                    sats = parts[7]
                    print(f"Fix quality: {fix}  Satellites: {sats}")
            
            if navic_seen:
                print("\n✅ L5 successfully enabled! NavIC is working.")
                break
        
        if not navic_seen:
            print("\n⚠️  NavIC not seen yet — this is normal indoors.")
            print("Place antenna outside with clear sky view and try again.")

if __name__ == '__main__':
    main()