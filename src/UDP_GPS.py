import socket
import threading

# -------------------- UDP Config --------------------
UDP_IP = "0.0.0.0"   # Listen on all interfaces
UDP_PORT = 5000

# Latest GPS received (shared across threads)
_latest_gps = {
    "lat": None,
    "lon": None,
    "alt": None
}

def _udp_listener():
    """Background thread to listen for UDP GPS packets."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening for UDP on port {UDP_PORT}...")

    while True:
        data, addr = sock.recvfrom(4096)
        try:
            text = data.decode(errors="ignore").strip()
            # Expecting CSV: timestamp,lat,lon,alt,...
            parts = text.split(",")
            if len(parts) >= 4:
                lat = float(parts[1])  # skip timestamp
                lon = float(parts[2])
                alt = float(parts[3])
                _latest_gps["lat"] = lat
                _latest_gps["lon"] = lon
                _latest_gps["alt"] = alt
        except Exception as e:
            print(f"[UDP] Failed to parse: {text} | {e}")

# Start listener in background thread
_thread = threading.Thread(target=_udp_listener, daemon=True)
_thread.start()

# -------------------- API --------------------
def get_current_lat_lon_alt():
    """Return the most recent GPS from UDP."""
    lat = _latest_gps["lat"]
    lon = _latest_gps["lon"]
    alt = _latest_gps["alt"]

    if lat is None or lon is None or alt is None:
        # Return some default if nothing received yet
        return 0.0, 0.0, 0.0
    return lat, lon, alt