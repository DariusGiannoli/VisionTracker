"""
Advertise the control endpoint via Bonjour/mDNS so the visionOS app can auto-discover.
Publishes a TXT record with "host=<ip_or_dns>".
"""
from zeroconf import Zeroconf, ServiceInfo
import socket, time

def get_host_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip

if __name__ == "__main__":
    zc = Zeroconf()
    host = get_host_ip()
    service = ServiceInfo(
        type_="_lerobot._tcp.local.",
        name=f"LeRobot Control._lerobot._tcp.local.",
        addresses=[socket.inet_aton(host)],
        port=8766,
        properties={"host": host.encode("utf-8")},
    )
    zc.register_service(service)
    print(f"mDNS advertised at {host}:8766 (type _lerobot._tcp)")
    try:
        while True: time.sleep(3600)
    except KeyboardInterrupt:
        zc.unregister_service(service)
        zc.close()
