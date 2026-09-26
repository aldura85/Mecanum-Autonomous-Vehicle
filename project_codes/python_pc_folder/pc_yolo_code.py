import cv2
import numpy as np
import socket
import struct
import time
from ultralytics import YOLO

# ─────────────────────────────────────────
#  AYARLAR
# ─────────────────────────────────────────
MODEL_PATH   = r"D:\user\Download\weights\weights\best.pt"
HOST         = "0.0.0.0"   # tüm arayüzleri dinle
PORT         = 5555
CONF         = 0.75

# Hangi sınıflar önemli → Pi'ye komut gönderilecek
ACTION_MAP = {
    "dur":         "DUR",
    "kirmizi":     "DUR",
    "sari":        "YAVAS",
    "yesil":       "DEVAM",
    "sag":         "SAG",
    "sol":         "SOL",
    "ilerisag":    "ILERISAG",
    "ilerisol":    "ILERISOL",
    "sagadonulmez":"SAGKAPALI",
    "soladonulmez":"SOLKAPALI",
    "girisyok":    "ILERIKAPALI",
    "20":          "HIZ20",
    "30":          "HIZ30",
    "durak":       "DEVAM",
    "park":        "DEVAM",
    "parkyasak":   "DEVAM",
}

# ─────────────────────────────────────────
#  MODEL YÜKLEMESİ
# ─────────────────────────────────────────
print("Model yükleniyor... - pc_yolo YOLO.py:39")
model = YOLO(MODEL_PATH)
print(f"Model hazır! Sınıflar: {model.names} - pc_yolo YOLO.py:41")


def recv_frame(conn):
    """Pi'den JPEG frame al."""
    raw = b""

    while len(raw) < 4:
        chunk = conn.recv(4 - len(raw))
        if not chunk:
            return None
        raw += chunk

    size = struct.unpack(">I", raw)[0]

    data = b""
    while len(data) < size:
        chunk = conn.recv(min(4096, size - len(data)))
        if not chunk:
            return None
        data += chunk

    arr = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

    return frame


def send_result(conn, label, conf):
    """Pi'ye sonuç gönder."""
    msg = f"{label}:{conf:.2f}\n".encode()
    conn.sendall(msg)


def run_yolo(frame):
    """YOLO çalıştır, en güvenilir tespiti döndür."""
    results = model(frame, verbose=False, conf=CONF, iou=0.4)

    best_label = "YOK"
    best_conf  = 0.0

    for result in results:
        for box in result.boxes:
            c = float(box.conf[0])
            cls = int(box.cls[0])
            name = model.names[cls]

            if c > best_conf:
                best_conf = c
                best_label = name

            # Debug — bounding box çiz
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cv2.putText(
                frame,
                f"{name} {c:.2f}",
                (x1, y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

    action = ACTION_MAP.get(best_label, "DEVAM")

    return best_label, best_conf, action, frame


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    print(f"Pi bekleniyor... port {PORT} - pc_yolo YOLO.py:117")

    while True:
        conn, addr = server.accept()
        print(f"Pi bağlandı: {addr} - pc_yolo YOLO.py:121")

        t_prev = time.time()

        try:
            while True:
                frame = recv_frame(conn)

                if frame is None:
                    print("Bağlantı kesildi. - pc_yolo YOLO.py:130")
                    break

                label, conf, action, debug = run_yolo(frame)

                # Pi'ye hâlâ action gönderiliyor
                send_result(conn, action, conf)

                now = time.time()
                fps = 1.0 / max(now - t_prev, 0.001)
                t_prev = now

                # Ekranda artık DUR / DEVAM / action yazmıyor
                cv2.putText(
                    debug,
                    f"{label}  conf={conf:.2f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )

                cv2.putText(
                    debug,
                    f"FPS={fps:.1f}",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 200, 0),
                    2
                )

                cv2.imshow("YOLO - PC", debug)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except Exception as e:
            print(f"Hata: {e} - pc_yolo YOLO.py:169")

        finally:
            conn.close()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()