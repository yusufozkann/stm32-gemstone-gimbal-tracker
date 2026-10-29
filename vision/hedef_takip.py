# hedef_takip.py
# kameradan kare al -> yolo ile ihayi bul -> merkezini uart'tan stm32'ye yolla
# stm32 tarafinda pid servolari ceviriyor
#
# python hedef_takip.py                 -> normal
# python hedef_takip.py --no-serial     -> stm32 olmadan sadece modeli dene
# python hedef_takip.py --flip-x        -> pan ters donerse
# cikmak icin q
# not: putty aciksa com portu mesgul olur, once kapat

import argparse
import time

import cv2
from ultralytics import YOLO

GENISLIK, YUKSEKLIK = 640, 480   # stm32'deki GORUNTU_GENISLIK/YUKSEKLIK ile ayni olmali


def argumanlar():
    p = argparse.ArgumentParser()
    p.add_argument("--model", default="best.pt")
    p.add_argument("--cam", type=int, default=1)          # 0 laptop kamerasi, 1 c270
    p.add_argument("--port", default="COM3")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--imgsz", type=int, default=640)      # yavas kalirsa 480 / 320
    p.add_argument("--conf", type=float, default=0.40)    # bunun altini hedef sayma
    p.add_argument("--flip-x", action="store_true")
    p.add_argument("--flip-y", action="store_true")
    p.add_argument("--no-serial", action="store_true")
    return p.parse_args()


def seri_ac(a):
    if a.no_serial:
        return None
    import serial
    s = serial.Serial(a.port, a.baud, timeout=0)   # timeout=0 -> okurken bekleme
    time.sleep(0.2)
    s.reset_input_buffer()
    print(f"[seri] {a.port} @ {a.baud} acildi")
    return s


def kamera_ac(indeks):
    cap = cv2.VideoCapture(indeks, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, GENISLIK)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, YUKSEKLIK)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # eski kareler birikmesin, gecikme artiyor
    if not cap.isOpened():
        raise SystemExit(f"kamera {indeks} acilamadi, kamera_bul.py ile numarasina bak")
    return cap


def en_iyi_hedef(sonuc):
    # birden fazla kutu varsa en emin olani al
    kutular = sonuc.boxes
    if kutular is None or len(kutular) == 0:
        return None
    i = int(kutular.conf.argmax())
    x1, y1, x2, y2 = kutular.xyxy[i].tolist()
    return x1, y1, x2, y2, float(kutular.conf[i])


def main():
    a = argumanlar()
    model = YOLO(a.model)
    print(f"[model] siniflar: {model.names}")
    cap = kamera_ac(a.cam)
    seri = seri_ac(a)

    son_telemetri = ""
    seri_tampon = b""
    fps, t_once = 0.0, time.time()

    try:
        while True:
            ok, kare = cap.read()
            if not ok:
                print("kare alinamadi")
                break
            if kare.shape[1] != GENISLIK or kare.shape[0] != YUKSEKLIK:
                kare = cv2.resize(kare, (GENISLIK, YUKSEKLIK))

            sonuc = model.predict(kare, imgsz=a.imgsz, conf=a.conf, verbose=False)[0]
            hedef = en_iyi_hedef(sonuc)

            # stm32'ye: hedef varsa C<x>,<y>  yoksa N
            if hedef:
                x1, y1, x2, y2, conf = hedef
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                gx = GENISLIK - cx if a.flip_x else cx
                gy = YUKSEKLIK - cy if a.flip_y else cy
                komut = f"C{int(gx)},{int(gy)}\n"
            else:
                komut = "N\n"

            if seri:
                seri.write(komut.encode())
                # stm32'nin telemetrisini oku, satir satir ayir
                seri_tampon += seri.read(seri.in_waiting or 1)
                while b"\n" in seri_tampon:
                    satir, seri_tampon = seri_tampon.split(b"\n", 1)
                    satir = satir.decode(errors="ignore").strip()
                    if satir:
                        son_telemetri = satir

            # ekran: merkez artisi, kutu, merkezden hata cizgisi
            mx, my = GENISLIK // 2, YUKSEKLIK // 2
            cv2.drawMarker(kare, (mx, my), (255, 255, 255), cv2.MARKER_CROSS, 30, 1)
            if hedef:
                x1, y1, x2, y2, conf = hedef
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                cv2.rectangle(kare, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                cv2.circle(kare, (cx, cy), 4, (0, 0, 255), -1)
                cv2.line(kare, (mx, my), (cx, cy), (0, 255, 255), 1)
                cv2.putText(kare, f"KILIT {conf:.2f}  hata=({cx - mx},{cy - my})",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                cv2.putText(kare, "HEDEF YOK", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

            # fps'i biraz yumusatiyorum, yoksa surekli zipliyor
            simdi = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(simdi - t_once, 1e-3))
            t_once = simdi
            cv2.putText(kare, f"FPS {fps:4.1f}", (GENISLIK - 110, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if son_telemetri:
                cv2.putText(kare, son_telemetri, (10, YUKSEKLIK - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

            cv2.imshow("Gimbal Tracker", kare)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        if seri:
            seri.write(b"N\n")   # cikarken hedefi birak
            seri.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
