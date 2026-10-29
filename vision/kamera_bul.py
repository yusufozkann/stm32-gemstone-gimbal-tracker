# kamera_bul.py
# bilgisayardaki kameralari tek tek deneyip calisanlari yazar
# c270'in kacinci kamera oldugunu bulmak icin yazdim
# calistirmadan once kamera kullanan uygulamalari kapat

import cv2

print("kameralar taraniyor...\n")
bulunan = 0
for backend, isim in [(cv2.CAP_DSHOW, "DSHOW"), (cv2.CAP_MSMF, "MSMF")]:
    for i in range(6):
        cap = cv2.VideoCapture(i, backend)
        ok, kare = cap.read() if cap.isOpened() else (False, None)
        if ok:
            bulunan += 1
            print(f"{isim} kamera {i}: CALISIYOR, {kare.shape[1]}x{kare.shape[0]}")
        cap.release()

if bulunan == 0:
    print("calisan kamera yok")
