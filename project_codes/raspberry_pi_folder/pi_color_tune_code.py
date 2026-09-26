import cv2
import numpy as np
from picamera2 import Picamera2

CAM_W = 640
CAM_H = 480

def nothing(x):
    pass

def main():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (CAM_W, CAM_H), "format": "BGR888"})
    picam2.configure(config)
    picam2.start()

    # KÄ±rmÄ±zÄ± penceresi
    cv2.namedWindow("Kirmizi Ayar")
    cv2.createTrackbar("H1_low",  "Kirmizi Ayar", 0,   10,  nothing)
    cv2.createTrackbar("H1_high", "Kirmizi Ayar", 10,  10,  nothing)
    cv2.createTrackbar("H2_low",  "Kirmizi Ayar", 160, 180, nothing)
    cv2.createTrackbar("H2_high", "Kirmizi Ayar", 180, 180, nothing)
    cv2.createTrackbar("S_low",   "Kirmizi Ayar", 88,  255, nothing)
    cv2.createTrackbar("S_high",  "Kirmizi Ayar", 255, 255, nothing)
    cv2.createTrackbar("V_low",   "Kirmizi Ayar", 10,  255, nothing)
    cv2.createTrackbar("V_high",  "Kirmizi Ayar", 255, 255, nothing)

    # Mavi penceresi
    cv2.namedWindow("Mavi Ayar")
    cv2.createTrackbar("H_low",  "Mavi Ayar", 100, 180, nothing)
    cv2.createTrackbar("H_high", "Mavi Ayar", 130, 180, nothing)
    cv2.createTrackbar("S_low",  "Mavi Ayar", 50,  255, nothing)
    cv2.createTrackbar("S_high", "Mavi Ayar", 255, 255, nothing)
    cv2.createTrackbar("V_low",  "Mavi Ayar", 50,  255, nothing)
    cv2.createTrackbar("V_high", "Mavi Ayar", 255, 255, nothing)

    # YeÅŸil penceresi
    cv2.namedWindow("Yesil Ayar")
    cv2.createTrackbar("H_low",  "Yesil Ayar", 40,  180, nothing)
    cv2.createTrackbar("H_high", "Yesil Ayar", 80,  180, nothing)
    cv2.createTrackbar("S_low",  "Yesil Ayar", 60,  255, nothing)
    cv2.createTrackbar("S_high", "Yesil Ayar", 255, 255, nothing)
    cv2.createTrackbar("V_low",  "Yesil Ayar", 60,  255, nothing)
    cv2.createTrackbar("V_high", "Yesil Ayar", 255, 255, nothing)

    print("Kaydet: 's' | Cik: 'q'")

    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # KÄ±rmÄ±zÄ±
        h1l  = cv2.getTrackbarPos("H1_low",  "Kirmizi Ayar")
        h1h  = cv2.getTrackbarPos("H1_high", "Kirmizi Ayar")
        h2l  = cv2.getTrackbarPos("H2_low",  "Kirmizi Ayar")
        h2h  = cv2.getTrackbarPos("H2_high", "Kirmizi Ayar")
        rs_l = cv2.getTrackbarPos("S_low",   "Kirmizi Ayar")
        rs_h = cv2.getTrackbarPos("S_high",  "Kirmizi Ayar")
        rv_l = cv2.getTrackbarPos("V_low",   "Kirmizi Ayar")
        rv_h = cv2.getTrackbarPos("V_high",  "Kirmizi Ayar")
        mask_r1  = cv2.inRange(hsv, np.array([h1l, rs_l, rv_l]), np.array([h1h, rs_h, rv_h]))
        mask_r2  = cv2.inRange(hsv, np.array([h2l, rs_l, rv_l]), np.array([h2h, rs_h, rv_h]))
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)

        # Mavi
        bhl  = cv2.getTrackbarPos("H_low",  "Mavi Ayar")
        bhh  = cv2.getTrackbarPos("H_high", "Mavi Ayar")
        bs_l = cv2.getTrackbarPos("S_low",  "Mavi Ayar")
        bs_h = cv2.getTrackbarPos("S_high", "Mavi Ayar")
        bv_l = cv2.getTrackbarPos("V_low",  "Mavi Ayar")
        bv_h = cv2.getTrackbarPos("V_high", "Mavi Ayar")
        mask_blue = cv2.inRange(hsv, np.array([bhl, bs_l, bv_l]), np.array([bhh, bs_h, bv_h]))

        # YeÅŸil
        ghl  = cv2.getTrackbarPos("H_low",  "Yesil Ayar")
        ghh  = cv2.getTrackbarPos("H_high", "Yesil Ayar")
        gs_l = cv2.getTrackbarPos("S_low",  "Yesil Ayar")
        gs_h = cv2.getTrackbarPos("S_high", "Yesil Ayar")
        gv_l = cv2.getTrackbarPos("V_low",  "Yesil Ayar")
        gv_h = cv2.getTrackbarPos("V_high", "Yesil Ayar")
        mask_green = cv2.inRange(hsv, np.array([ghl, gs_l, gv_l]), np.array([ghh, gs_h, gv_h]))

        # BirleÅŸik
        mask_combined = cv2.bitwise_or(mask_red, mask_blue)
        mask_combined = cv2.bitwise_or(mask_combined, mask_green)

        cv2.imshow("Kirmizi Ayar", mask_red)
        cv2.imshow("Mavi Ayar",    mask_blue)
        cv2.imshow("Yesil Ayar",   mask_green)
        cv2.imshow("Birlesik Maske", mask_combined)
        cv2.imshow("Kamera", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            print("\n=== KAYDET ===")
            print(f"lower_red1  = np.array([{h1l}, {rs_l}, {rv_l}])")
            print(f"upper_red1  = np.array([{h1h}, {rs_h}, {rv_h}])")
            print(f"lower_red2  = np.array([{h2l}, {rs_l}, {rv_l}])")
            print(f"upper_red2  = np.array([{h2h}, {rs_h}, {rv_h}])")
            print(f"lower_blue  = np.array([{bhl}, {bs_l}, {bv_l}])")
            print(f"upper_blue  = np.array([{bhh}, {bs_h}, {bv_h}])")
            print(f"lower_green = np.array([{ghl}, {gs_l}, {gv_l}])")
            print(f"upper_green = np.array([{ghh}, {gs_h}, {gv_h}])")
            print("==============\n")

    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()