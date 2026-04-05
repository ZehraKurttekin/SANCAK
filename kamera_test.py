import cv2

# Genellikle simülasyonda 0 numaralı port varsayılan kameradır
cap = cv2.VideoCapture(0) 

while True:
    ret, frame = cap.read()
    if not ret:
        print("Kamera verisi alınamıyor! Modelde kamera ekli mi?")
        break

    # Görüntüyü ekranda göster
    cv2.imshow('Sancak IHA Kamera Testi', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
