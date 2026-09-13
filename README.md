# Mecanum Otonom Araç Platformu

Bu proje, Elektrik-Elektronik Mühendisliği bitirme projesi kapsamında geliştirilen, bilgisayarlı görü destekli otonom bir Mecanum tekerlekli araç platformudur.

Sistem; gerçek zamanlı motor ve sensör kontrolü için **STM32F407 Discovery**, görüntü işleme ve otonom navigasyon işlemleri için ise **Raspberry Pi 5** kullanmaktadır.

## Projenin Özellikleri

- Mecanum tekerlekler ile holonomik hareket
- Otonom şerit takibi
- Trafik işareti algılama ve sınıflandırma
- YOLO tabanlı görüntü işleme
- Dijkstra algoritması ile rota planlama
- Kavşaklarda otonom karar verme
- Engel algılama
- MPU6050 ile yönelim ve yaw kontrolü
- STM32 ve Raspberry Pi arasında UART haberleşmesi
- NRF24L01+ ile kablosuz manuel kontrol

## Kullanılan Teknolojiler

- STM32F407 Discovery
- Raspberry Pi 5
- C / STM32 HAL
- Python
- OpenCV
- YOLO
- Dijkstra Algoritması
- MPU6050
- Ultrasonik Sensörler
- NRF24L01+

## Sistem Yapısı

**STM32F407**
- Motor kontrolü
- Encoder okuma
- IMU verilerinin işlenmesi
- Yaw stabilizasyonu
- Ultrasonik sensörlerin kontrolü
- Gerçek zamanlı kontrol işlemleri

**Raspberry Pi 5**
- Kamera görüntüsünün işlenmesi
- Şerit takibi
- Trafik işareti tespiti
- Rota planlama
- Otonom sürüş kararları

## Proje Ekibi

- MohamedEldur Esmaeil 
- Ehsan Alzarrad

## Akademik Bilgi

**KTO Karatay Üniversitesi**  
Elektrik-Elektronik Mühendisliği  
Bitirme Projesi  
2025–2026
