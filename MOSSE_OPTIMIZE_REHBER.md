# MOSSE Optimize Mod - Kullanim Rehberi

## Ozellikler

MOSSE Optimize Mod, maksimum hiz icin ozel olarak ayarlanmis bir tracking sistemidir.

### Optimizasyonlar

1. **MOSSE Tracker**
   - En hizli OpenCV tracker
   - Dusuk hesaplama maliyeti
   - Gerçek zamanli performans

2. **Resize Optimizasyonu**
   - Input: 416x416 (YOLOv8n icin ideal)
   - Output: 1920x1080 (Tam ekran gosterim)
   - %60+ hiz artisi

3. **Kalman Filter Optimizasyonu**
   - Process noise: 0.01 (Hizli degisimlere uyum)
   - Prediction steps: 3 (Daha az hesaplama)
   - Daha hizli tahmin

4. **Trajectory Optimizasyonu**
   - Length: 20 frame (Daha kisa iz)
   - Daha az cizim islemi
   - Daha hizli render

5. **Bbox Validasyonu**
   - Min area: 50 px² (Daha esnek)
   - Max change: 0.6 (Daha toleransli)
   - Daha az red

## Kullanim

### Hizli Baslangic

```bash
run_mosse_fast.bat
```

### Manuel Calistirma

```bash
# Sanal ortami aktive et
venv\Scripts\activate

# MOSSE optimize mod
python src\main_mosse_fast.py --source screen --save
```

### Komut Satiri Secenekleri

```bash
python src\main_mosse_fast.py [SECENEKLER]

Secenekler:
  --source KAYNAK    Goruntu kaynagi (screen, 0, video.mp4)
  --model MODEL      YOLOv8 model (varsayilan: yolov8n.pt)
  --save             Video kayit
```

## Beklenen Performans

### Bilgisayarinizda

| Metrik | Deger |
|--------|-------|
| **FPS** | 30-40 |
| **Tracking Orani** | %65-75 |
| **Latency** | <50ms |
| **CPU Kullanimi** | %40-60 |

### Raspberry Pi 5'te

| Metrik | Deger |
|--------|-------|
| **FPS** | 18-25 |
| **Tracking Orani** | %55-65 |
| **Latency** | <100ms |
| **CPU Kullanimi** | %70-90 |

## Diger Modlarla Karsilastirma

| Mod | FPS | Dogruluk | Kullanim |
|-----|-----|----------|----------|
| **MOSSE Fast** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | Hiz kritik |
| **KCF Balanced** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | Genel kullanim |
| **CSRT Quality** | ⭐⭐ | ⭐⭐⭐⭐⭐ | Dogruluk kritik |

## Optimizasyon Detaylari

### 1. Model Ayarlari

```python
conf_threshold: 0.4    # Dusuk esik - daha fazla tespit
iou_threshold: 0.4     # Agresif NMS
input_size: (416, 416) # Hizli islem
```

### 2. Tracker Ayarlari

```python
tracker_type: 'MOSSE'      # En hizli
max_bbox_change: 0.6       # Esnek
min_bbox_area: 50          # Dusuk minimum
max_consecutive_failures: 2 # Hizli yeniden tespit
```

### 3. Kalman Ayarlari

```python
process_noise: 0.01        # Yuksek - hizli uyum
measurement_noise: 0.1     # Standart
prediction_steps: 3        # Az tahmin - hizli
```

### 4. Gorsellestime

```python
trajectory_length: 20      # Kisa iz
show_predictions: True     # Sadece 3 frame
bbox_thickness: 2          # Ince cizgi
```

## Performans Ipuclari

### Daha Fazla Hiz Icin

1. **Daha Kucuk Input**
   ```python
   # src/main_mosse_fast.py
   frame_process = cv2.resize(frame, (320, 320))  # 416 -> 320
   ```

2. **Trajectory Kapatma**
   ```python
   # Trajectory cizimini kapat
   # frame_vis = self.tracker.draw_trajectory(...)
   ```

3. **Prediction Kapatma**
   ```python
   # Kalman prediction'i kapat
   # predictions = self.kalman.predict_ahead(3)
   ```

### Daha Fazla Dogruluk Icin

1. **Yuksek Guven Esigi**
   ```bash
   # src/main_mosse_fast.py'de conf_threshold'u artir
   conf_threshold=0.5  # 0.4 -> 0.5
   ```

2. **Daha Buyuk Input**
   ```python
   frame_process = cv2.resize(frame, (512, 512))  # 416 -> 512
   ```

3. **Daha Uzun Trajectory**
   ```python
   trajectory_length=30  # 20 -> 30
   ```

## Sorun Giderme

### FPS Dusuk (<25)

**Cozum 1**: Daha kucuk input
```python
frame_process = cv2.resize(frame, (320, 320))
```

**Cozum 2**: Gorsellestime azalt
```python
# Trajectory ve prediction'i kapat
```

**Cozum 3**: Frame atlama
```python
if self.frame_count % 2 == 0:  # Her 2 frame'de bir isle
    continue
```

### Tracking Orani Dusuk (<50%)

**Cozum 1**: Dusuk guven esigi
```python
conf_threshold=0.3  # 0.4 -> 0.3
```

**Cozum 2**: Daha esnek bbox
```python
max_bbox_change=0.8  # 0.6 -> 0.8
```

**Cozum 3**: KCF kullan
```bash
python src\main.py --tracker KCF --use-kalman --save
```

### Cok Fazla False Positive

**Cozum**: Yuksek guven esigi
```python
conf_threshold=0.5  # 0.4 -> 0.5
```

## Test Sonuclari

Her test sonrasi:

```
testler/
└── test-X/
    ├── test_results.json    # Detayli veri
    ├── OZET.txt             # Okunabilir rapor
    └── mosse_fast_*.mp4     # Video
```

### Sonuclari Analiz Et

```bash
# Tum testleri karsilastir
python analiz_testler.py

# Belirli bir test
python analiz_testler.py test-X
```

## Raspberry Pi Deployment

### 1. Model Transfer

```bash
# Bilgisayarinizda
scp src/main_mosse_fast.py pi@raspberrypi:/home/pi/uav_tracking/src/

# Model dosyasi
scp yolov8n.pt pi@raspberrypi:/home/pi/uav_tracking/
```

### 2. Raspberry Pi'de Calistirma

```bash
# SSH ile baglan
ssh pi@raspberrypi

# Dizine git
cd /home/pi/uav_tracking

# Calistir
python src/main_mosse_fast.py --source 0 --save
```

### 3. Otomatik Baslangic

```bash
# systemd service olustur
sudo nano /etc/systemd/system/uav-tracking.service

[Unit]
Description=UAV Tracking System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/uav_tracking
ExecStart=/home/pi/uav_tracking/venv/bin/python src/main_mosse_fast.py --source 0 --save
Restart=always

[Install]
WantedBy=multi-user.target

# Servisi aktive et
sudo systemctl enable uav-tracking
sudo systemctl start uav-tracking
```

## Ozet

**MOSSE Optimize Mod**:
- ✅ Maksimum hiz (30-40 FPS)
- ✅ Dusuk latency (<50ms)
- ✅ Raspberry Pi uyumlu
- ✅ Gerçek zamanli tracking
- ✅ Otomatik test kaydi

**Kullanim**:
```bash
run_mosse_fast.bat
```

**Basarilar!** 🚀
