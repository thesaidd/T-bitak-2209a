# İHA Tespit ve Takip Sistemi - Teknik Dokümantasyon

## Sistem Mimarisi

```
┌─────────────────────────────────────────────────────────────────┐
│                    İHA TESPİT VE TAKİP SİSTEMİ                  │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────┐
│  Görüntü Kaynağı│
│  - Ekran        │
│  - Video        │──────┐
│  - Webcam       │      │
└─────────────────┘      │
                         ▼
                ┌─────────────────┐
                │ Screen Capture  │
                │   (MSS/OpenCV)  │
                └─────────────────┘
                         │
                         ▼
                ┌─────────────────┐
                │   Frame Buffer  │
                └─────────────────┘
                         │
                         ▼
        ┌────────────────┴────────────────┐
        │                                 │
        ▼                                 ▼
┌──────────────┐                  ┌──────────────┐
│   DETECTOR   │                  │   TRACKER    │
│   (YOLOv8)   │                  │  (OpenCV)    │
│              │                  │              │
│ - Tespit     │◄─────────────────│ - Takip      │
│ - Sınıfland. │   Hedef Kaybı    │ - Trajectory │
│ - Bbox       │                  │ - Velocity   │
└──────────────┘                  └──────────────┘
        │                                 │
        │                                 │
        └────────────────┬────────────────┘
                         │
                         ▼
                ┌─────────────────┐
                │ Kalman Filter   │
                │                 │
                │ - Smoothing     │
                │ - Prediction    │
                │ - Noise Filter  │
                └─────────────────┘
                         │
                         ▼
                ┌─────────────────┐
                │ Visualization   │
                │                 │
                │ - Bbox Draw     │
                │ - Trajectory    │
                │ - Info Overlay  │
                └─────────────────┘
                         │
                         ▼
                ┌─────────────────┐
                │  Output         │
                │  - Display      │
                │  - Video Save   │
                │  - Statistics   │
                └─────────────────┘
```

## Modül Detayları

### 1. Screen Capture (screen_capture.py)
**Görev**: Görüntü kaynağından frame yakalama

**Özellikler**:
- MSS ile ekran yakalama (30+ FPS)
- OpenCV ile video/webcam okuma
- Çoklu monitör desteği
- Bölge seçimi

**Çıktı**: BGR formatında numpy array

---

### 2. Detector (detector.py)
**Görev**: YOLOv8 ile nesne tespiti

**Özellikler**:
- Ultralytics YOLOv8 entegrasyonu
- COCO dataset (airplane, bird)
- Özel model desteği
- Güven eşiği filtreleme
- Alan bazlı filtreleme

**Metrikler**:
- Inference süresi: ~50-100ms (CPU)
- mAP50: 37.3 (YOLOv8n)
- Tespit/frame: 0-5

**Çıktı**: 
```python
{
    'bbox': (x1, y1, x2, y2),
    'conf': 0.85,
    'class': 4,
    'name': 'airplane',
    'center': (cx, cy)
}
```

---

### 3. Tracker (tracker.py)
**Görev**: Tespit edilen nesneyi takip etme

**Tracker Tipleri**:
| Tracker | Hız | Doğruluk | Kullanım |
|---------|-----|----------|----------|
| CSRT | Yavaş | Yüksek | Hassas takip |
| KCF | Orta | Orta | Dengeli |
| MOSSE | Hızlı | Düşük | Gerçek zamanlı |

**Özellikler**:
- Trajectory tracking (30 frame)
- Velocity hesaplama
- Otomatik re-initialization
- Multi-object desteği

**Performans**:
- Update süresi: ~10-30ms
- Başarı oranı: %70-90

---

### 4. Kalman Filter (kalman_filter.py)
**Görev**: Hareket tahmini ve gürültü filtreleme

**State Vector**: [x, y, vx, vy]
- x, y: Konum
- vx, vy: Hız

**Matrisler**:
```
F (State Transition):
[1  0  dt  0]
[0  1  0  dt]
[0  0  1   0]
[0  0  0   1]

H (Measurement):
[1  0  0  0]
[0  1  0  0]
```

**Parametreler**:
- Process noise (Q): 1e-3
- Measurement noise (R): 1e-1
- Prediction steps: 5 frame

**Performans**:
- Hata azaltma: %30-50
- Tahmin doğruluğu: ±10 piksel

---

### 5. Main (main.py)
**Görev**: Tüm modülleri koordine etme

**İş Akışı**:
```
1. Frame Capture
   ↓
2. Mode Check
   ↓
3a. DETECTION MODE          3b. TRACKING MODE
    - YOLOv8 detect             - Tracker update
    - Best detection            - Kalman update
    - Init tracker              - Trajectory draw
    ↓                           ↓
4. Visualization
   ↓
5. Display & Save
```

**Durum Makinesi**:
```
DETECTION ←──────────┐
    │                │
    │ Tespit OK      │ Hedef Kaybı
    ↓                │
TRACKING ────────────┘
```

## Veri Akışı

### Frame Processing Pipeline

```
Raw Frame (1920x1080)
    │
    ├─► Resize (640x480)        [Performance]
    │
    ├─► YOLOv8 Inference        [Detection]
    │   └─► NMS Filtering
    │       └─► Class Filtering
    │
    ├─► Tracker Update          [Tracking]
    │   └─► Bbox Prediction
    │
    ├─► Kalman Update           [Filtering]
    │   └─► Position Smoothing
    │   └─► Future Prediction
    │
    └─► Visualization           [Output]
        └─► Bbox Draw
        └─► Trajectory Draw
        └─► Info Overlay
```

## Performans Optimizasyonu

### CPU Optimizasyonu
1. **Input Resize**: 640x480 (YOLOv8 için optimal)
2. **Frame Skip**: Her N frame'de tespit (opsiyonel)
3. **Tracker Kullanımı**: Tespit yerine tracking (10x hızlı)
4. **Batch Processing**: Devre dışı (gerçek zamanlı için)

### Raspberry Pi 5 Optimizasyonu
1. **Model**: YOLOv8n (en hafif)
2. **Quantization**: INT8 TFLite
3. **Input Size**: 416x416 veya 320x320
4. **Tracker**: MOSSE (en hızlı)

**Beklenen FPS**:
- YOLOv8n + MOSSE: 15-20 FPS
- YOLOv8n + KCF: 12-15 FPS
- YOLOv8n + CSRT: 8-12 FPS

## Konfigürasyon

### Tespit Hassasiyeti
```python
# Yüksek hassasiyet (daha az false positive)
conf_threshold = 0.7

# Dengeli
conf_threshold = 0.5

# Yüksek recall (daha fazla tespit)
conf_threshold = 0.3
```

### Tracking Kararlılığı
```python
# Kararlı takip (yavaş)
tracker_type = 'CSRT'

# Dengeli
tracker_type = 'KCF'

# Hızlı takip (az kararlı)
tracker_type = 'MOSSE'
```

### Kalman Agresifliği
```python
# Agresif filtreleme
process_noise = 1e-4
measurement_noise = 1e-1

# Dengeli
process_noise = 1e-3
measurement_noise = 1e-1

# Ölçüme güven
process_noise = 1e-2
measurement_noise = 1e-2
```

## Test Senaryoları

### Senaryo 1: Statik Kamera
- **Kaynak**: Ekran/Video
- **Mod**: Detection → Tracking
- **Zorluk**: Kolay
- **Beklenen FPS**: 15-20

### Senaryo 2: Hareketli Kamera
- **Kaynak**: Webcam/Video
- **Mod**: Detection ağırlıklı
- **Zorluk**: Orta
- **Beklenen FPS**: 10-15

### Senaryo 3: Çoklu Hedef
- **Kaynak**: Video
- **Mod**: Multi-tracker
- **Zorluk**: Zor
- **Beklenen FPS**: 5-10

## Hata Durumları ve Çözümler

### 1. Tracker Kaybı
**Sebep**: Hızlı hareket, oklüzyon
**Çözüm**: Otomatik detection modu

### 2. False Positive
**Sebep**: Düşük confidence threshold
**Çözüm**: Threshold artır, alan filtresi

### 3. Düşük FPS
**Sebep**: Ağır model, büyük input
**Çözüm**: Model küçült, resize input

### 4. Titrek Tracking
**Sebep**: Kalman noise ayarları
**Çözüm**: Process noise artır

## Metrikler ve Logging

### Toplanan Metrikler
- Frame sayısı
- Tespit sayısı
- Tracking başarı oranı
- Ortalama FPS
- Inference süresi
- Tracker süresi

### Log Formatı
```
[TIMESTAMP] [LEVEL] [MODULE] Message
2026-01-21 15:30:45 INFO DETECTOR Detection: 2 objects, conf: 0.85
2026-01-21 15:30:45 INFO TRACKER Tracking: SUCCESS, speed: 15.3 px/f
```

## Gelecek Geliştirmeler

### Aşama 2: Simülasyon Entegrasyonu
- [ ] Gazebo/AirSim entegrasyonu
- [ ] MAVLink protokolü
- [ ] Gimbal kontrol
- [ ] Otopilot entegrasyonu

### Aşama 3: Gelişmiş Özellikler
- [ ] Multi-target tracking
- [ ] 3D konum tahmini
- [ ] Collision avoidance
- [ ] Swarm coordination

### Aşama 4: Optimizasyon
- [ ] TensorRT entegrasyonu
- [ ] CUDA acceleration
- [ ] Model pruning
- [ ] Knowledge distillation

## Kaynaklar

### Akademik
- YOLOv8: https://arxiv.org/abs/2305.09972
- Kalman Filter: Welch & Bishop, 2006
- Visual Tracking: Henriques et al., 2015

### Implementasyon
- Ultralytics: https://github.com/ultralytics/ultralytics
- OpenCV: https://github.com/opencv/opencv
- FilterPy: https://github.com/rlabbe/filterpy
