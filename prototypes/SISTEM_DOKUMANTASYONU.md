# IHA Tespit ve Takip Sistemi - Kapsamli Sistem Dokumantasyonu

## Icindekiler

1. [Sistem Genel Bakis](#sistem-genel-bakis)
2. [Mimari ve Moduller](#mimari-ve-moduller)
3. [Veri Akisi](#veri-akisi)
4. [Adim Adim Calisma Prensibi](#adim-adim-calisma-prensibi)
5. [Optimizasyonlar](#optimizasyonlar)
6. [Kullanim Senaryolari](#kullanim-senaryolari)
7. [Sorun Giderme](#sorun-giderme)

---

## Sistem Genel Bakis

### Amac

IHA (Insansiz Hava Araci) tespit ve takip sistemi, kamera veya ekran goruntulerinden gercek zamanli olarak IHA'lari tespit edip takip eder.

### Temel Ozellikler

- **Gercek Zamanli Tespit**: YOLOv8 ile hizli nesne tespiti
- **Robust Tracking**: OpenCV tracker'lari ile stabil takip
- **Hareket Tahmini**: Kalman Filter ile gelecek pozisyon tahmini
- **Otomatik Test Kaydi**: Her test otomatik olarak kaydedilir
- **Coklu Tracker Destegi**: CSRT, KCF, MOSSE, MIL
- **Performans Profilleri**: Hiz/Kalite dengesi icin hazir profiller

### Sistem Gereksinimleri

**Donanim**:
- CPU: Intel i5 veya ust (Raspberry Pi 5 destekli)
- RAM: 4GB minimum, 8GB onerilir
- Ekran: 1920x1080 (diger cozunurlukler de desteklenir)

**Yazilim**:
- Python 3.8+
- Windows 10/11, Linux, macOS
- Webcam veya ekran yakalama destegi

---

## Mimari ve Moduller

### 1. Moduler Yapi

```
src/
├── main.py                 # Ana program (genel kullanim)
├── main_mosse_fast.py      # MOSSE optimize mod
├── config.py               # Konfigurasyon ve profiller
├── screen_capture.py       # Goruntu yakalama
├── detector.py             # YOLOv8 tespit
├── tracker.py              # OpenCV tracking (temel)
├── robust_tracker.py       # Gelismis tracking (stabil)
├── kalman_filter.py        # Kalman filter
└── test_logger.py          # Test kayit sistemi
```

### 2. Modul Detaylari

#### 2.1. screen_capture.py - Goruntu Yakalama

**Amac**: Ekran, webcam veya video dosyasindan goruntu yakalar.

**Temel Fonksiyonlar**:
```python
class ScreenCapture:
    def __init__(self, monitor=1):
        # Ekran yakalama baslatir
        
    def capture(self):
        # Bir frame yakalar
        # Returns: numpy array (BGR format)
        
    def close(self):
        # Kaynaklari serbest birakir
```

**Desteklenen Kaynaklar**:
- `screen`: Bilgisayar ekrani
- `0, 1, 2...`: Webcam indeksi
- `video.mp4`: Video dosyasi

**Ornek Kullanim**:
```python
from screen_capture import create_capture

# Ekran yakalama
capture = create_capture('screen')
frame = capture.capture()  # 1920x1080 BGR image

# Webcam
capture = create_capture(0)
frame = capture.capture()

# Video
capture = create_capture('test_video.mp4')
frame = capture.capture()
```

---

#### 2.2. detector.py - YOLOv8 Tespit

**Amac**: YOLOv8 modeli ile nesneleri tespit eder.

**Temel Fonksiyonlar**:
```python
class UAVDetector:
    def __init__(self, model_path='yolov8n.pt', conf_threshold=0.5):
        # YOLOv8 modelini yukler
        
    def detect(self, frame):
        # Frame'de nesneleri tespit eder
        # Returns: List[Dict] - tespit listesi
        
    def get_best_detection(self, detections):
        # En yuksek guvenli tespiti dondurur
        
    def draw_detections(self, frame, detections):
        # Tespitleri frame uzerine cizer
```

**Tespit Formati**:
```python
detection = {
    'bbox': (x1, y1, x2, y2),  # Bounding box koordinatlari
    'center': (cx, cy),         # Merkez noktasi
    'confidence': 0.85,         # Guven skoru (0-1)
    'class_id': 4,              # Sinif ID (COCO)
    'name': 'airplane'          # Sinif adi
}
```

**Ornek Kullanim**:
```python
from detector import UAVDetector

detector = UAVDetector(
    model_path='yolov8n.pt',
    conf_threshold=0.5,
    target_classes=[4, 14]  # airplane, bird
)

# Tespit
detections = detector.detect(frame)

# En iyi tespit
if detections:
    best = detector.get_best_detection(detections)
    print(f"Tespit: {best['name']}, Guven: {best['confidence']:.2f}")
```

**YOLOv8 Modelleri**:
- `yolov8n.pt`: Nano (en hizli, 6MB)
- `yolov8s.pt`: Small (dengeli, 22MB)
- `yolov8m.pt`: Medium (dogru, 52MB)
- `yolov8l.pt`: Large (en dogru, 87MB)

---

#### 2.3. tracker.py & robust_tracker.py - Nesne Takibi

**Amac**: Tespit edilen nesneyi frame'ler arasinda takip eder.

**Temel Tracker (tracker.py)**:
```python
class ObjectTracker:
    def __init__(self, tracker_type='CSRT'):
        # Tracker olusturur
        
    def init(self, frame, bbox):
        # Tracker'i baslatir
        # bbox: (x1, y1, x2, y2)
        
    def update(self, frame):
        # Tracker'i gunceller
        # Returns: (success, bbox)
```

**Gelismis Tracker (robust_tracker.py)**:
```python
class RobustTracker(ObjectTracker):
    # Ek ozellikler:
    # - Bbox validasyonu
    # - Hareket sinirlamasi
    # - Temporal smoothing
    # - Guven skoru tahmini
    
    def get_confidence(self):
        # Tracker guven skorunu dondurur (0-1)
```

**Tracker Tipleri**:

| Tracker | Hiz | Dogruluk | Kullanim |
|---------|-----|----------|----------|
| **CSRT** | ⭐⭐ | ⭐⭐⭐⭐⭐ | Yuksek dogruluk gerekli |
| **KCF** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | Dengeli performans |
| **MOSSE** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | Maksimum hiz |
| **MIL** | ⭐⭐⭐ | ⭐⭐⭐ | Genel amacli |

**Ornek Kullanim**:
```python
from robust_tracker import RobustTracker

tracker = RobustTracker(
    tracker_type='MOSSE',
    max_bbox_change=0.6,
    min_bbox_area=50
)

# Baslatma
tracker.init(frame, bbox=(100, 200, 150, 250))

# Guncelleme
success, new_bbox = tracker.update(next_frame)

if success:
    print(f"Tracking basarili: {new_bbox}")
    confidence = tracker.get_confidence()
    print(f"Guven: {confidence:.2f}")
```

---

#### 2.4. kalman_filter.py - Hareket Tahmini

**Amac**: Kalman Filter ile nesnenin gelecekteki pozisyonunu tahmin eder.

**Temel Fonksiyonlar**:
```python
class UAVKalmanFilter:
    def __init__(self, dt=1.0, process_noise=0.001, measurement_noise=0.1):
        # Kalman filter baslatir
        # State: [x, y, vx, vy]
        
    def init(self, x, y):
        # Baslangic pozisyonu ayarlar
        
    def update(self, measurement):
        # Olcum ile gunceller
        # Returns: (x, y) - filtrelenmis pozisyon
        
    def predict_ahead(self, steps=5):
        # Gelecek pozisyonlari tahmin eder
        # Returns: List[(x, y)]
```

**Kalman Filter Parametreleri**:
- `dt`: Zaman adimi (1.0 = 1 frame)
- `process_noise`: Surec gurultusu (dusuk = yavas degisim)
- `measurement_noise`: Olcum gurultusu (dusuk = olcume guven)

**Ornek Kullanim**:
```python
from kalman_filter import UAVKalmanFilter

kalman = UAVKalmanFilter(
    dt=1.0,
    process_noise=0.01,  # Hizli degisimlere uyum
    measurement_noise=0.1
)

# Baslangic
kalman.init(x=100, y=200)

# Guncelleme
filtered_pos = kalman.update((105, 205))

# Tahmin (5 frame ilerisi)
predictions = kalman.predict_ahead(steps=5)
for i, (px, py) in enumerate(predictions):
    print(f"Frame +{i+1}: ({px:.1f}, {py:.1f})")
```

---

#### 2.5. test_logger.py - Test Kayit Sistemi

**Amac**: Her test otomatik olarak kaydeder ve analiz edilebilir hale getirir.

**Temel Fonksiyonlar**:
```python
class TestLogger:
    def __init__(self, base_dir='testler'):
        # Test logger baslatir
        # Otomatik olarak test-X klasoru olusturur
        
    def log_parameters(self, params):
        # Test parametrelerini kaydeder
        
    def log_frame_stats(self, frame_num, detections, tracking, fps):
        # Frame bazinda istatistik kaydeder
        
    def log_event(self, event_type, message):
        # Onemli olaylari kaydeder
        
    def finalize(self):
        # Test sonuclarini kaydeder
        # - test_results.json
        # - OZET.txt
```

**Kaydedilen Veriler**:
```
testler/
└── test-X/
    ├── test_results.json    # Detayli JSON veri
    ├── OZET.txt             # Okunabilir ozet
    └── tracking_*.mp4       # Video (varsa)
```

**Ornek Kullanim**:
```python
from test_logger import TestLogger

logger = TestLogger()

# Parametreleri kaydet
logger.log_parameters({
    'tracker': 'MOSSE',
    'use_kalman': True,
    'source': 'screen'
})

# Frame istatistikleri
logger.log_frame_stats(
    frame_num=100,
    detections=1,
    tracking=True,
    fps=30.5
)

# Olay kaydi
logger.log_event('TRACKER_INIT', 'Tracker baslatildi')

# Sonuclari kaydet
logger.finalize()
```

---

## Veri Akisi

### 1. Genel Veri Akisi

```
[Ekran/Webcam/Video]
        ↓
[screen_capture.py]
        ↓
    Frame (1920x1080)
        ↓
    ┌───────┴───────┐
    ↓               ↓
frame_original  frame_detect (416x416)
    ↓               ↓
[Tracking]      [Detection]
    ↓               ↓
  Bbox          Detections
    ↓               ↓
    └───────┬───────┘
            ↓
    [Gorsellestirme]
            ↓
    [Video Kayit]
            ↓
    [Test Logger]
```

### 2. Detection Modu Veri Akisi

```
Frame (416x416)
    ↓
[YOLOv8 Model]
    ↓
Raw Detections
    ↓
[NMS - Non-Max Suppression]
    ↓
Filtered Detections
    ↓
[Confidence Filtering]
    ↓
Valid Detections
    ↓
[Best Detection Selection]
    ↓
Best Detection
    ↓
[Bbox Scale Transform] (416 → 1920)
    ↓
Scaled Bbox
    ↓
[Tracker Init]
```

### 3. Tracking Modu Veri Akisi

```
Frame (1920x1080)
    ↓
[OpenCV Tracker Update]
    ↓
New Bbox
    ↓
[Bbox Validation]
    ↓
Valid? ──No──→ [Detection Mode]
    ↓ Yes
[Temporal Smoothing]
    ↓
Smoothed Bbox
    ↓
[Kalman Filter Update]
    ↓
Filtered Position
    ↓
[Kalman Prediction]
    ↓
Future Positions
    ↓
[Visualization]
```

---

## Adim Adim Calisma Prensibi

### Adim 1: Sistem Baslatma

```python
# 1. Konfigurasyon yukleme
import config
config.apply_profile('mosse_fast')

# 2. Goruntu kaynagi baslatma
from screen_capture import create_capture
capture = create_capture('screen')

# 3. Detector baslatma
from detector import UAVDetector
detector = UAVDetector(
    model_path='yolov8n.pt',
    conf_threshold=0.4
)

# 4. Tracker baslatma
from robust_tracker import RobustTracker
tracker = RobustTracker(
    tracker_type='MOSSE',
    max_bbox_change=0.6
)

# 5. Kalman Filter baslatma
from kalman_filter import UAVKalmanFilter
kalman = UAVKalmanFilter(
    dt=1.0,
    process_noise=0.01
)

# 6. Test Logger baslatma
from test_logger import TestLogger
logger = TestLogger()
```

### Adim 2: Ana Dongu

```python
while running:
    # 1. Frame yakalama
    frame = capture.capture()
    
    # 2. Frame hazirlama
    frame_original = frame.copy()  # Tracking icin
    frame_detect = cv2.resize(frame, (416, 416))  # Detection icin
    
    # 3. Mod secimi
    if detection_mode:
        # DETECTION MODU
        # Adim 3'e git
    else:
        # TRACKING MODU
        # Adim 4'e git
```

### Adim 3: Detection Modu

```python
# 1. Tespit
detections = detector.detect(frame_detect)

# 2. Bbox olcekleme (416 → 1920)
scale_x = 1920 / 416
scale_y = 1080 / 416

for det in detections:
    x1, y1, x2, y2 = det['bbox']
    det['bbox'] = (
        int(x1 * scale_x),
        int(y1 * scale_y),
        int(x2 * scale_x),
        int(y2 * scale_y)
    )

# 3. En iyi tespit
if detections:
    best_det = detector.get_best_detection(detections)
    
    # 4. Tracker baslatma
    tracker.init(frame_original, best_det['bbox'])
    
    # 5. Kalman baslatma
    kalman.init(best_det['center'][0], best_det['center'][1])
    
    # 6. Tracking moduna gec
    detection_mode = False
```

### Adim 4: Tracking Modu

```python
# 1. Tracker guncelleme
success, bbox = tracker.update(frame_original)

if success:
    # 2. Kalman guncelleme
    center = tracker.last_center
    filtered_pos = kalman.update(center)
    
    # 3. Gelecek tahmini
    predictions = kalman.predict_ahead(steps=3)
    
    # 4. Gorsellestirme
    # - Bbox cizimi
    # - Trajectory cizimi
    # - Prediction cizimi
    
else:
    # Tracker kaybetti
    # Detection moduna don
    detection_mode = True
    tracker.reset()
    kalman.reset()
```

### Adim 5: Gorsellestirme

```python
# 1. Bbox cizimi
x1, y1, x2, y2 = bbox
cv2.rectangle(frame_vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

# 2. Trajectory cizimi
trajectory = tracker.get_trajectory()
for i in range(1, len(trajectory)):
    cv2.line(frame_vis, trajectory[i-1], trajectory[i], (255, 0, 0), 2)

# 3. Prediction cizimi
if predictions:
    for px, py in predictions:
        cv2.circle(frame_vis, (int(px), int(py)), 3, (0, 0, 255), -1)

# 4. Bilgi metni
cv2.putText(frame_vis, f"FPS: {fps:.1f}", (10, 30), ...)
cv2.putText(frame_vis, f"Mode: TRACKING", (10, 60), ...)
```

### Adim 6: Kayit ve Gosterim

```python
# 1. Test logger guncelleme
logger.log_frame_stats(
    frame_num=frame_count,
    detections=len(detections),
    tracking=tracker.is_tracking,
    fps=current_fps
)

# 2. Video kayit
if save_video:
    video_writer.write(frame_vis)

# 3. Ekranda gosterim
cv2.imshow('UAV Tracking', frame_vis)

# 4. Tus kontrolu
key = cv2.waitKey(1) & 0xFF
if key == 27:  # ESC
    break
```

### Adim 7: Sistem Kapatma

```python
# 1. Istatistik toplama
det_stats = detector.get_stats()
track_stats = tracker.get_stats()

# 2. Test sonuclarini kaydetme
logger.log_statistic('total_frames', det_stats['total_frames'])
logger.log_statistic('tracking_success_rate', success_rate)
logger.finalize()

# 3. Video kapatma
if video_writer:
    video_writer.release()

# 4. Kaynaklari serbest birakma
capture.close()
cv2.destroyAllWindows()
```

---

## Optimizasyonlar

### 1. MOSSE Optimize Mod

**Hedef**: Maksimum hiz

**Optimizasyonlar**:
```python
# 1. En hizli tracker
tracker_type = 'MOSSE'

# 2. Kucuk detection boyutu
detection_size = (416, 416)  # Yerine 640x640

# 3. Dusuk guven esigi
conf_threshold = 0.4  # Yerine 0.5

# 4. Kisa trajectory
trajectory_length = 20  # Yerine 30

# 5. Az prediction
prediction_steps = 3  # Yerine 5

# 6. Esnek bbox validasyonu
max_bbox_change = 0.6  # Yerine 0.3
min_bbox_area = 50  # Yerine 100
```

**Beklenen Performans**:
- FPS: 25-35
- Tracking Orani: %65-75
- Latency: <50ms

### 2. CSRT Kalite Mod

**Hedef**: Maksimum dogruluk

**Optimizasyonlar**:
```python
# 1. En dogru tracker
tracker_type = 'CSRT'

# 2. Buyuk detection boyutu
detection_size = (640, 480)

# 3. Yuksek guven esigi
conf_threshold = 0.5

# 4. Uzun trajectory
trajectory_length = 30

# 5. Fazla prediction
prediction_steps = 5

# 6. Siki bbox validasyonu
max_bbox_change = 0.3
min_bbox_area = 100
```

**Beklenen Performans**:
- FPS: 15-20
- Tracking Orani: %80-90
- Latency: <100ms

### 3. Goruntu Kalitesi Optimizasyonu

**Sorun**: Resize ile goruntu bulaniklasiyor

**Cozum**: Iki frame sistemi
```python
# Orijinal frame (tracking ve gorsellestirme)
frame_original = frame.copy()  # 1920x1080

# Kucuk frame (sadece detection)
frame_detect = cv2.resize(frame, (416, 416))

# Detection
detections = detector.detect(frame_detect)

# Bbox olcekleme
for det in detections:
    det['bbox'] = scale_bbox(det['bbox'], scale_x, scale_y)

# Tracking (orijinal boyutta)
success, bbox = tracker.update(frame_original)

# Sonuc: Net goruntu + Hizli detection
```

---

## Kullanim Senaryolari

### Senaryo 1: Hizli Test (Ekran)

**Amac**: Ekrandaki videodan hizli test

**Adimlar**:
```bash
# 1. YouTube'da ucak videosu ac
# 2. Tam ekran yap
# 3. Sistemi baslat
run_mosse_fast.bat

# Veya manuel:
python src\main_mosse_fast.py --source screen --save
```

**Beklenen Sonuc**:
- FPS: 25-35
- Net goruntu
- Otomatik tracking
- Video kaydi: results/mosse_fast_*.mp4
- Test kaydi: testler/test-X/

### Senaryo 2: Webcam ile Canli Test

**Amac**: Webcam'den gercek zamanli test

**Adimlar**:
```bash
# Webcam index 0
python src\main_mosse_fast.py --source 0 --save

# Veya ikinci webcam
python src\main_mosse_fast.py --source 1 --save
```

**Ipuclari**:
- Iyi aydinlatma kullanin
- Kamera sabit tutun
- Hedef net gorunsun

### Senaryo 3: Video Dosyasi Analizi

**Amac**: Kayitli videoyu analiz et

**Adimlar**:
```bash
python src\main_mosse_fast.py --source test_video.mp4 --save
```

**Avantajlar**:
- Tekrarlanabilir testler
- Ayni video ile farkli tracker'lari karsilastir
- Offline analiz

### Senaryo 4: Tracker Karsilastirmasi

**Amac**: En iyi tracker'i bul

**Adimlar**:
```bash
# 1. Hizli test menusu
test_hizli.bat

# 2. Secenek 6: Tum tracker'lari karsilastir

# 3. Sonuclari analiz et
python analiz_testler.py
```

**Cikti**:
```
Tracker    FPS    Track%    Kullanim
CSRT       18.5   85.2%     Yuksek dogruluk
KCF        25.1   78.5%     Dengeli
MOSSE      32.4   65.3%     Maksimum hiz
```

### Senaryo 5: Raspberry Pi Deployment

**Amac**: Raspberry Pi'de calistir

**Adimlar**:
```bash
# 1. Dosyalari transfer et
scp -r src/ pi@raspberrypi:/home/pi/uav_tracking/

# 2. SSH ile baglan
ssh pi@raspberrypi

# 3. Calistir
cd /home/pi/uav_tracking
python src/main_mosse_fast.py --source 0 --save
```

**Beklenen Performans** (Raspberry Pi 5):
- FPS: 18-25
- Tracking: %55-65
- Latency: <100ms

---

## Sorun Giderme

### Sorun 1: Dusuk FPS (<15)

**Olasi Nedenler**:
- Agir tracker (CSRT)
- Buyuk input boyutu
- Cok fazla gorsellestirme

**Cozumler**:
```bash
# Cozum 1: MOSSE kullan
python src\main_mosse_fast.py --source screen --save

# Cozum 2: Daha kucuk input
# main_mosse_fast.py'de:
frame_detect = cv2.resize(frame, (320, 320))

# Cozum 3: Gorsellestirme azalt
# Trajectory ve prediction'i kapat
```

### Sorun 2: Tracking Kaybi (<%50)

**Olasi Nedenler**:
- Hizli hareket
- Okluzyon (nesne gizleniyor)
- Dusuk guven esigi

**Cozumler**:
```bash
# Cozum 1: CSRT kullan
python src\main.py --tracker CSRT --use-kalman --save

# Cozum 2: Dusuk guven esigi
python src\main.py --tracker MOSSE --conf 0.3 --save

# Cozum 3: Esnek bbox
# robust_tracker.py'de:
max_bbox_change=0.8
```

### Sorun 3: Goruntu Bulanik

**Neden**: Resize ile kalite kaybi

**Cozum**: MOSSE optimize mod kullan (zaten duzeltildi)
```bash
run_mosse_fast.bat
```

### Sorun 4: Cok Fazla False Positive

**Neden**: Dusuk guven esigi

**Cozum**: Yuksek esik kullan
```bash
python src\main.py --conf 0.7 --use-kalman --save
```

### Sorun 5: "Tracker baslatilamadi" Hatasi

**Neden**: Bbox validasyonu cok siki

**Cozum**: Zaten duzeltildi (tracker.py'de validasyon eklendi)

**Manuel Cozum**:
```python
# robust_tracker.py'de:
min_bbox_area=25  # 50 -> 25
```

---

## Ozet

### Sistem Akisi

```
1. Baslatma
   ├─ Konfigurasyon yukleme
   ├─ Moduller baslatma
   └─ Test logger baslatma

2. Ana Dongu
   ├─ Frame yakalama
   ├─ Detection/Tracking
   ├─ Gorsellestirme
   ├─ Kayit
   └─ Gosterim

3. Kapatma
   ├─ Istatistik toplama
   ├─ Test kaydi
   └─ Kaynak temizleme
```

### Temel Komutlar

```bash
# Hizli test
run_mosse_fast.bat

# Manuel test
python src\main_mosse_fast.py --source screen --save

# Tracker karsilastirma
test_hizli.bat

# Sonuc analizi
python analiz_testler.py
```

### Performans Profilleri

| Profil | FPS | Dogruluk | Kullanim |
|--------|-----|----------|----------|
| MOSSE Fast | 25-35 | %65-75 | Hiz kritik |
| KCF Balanced | 20-25 | %70-80 | Genel kullanim |
| CSRT Quality | 15-20 | %80-90 | Dogruluk kritik |

### Dosya Yapisi

```
Tübitak/
├── src/                    # Kaynak kodlar
├── testler/                # Test sonuclari
├── results/                # Video kayitlari
├── models/                 # YOLOv8 modelleri
├── run_mosse_fast.bat      # Hizli baslangic
├── test_hizli.bat          # Test menusu
└── analiz_testler.py       # Sonuc analizi
```

---

**Sistem hazir! Hemen test edin!** 🚀
