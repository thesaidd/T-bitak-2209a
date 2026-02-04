# 🚁 İHA Tespit ve Takip Sistemi

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-00FFFF.svg)](https://github.com/ultralytics/ultralytics)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.10-green.svg)](https://opencv.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

İnsansız Hava Araçları (İHA) için gerçek zamanlı tespit ve takip sistemi. Raspberry Pi 5 uyumlu, YOLOv8 tabanlı, Kalman filter ile geliştirilmiş hareket tahmini.

## 🎯 Proje Hedefi

İHA-İHA savaşlarında kullanılmak üzere tasarlanmış, hedef İHA'yı tespit eden ve sürekli takip eden bir sistem geliştirmek.

### Proje Aşamaları

#### ✅ Aşama 1: Video/Ekran Testi (Mevcut)
- Sabit kamera görüntüsünden tespit
- Gerçek zamanlı takip
- Trajectory ve hız analizi
- Kalman filter ile tahmin

#### 🔄 Aşama 2: Simülasyon (Gelecek)
- Gazebo/AirSim entegrasyonu
- Aktif gimbal kontrol
- MAVLink protokolü
- İHA-İHA etkileşimi

## ✨ Özellikler

### 🔍 Tespit (Detection)
- **YOLOv8-Nano**: Raspberry Pi 5 için optimize edilmiş
- **COCO Dataset**: Airplane, bird sınıfları (başlangıç)
- **Özel Model Desteği**: Kendi dataset'iniz ile eğitim
- **Yüksek Doğruluk**: mAP50 37.3+

### 🎯 Takip (Tracking)
- **OpenCV Trackers**: CSRT, KCF, MOSSE
- **Trajectory Tracking**: 30 frame hareket izi
- **Velocity Estimation**: Gerçek zamanlı hız hesaplama
- **Auto Re-initialization**: Hedef kaybında otomatik yeniden tespit

### 📊 Kalman Filter
- **State Estimation**: [x, y, vx, vy]
- **Noise Filtering**: Gürültü azaltma
- **Future Prediction**: 5 frame ilerisi tahmin
- **Adaptive Mode**: Otomatik parametre ayarlama

### 🖥️ Görüntü Kaynakları
- **Ekran Yakalama**: MSS ile 30+ FPS
- **Video Dosyası**: MP4, AVI, vb.
- **Webcam**: Gerçek zamanlı kamera
- **Çoklu Monitör**: Monitör seçimi

## 🚀 Hızlı Başlangıç

### Gereksinimler
- Python 3.8+
- 4GB+ RAM
- ~2GB disk alanı

### Kurulum (5 Dakika)

#### Windows
```bash
# Kurulum scriptini çalıştır
install.bat

# Veya manuel kurulum
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

#### Linux/Mac
```bash
# Sanal ortam oluştur
python3 -m venv venv
source venv/bin/activate

# Bağımlılıkları yükle
pip install -r requirements.txt
```

### Test Et
```bash
# Sistem testini çalıştır
python test_system.py
```

### Çalıştır
```bash
# Hızlı başlatma (ekran + Kalman + kayıt)
run.bat  # Windows

# Veya manuel
python src/main.py --source screen --use-kalman --save
```

## 📖 Kullanım

### Temel Komutlar

```bash
# Ekran görüntüsünden tespit
python src/main.py --source screen

# Video dosyasından tespit
python src/main.py --source video.mp4

# Webcam'den tespit
python src/main.py --source 0

# Kalman filter ile gelişmiş takip
python src/main.py --source screen --use-kalman

# Sonuçları kaydet
python src/main.py --source screen --save

# Özel model kullan
python src/main.py --source screen --model custom.pt
```

### Parametreler

| Parametre | Açıklama | Varsayılan |
|-----------|----------|------------|
| `--source` | Görüntü kaynağı (screen/video/webcam) | screen |
| `--model` | YOLOv8 model dosyası | yolov8n.pt |
| `--conf` | Tespit güven eşiği (0-1) | 0.5 |
| `--tracker` | Tracker tipi (CSRT/KCF/MOSSE) | CSRT |
| `--use-kalman` | Kalman filter kullan | False |
| `--save` | Video kaydet | False |
| `--all-classes` | Tüm COCO sınıfları | False |

### Kontroller (Runtime)

| Tuş | Fonksiyon |
|-----|-----------|
| **SPACE** | Duraklat/Devam |
| **r** | Tracker'ı sıfırla |
| **d** | Tespit moduna geç |
| **t** | Takip moduna geç |
| **s** | Manuel bbox seçimi |
| **ESC** | Çıkış |

## 📁 Proje Yapısı

```
Tübitak/
├── src/                      # Kaynak kodlar
│   ├── main.py              # Ana program
│   ├── detector.py          # YOLOv8 tespit modülü
│   ├── tracker.py           # OpenCV takip modülü
│   ├── kalman_filter.py     # Kalman filter
│   ├── screen_capture.py    # Ekran/video yakalama
│   └── config.py            # Konfigürasyon
├── models/                   # Eğitilmiş modeller
├── data/                     # Test videoları
├── results/                  # Çıktı videoları
├── requirements.txt          # Python bağımlılıkları
├── install.bat              # Kurulum scripti (Windows)
├── run.bat                  # Hızlı başlatma (Windows)
├── test_system.py           # Test suite
├── train_custom_model.py    # Model eğitim scripti
├── README.md                # Bu dosya
├── QUICKSTART.md            # Hızlı başlangıç kılavuzu
├── TECHNICAL.md             # Teknik dokümantasyon
└── CUSTOM_TRAINING.md       # Özel model eğitim rehberi
```

## 📊 Performans

### Raspberry Pi 5 Benchmark

| Model | FPS | mAP50 | Boyut | Kullanım |
|-------|-----|-------|-------|----------|
| YOLOv8n | 15-20 | 37.3 | 6.2MB | ✅ Önerilen |
| YOLOv8s | 8-12 | 44.9 | 21.5MB | Yüksek doğruluk |
| YOLOv8m | 3-5 | 50.2 | 49.7MB | Çok yavaş |

### Tracker Karşılaştırması

| Tracker | Hız | Doğruluk | Kullanım |
|---------|-----|----------|----------|
| CSRT | Yavaş | ⭐⭐⭐⭐⭐ | Hassas takip |
| KCF | Orta | ⭐⭐⭐⭐ | Dengeli |
| MOSSE | Hızlı | ⭐⭐⭐ | Gerçek zamanlı |

## 🎓 Özel Model Eğitimi

Kendi İHA dataset'iniz ile model eğitmek için:

```bash
# 1. Dataset yapısını oluştur
python train_custom_model.py setup

# 2. Dataset'i hazırla (images + labels)
# Detaylar için: CUSTOM_TRAINING.md

# 3. Eğitimi başlat
python train_custom_model.py train --data dataset/data.yaml --epochs 100

# 4. Modeli kullan
python src/main.py --source screen --model runs/train/uav_detector/weights/best.pt
```

Detaylı rehber: **[CUSTOM_TRAINING.md](CUSTOM_TRAINING.md)**

## 📚 Dokümantasyon

- **[QUICKSTART.md](QUICKSTART.md)**: Hızlı başlangıç kılavuzu
- **[TECHNICAL.md](TECHNICAL.md)**: Teknik dokümantasyon ve mimari
- **[CUSTOM_TRAINING.md](CUSTOM_TRAINING.md)**: Özel dataset eğitim rehberi

## 🔧 Konfigürasyon

Tüm ayarlar `src/config.py` dosyasında:

```python
# Model ayarları
MODEL_CONFIG = {
    'confidence_threshold': 0.5,
    'target_classes': {4: 'airplane', 14: 'bird'}
}

# Tracker ayarları
TRACKER_CONFIG = {
    'tracker_type': 'CSRT',
    'reinit_threshold': 5
}

# Kalman filter ayarları
KALMAN_CONFIG = {
    'use_kalman': True,
    'prediction_steps': 5
}
```

## 🐛 Sorun Giderme

### Düşük FPS
```python
# config.py'de input size'ı küçült
PERFORMANCE_CONFIG = {
    'input_size': (416, 416)  # 640x480 yerine
}
```

### Tespit Yapılmıyor
```bash
# Confidence threshold'u düşür
python src/main.py --source screen --conf 0.3

# Tüm sınıfları aktive et
python src/main.py --source screen --all-classes
```

### Tracker Kaybediyor
```bash
# Daha güçlü tracker kullan
python src/main.py --source screen --tracker CSRT

# Kalman filter ekle
python src/main.py --source screen --use-kalman
```

## 🛠️ Geliştirme Yol Haritası

### Aşama 2: Simülasyon (Q2 2026)
- [ ] Gazebo entegrasyonu
- [ ] MAVLink protokolü
- [ ] Gimbal kontrol
- [ ] ROS2 node

### Aşama 3: Gelişmiş Özellikler (Q3 2026)
- [ ] Multi-target tracking
- [ ] 3D konum tahmini
- [ ] Collision avoidance
- [ ] Deep SORT entegrasyonu

### Aşama 4: Optimizasyon (Q4 2026)
- [ ] TensorRT acceleration
- [ ] Model quantization
- [ ] Edge TPU desteği
- [ ] Real-time benchmarks

## 🤝 Katkıda Bulunma

Katkılarınızı bekliyoruz! Lütfen:
1. Fork yapın
2. Feature branch oluşturun (`git checkout -b feature/amazing-feature`)
3. Commit yapın (`git commit -m 'Add amazing feature'`)
4. Push edin (`git push origin feature/amazing-feature`)
5. Pull Request açın

## 📄 Lisans

Bu proje MIT lisansı altında lisanslanmıştır. Detaylar için [LICENSE](LICENSE) dosyasına bakın.

## 📞 İletişim

- **Proje**: TÜBİTAK İHA Tespit ve Takip Sistemi
- **Geliştirici**: Abdullah Gül Üniversitesi
- **E-posta**: [email protected]

## 🙏 Teşekkürler

- [Ultralytics](https://github.com/ultralytics/ultralytics) - YOLOv8
- [OpenCV](https://opencv.org/) - Computer Vision
- [FilterPy](https://github.com/rlabbe/filterpy) - Kalman Filter

---

**⭐ Projeyi beğendiyseniz yıldız vermeyi unutmayın!**
