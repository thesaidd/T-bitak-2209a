# 🚁 İHA Tespit ve Takip Sistemi - Hızlı Başlangıç

## 📋 Sistem Gereksinimleri

- **Python**: 3.8 veya üzeri
- **İşletim Sistemi**: Windows, Linux, macOS
- **RAM**: Minimum 4GB (8GB önerilir)
- **Disk**: ~2GB boş alan

## 🚀 Kurulum (5 Dakika)

### Adım 1: Kurulum Scriptini Çalıştır

**Windows:**
```bash
# install.bat dosyasına çift tıklayın
# VEYA
install.bat
```

**Linux/Mac:**
```bash
chmod +x install.sh
./install.sh
```

### Adım 2: Test Et

```bash
# Sanal ortamı aktive et
venv\Scripts\activate  # Windows
# source venv/bin/activate  # Linux/Mac

# Sistem testini çalıştır
python test_system.py
```

## 🎯 Kullanım

### Temel Kullanım

#### 1. Ekran Görüntüsünden Tespit
```bash
python src/main.py --source screen
```

#### 2. Video Dosyasından Tespit
```bash
python src/main.py --source video.mp4
```

#### 3. Webcam'den Tespit
```bash
python src/main.py --source 0
```

### Gelişmiş Kullanım

#### Kalman Filter ile Tahmin
```bash
python src/main.py --source screen --use-kalman
```

#### Video Kaydetme
```bash
python src/main.py --source screen --use-kalman --save
```

#### Özel Model Kullanma
```bash
python src/main.py --source screen --model custom.pt
```

#### Farklı Tracker
```bash
python src/main.py --source screen --tracker KCF
```

## ⌨️ Kontroller

Sistem çalışırken kullanabileceğiniz tuşlar:

| Tuş | Fonksiyon |
|-----|-----------|
| **SPACE** | Duraklat/Devam |
| **r** | Tracker'ı sıfırla |
| **d** | Tespit moduna geç |
| **t** | Takip moduna geç |
| **s** | Manuel bbox seçimi |
| **ESC** | Çıkış |

## 📊 Sistem Modları

### 1. Tespit Modu (DETECTION)
- YOLOv8 ile sürekli tespit
- Otomatik hedef seçimi
- Turuncu renk ile gösterilir

### 2. Takip Modu (TRACKING)
- OpenCV tracker ile hızlı takip
- Trajectory (iz) gösterimi
- Kalman filter ile tahmin (opsiyonel)
- Yeşil renk ile gösterilir

## 🎨 Görselleştirme

Ekranda gösterilen bilgiler:
- **Bbox**: Tespit edilen nesnenin kutusu
- **Confidence**: Güven skoru (0-1)
- **Trajectory**: Hareket izi (mavi çizgi)
- **Prediction**: Kalman tahmini (kırmızı noktalar)
- **FPS**: Saniyedeki frame sayısı
- **Speed**: Hareket hızı (piksel/frame)

## 🔧 Ayarlar

### config.py Dosyası

Tüm ayarlar `src/config.py` dosyasında:

```python
# Model ayarları
MODEL_CONFIG = {
    'confidence_threshold': 0.5,  # Tespit eşiği
    'target_classes': {4: 'airplane', 14: 'bird'}
}

# Tracker ayarları
TRACKER_CONFIG = {
    'tracker_type': 'CSRT',  # CSRT, KCF, MOSSE
}

# Kalman filter ayarları
KALMAN_CONFIG = {
    'use_kalman': True,
    'prediction_steps': 5,  # Kaç frame ilerisi
}
```

## 📈 Performans

### Raspberry Pi 5 Beklenen Performans

| Model | FPS | mAP | Boyut |
|-------|-----|-----|-------|
| YOLOv8n | 15-20 | 37.3 | 6.2MB |
| YOLOv8s | 8-12 | 44.9 | 21.5MB |

**Öneri**: Raspberry Pi 5 için YOLOv8n kullanın.

## 🐛 Sorun Giderme

### Kamera Açılamıyor
```bash
# Webcam index'ini kontrol edin
python src/main.py --source 1  # Farklı index deneyin
```

### Düşük FPS
```python
# config.py dosyasında
PERFORMANCE_CONFIG = {
    'resize_input': True,
    'input_size': (416, 416),  # Daha küçük boyut
}
```

### Model Bulunamıyor
```bash
# İlk çalıştırmada YOLOv8 otomatik indirilir
# İnternet bağlantınızı kontrol edin
```

### Tespit Yapılmıyor
```bash
# Güven eşiğini düşürün
python src/main.py --source screen --conf 0.3

# Veya tüm sınıfları aktive edin
python src/main.py --source screen --all-classes
```

## 📚 Örnekler

### Örnek 1: Basit Ekran Testi
```bash
# Ekranınızda bir uçak videosu açın (YouTube vb.)
# Sistemi başlatın
python src/main.py --source screen
```

### Örnek 2: Kalman Filter ile Takip
```bash
# Kalman filter ile gelişmiş tahmin
python src/main.py --source screen --use-kalman --tracker CSRT
```

### Örnek 3: Video Analizi ve Kayıt
```bash
# Video dosyasını analiz et ve sonucu kaydet
python src/main.py --source drone_video.mp4 --use-kalman --save
# Sonuç: results/tracking_YYYYMMDD_HHMMSS.mp4
```

## 🎓 Özel Model Eğitimi

Kendi İHA dataset'iniz ile model eğitmek için:

```bash
# 1. Dataset yapısını oluştur
python train_custom_model.py setup

# 2. Dataset'i hazırla (CUSTOM_TRAINING.md'ye bakın)

# 3. Eğitimi başlat
python train_custom_model.py train --data dataset/data.yaml --epochs 100

# 4. Eğitilmiş modeli kullan
python src/main.py --source screen --model runs/train/uav_detector/weights/best.pt
```

Detaylı bilgi için: **CUSTOM_TRAINING.md**

## 📁 Proje Yapısı

```
Tübitak/
├── src/                    # Ana kaynak kodlar
│   ├── main.py            # Ana program
│   ├── detector.py        # YOLOv8 tespit
│   ├── tracker.py         # Nesne takip
│   ├── kalman_filter.py   # Kalman filter
│   ├── screen_capture.py  # Ekran yakalama
│   └── config.py          # Ayarlar
├── models/                # Eğitilmiş modeller
├── data/                  # Test videoları
├── results/               # Çıktı videoları
├── requirements.txt       # Python bağımlılıkları
├── install.bat           # Kurulum scripti
├── run.bat               # Hızlı başlatma
├── test_system.py        # Test scripti
└── README.md             # Dokümantasyon
```

## 🔗 Faydalı Linkler

- **YOLOv8 Dokümantasyonu**: https://docs.ultralytics.com
- **OpenCV Trackers**: https://docs.opencv.org/4.x/d9/df8/group__tracking.html
- **Kalman Filter**: https://filterpy.readthedocs.io

## 💡 İpuçları

1. **İlk Kullanım**: Ekranınızda YouTube'dan bir uçak videosu açın ve test edin
2. **Performans**: Düşük FPS için input size'ı küçültün (config.py)
3. **Doğruluk**: Daha iyi tespit için confidence threshold'u ayarlayın
4. **Raspberry Pi**: TFLite formatına export edin (daha hızlı)

## 📞 Destek

Sorun yaşarsanız:
1. `test_system.py` çalıştırın
2. Log dosyalarını kontrol edin (`results/detection_log.txt`)
3. GitHub Issues'da sorun bildirin

## 🎉 Başarılı Kurulum Kontrolü

Sistem doğru çalışıyorsa:
- ✅ Test scripti tüm testleri geçti
- ✅ Ekran görüntüsü yakalanıyor
- ✅ YOLOv8 modeli yüklendi
- ✅ Tracker çalışıyor
- ✅ FPS > 10

**Sistem hazır! İyi testler! 🚀**
