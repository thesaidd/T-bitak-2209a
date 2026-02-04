# Özel İHA Dataset Eğitimi Rehberi

Bu rehber, kendi İHA dataset'inizi oluşturup YOLOv8 modelini eğitmek için adım adım talimatlar içerir.

## Aşama 1: Dataset Toplama

### 1.1 Görüntü Toplama
- **Kaynak**: Gerçek İHA uçuşları, simülasyon, internet
- **Çeşitlilik**: Farklı açılar, mesafeler, hava koşulları, arka planlar
- **Minimum**: 500-1000 görüntü (daha fazlası daha iyi)
- **Format**: JPG veya PNG

### 1.2 Veri Artırma (Data Augmentation)
Daha az görüntüyle daha iyi sonuç için:
- Döndürme (rotation)
- Ölçekleme (scaling)
- Parlaklık/kontrast değişimi
- Gürültü ekleme
- Yatay/dikey çevirme

## Aşama 2: Etiketleme (Annotation)

### 2.1 Etiketleme Araçları
Önerilen araçlar:
- **Roboflow**: https://roboflow.com (web tabanlı, kolay)
- **LabelImg**: https://github.com/heartexlabs/labelImg (masaüstü)
- **CVAT**: https://www.cvat.ai (gelişmiş)

### 2.2 YOLO Format
Her görüntü için bir `.txt` dosyası:
```
<class_id> <x_center> <y_center> <width> <height>
```

Değerler normalize edilmiş (0-1 arası):
```
0 0.5 0.5 0.3 0.2
```

### 2.3 Sınıf Tanımları
`classes.txt` dosyası oluşturun:
```
drone
quadcopter
fixed_wing_uav
helicopter
```

## Aşama 3: Dataset Yapısı

```
dataset/
├── images/
│   ├── train/
│   │   ├── img1.jpg
│   │   ├── img2.jpg
│   │   └── ...
│   ├── val/
│   │   ├── img100.jpg
│   │   └── ...
│   └── test/
│       ├── img200.jpg
│       └── ...
├── labels/
│   ├── train/
│   │   ├── img1.txt
│   │   ├── img2.txt
│   │   └── ...
│   ├── val/
│   │   ├── img100.txt
│   │   └── ...
│   └── test/
│       ├── img200.txt
│       └── ...
└── data.yaml
```

### data.yaml
```yaml
# Dataset yolu
path: ./dataset

# Alt dizinler
train: images/train
val: images/val
test: images/test

# Sınıf sayısı
nc: 4

# Sınıf isimleri
names: ['drone', 'quadcopter', 'fixed_wing_uav', 'helicopter']
```

## Aşama 4: Model Eğitimi

### 4.1 Eğitim Scripti

`train_custom_model.py` dosyası oluşturun:

```python
from ultralytics import YOLO

# Model seç (Raspberry Pi için nano veya small)
model = YOLO('yolov8n.pt')  # veya yolov8s.pt

# Eğitim
results = model.train(
    data='dataset/data.yaml',
    epochs=100,              # Epoch sayısı
    imgsz=640,               # Görüntü boyutu
    batch=16,                # Batch size (GPU'ya göre ayarla)
    device='cpu',            # 'cpu' veya 'cuda'
    workers=4,               # Paralel işlem sayısı
    patience=20,             # Early stopping
    save=True,               # Model kaydet
    project='runs/train',    # Çıktı dizini
    name='uav_detector',     # Deney adı
    
    # Augmentation
    augment=True,
    hsv_h=0.015,            # Hue
    hsv_s=0.7,              # Saturation
    hsv_v=0.4,              # Value
    degrees=10,             # Rotation
    translate=0.1,          # Translation
    scale=0.5,              # Scale
    flipud=0.0,             # Vertical flip
    fliplr=0.5,             # Horizontal flip
    mosaic=1.0,             # Mosaic augmentation
)

# Sonuçları yazdır
print(results)
```

### 4.2 Eğitimi Başlat

```bash
# Sanal ortamı aktive et
venv\Scripts\activate

# Eğitimi başlat
python train_custom_model.py
```

### 4.3 Eğitim Takibi

Eğitim sırasında `runs/train/uav_detector/` dizininde:
- `weights/best.pt`: En iyi model
- `weights/last.pt`: Son model
- `results.png`: Eğitim grafikleri
- `confusion_matrix.png`: Karışıklık matrisi

## Aşama 5: Model Değerlendirme

### 5.1 Validasyon

```python
from ultralytics import YOLO

# Eğitilmiş modeli yükle
model = YOLO('runs/train/uav_detector/weights/best.pt')

# Validasyon
metrics = model.val()

print(f"mAP50: {metrics.box.map50}")
print(f"mAP50-95: {metrics.box.map}")
```

### 5.2 Test

```python
# Test görüntülerinde tahmin
results = model.predict(
    source='dataset/images/test',
    save=True,
    conf=0.5
)
```

## Aşama 6: Raspberry Pi için Optimizasyon

### 6.1 Model Export

```python
from ultralytics import YOLO

model = YOLO('runs/train/uav_detector/weights/best.pt')

# TensorFlow Lite export (Raspberry Pi için ideal)
model.export(format='tflite', int8=True)

# ONNX export (alternatif)
model.export(format='onnx')
```

### 6.2 Quantization (Boyut küçültme)

```python
# INT8 quantization
model.export(
    format='tflite',
    int8=True,
    data='dataset/data.yaml'  # Kalibasyon için
)
```

## Aşama 7: Sisteme Entegrasyon

### 7.1 Özel Modeli Kullan

```bash
# Ana programda özel model kullan
python src/main.py --source screen --model runs/train/uav_detector/weights/best.pt
```

### 7.2 Config Güncelleme

`src/config.py` dosyasında:

```python
MODEL_CONFIG = {
    'default_model': 'runs/train/uav_detector/weights/best.pt',
    'target_classes': {
        0: 'drone',
        1: 'quadcopter',
        2: 'fixed_wing_uav',
        3: 'helicopter'
    }
}
```

## Performans İyileştirme İpuçları

### Dataset
- ✅ En az 1000 görüntü
- ✅ Dengeli sınıf dağılımı
- ✅ Çeşitli koşullar (gece/gündüz, hava durumu)
- ✅ Farklı mesafeler (yakın/uzak)

### Eğitim
- ✅ Transfer learning kullan (pretrained model)
- ✅ Data augmentation uygula
- ✅ Early stopping ile overfitting önle
- ✅ Learning rate scheduling

### Model
- ✅ Raspberry Pi için YOLOv8n veya YOLOv8s
- ✅ Quantization uygula
- ✅ Input size'ı optimize et (640x640 veya 416x416)

## Faydalı Kaynaklar

- **YOLOv8 Dokümantasyonu**: https://docs.ultralytics.com
- **Roboflow Universe**: Hazır İHA datasets
- **Kaggle**: Açık kaynak datasets
- **Papers with Code**: En son araştırmalar

## Örnek Dataset Kaynakları

1. **Anti-UAV Dataset**: https://anti-uav.github.io
2. **Drone Detection Dataset (Kaggle)**: Arama yapın
3. **Roboflow Universe**: "drone detection" ara
4. **YouTube**: Video'lardan frame çıkarma

## Sorun Giderme

### Düşük mAP
- Daha fazla veri topla
- Etiketleme kalitesini kontrol et
- Augmentation artır
- Daha uzun eğit

### Overfitting
- Dropout ekle
- Data augmentation artır
- Early stopping kullan
- Daha fazla validation data

### Yavaş Inference
- Daha küçük model (yolov8n)
- Quantization uygula
- Input size küçült
- TFLite kullan

---

**Not**: Özel dataset eğitimi için GPU kullanımı şiddetle önerilir. Google Colab ücretsiz GPU sağlar.
