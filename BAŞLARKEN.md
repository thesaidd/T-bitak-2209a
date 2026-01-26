# ✅ Kurulum Tamamlandı!

## 🎉 Sistem Başarıyla Kuruldu

Tüm testler başarıyla geçti! Sisteminiz kullanıma hazır.

## 🚀 Hemen Başlayın

### 1. Test İçin YouTube Videosu Açın
Tarayıcınızda şu aramalardan birini yapın:
- "airplane landing"
- "drone flying"
- "uav flight"

Videoyu **tam ekran** yapın.

### 2. Sistemi Başlatın

**Yöntem 1: Hızlı Başlatma (Önerilen)**
```bash
run.bat
```

**Yöntem 2: Manuel**
```bash
# Sanal ortamı aktive edin
venv\Scripts\activate

# Sistemi başlatın
python src\main.py --source screen --use-kalman --save
```

### 3. Kontroller

Sistem çalışırken kullanabileceğiniz tuşlar:

| Tuş | Fonksiyon |
|-----|-----------|
| **SPACE** | Duraklat/Devam |
| **r** | Tracker'ı sıfırla |
| **d** | Tespit moduna geç |
| **t** | Takip moduna geç |
| **s** | Manuel hedef seçimi |
| **ESC** | Çıkış |

## 📊 Ne Göreceksiniz?

- **Yeşil Kutu**: Tespit edilen İHA/uçak
- **Mavi Çizgi**: Hareket izi (trajectory)
- **Kırmızı Noktalar**: Kalman filter tahmini
- **FPS**: Saniyedeki frame sayısı
- **Mod**: DETECTION veya TRACKING

## 🎯 İlk Test Önerileri

### Test 1: Basit Tespit
```bash
python src\main.py --source screen
```
Sadece tespit ve takip, kayıt yok.

### Test 2: Kalman Filter ile
```bash
python src\main.py --source screen --use-kalman
```
Gelişmiş hareket tahmini ile.

### Test 3: Tam Özellikli
```bash
python src\main.py --source screen --use-kalman --save
```
Tüm özellikler + video kayıt.

## 📁 Kayıtlar

Video kayıtları `results/` klasöründe:
```
results/tracking_YYYYMMDD_HHMMSS.mp4
```

## 🔧 Sorun mu Yaşıyorsunuz?

### Tespit Yapılmıyor
```bash
# Güven eşiğini düşürün
python src\main.py --source screen --conf 0.3

# Tüm sınıfları aktive edin
python src\main.py --source screen --all-classes
```

### Düşük FPS
`src\config.py` dosyasında:
```python
PERFORMANCE_CONFIG = {
    'input_size': (416, 416)  # Daha küçük boyut
}
```

### Tracker Kaybediyor
```bash
# Daha güçlü tracker kullanın
python src\main.py --source screen --tracker CSRT
```

## 📚 Daha Fazla Bilgi

- **Hızlı Başlangıç**: `QUICKSTART.md`
- **Teknik Detaylar**: `TECHNICAL.md`
- **Özel Model Eğitimi**: `CUSTOM_TRAINING.md`
- **Ana Dokümantasyon**: `README.md`

## 🎓 Sonraki Adımlar

1. ✅ Ekran testi yaptınız
2. ⏭️ Video dosyası ile test edin
3. ⏭️ Webcam ile deneyin
4. ⏭️ Farklı tracker'ları karşılaştırın
5. ⏭️ Özel dataset toplayın ve model eğitin

---

**Başarılar! İyi testler! 🚀**

Sorularınız için dokümantasyona bakın veya GitHub'da issue açın.
