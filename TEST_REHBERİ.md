# 🎯 Test Rehberi - Farklı Konfigürasyonları Deneyin

## ✅ Tracker Sorunu Çözüldü!

**Yapılan Düzeltme**:
- Bbox validasyonu ve düzeltme eklendi
- Negatif genişlik/yükseklik kontrolü
- Frame sınırları kontrolü
- Detaylı hata mesajları

**Artık tracker başarıyla başlıyor!** ✅

---

## 🚀 Hızlı Test Komutları

### Yöntem 1: Batch Script (Önerilen)

```bash
test_hizli.bat
```

**Menü Seçenekleri**:
1. CSRT + Kalman (En stabil)
2. CSRT (Kalman olmadan)
3. KCF + Kalman (Dengeli)
4. MOSSE + Kalman (En hızlı)
5. MIL + Kalman
6. Tüm tracker'ları karşılaştır
7. Düşük güven (0.3) - Daha fazla tespit
8. Yüksek güven (0.7) - Daha az tespit

### Yöntem 2: Manuel Komutlar

```bash
# CSRT + Kalman (En stabil)
python src\main.py --source screen --tracker CSRT --use-kalman --save

# KCF + Kalman (Dengeli)
python src\main.py --source screen --tracker KCF --use-kalman --save

# MOSSE + Kalman (En hızlı)
python src\main.py --source screen --tracker MOSSE --use-kalman --save

# MIL + Kalman
python src\main.py --source screen --tracker MIL --use-kalman --save

# Kalman olmadan
python src\main.py --source screen --tracker CSRT --save

# Düşük güven eşiği (daha fazla tespit)
python src\main.py --source screen --conf 0.3 --use-kalman --save

# Yüksek güven eşiği (daha az tespit)
python src\main.py --source screen --conf 0.7 --use-kalman --save
```

---

## 📊 Test Sonuçlarını Analiz Etme

### Tüm Testleri Karşılaştır

```bash
python analiz_testler.py
```

**Çıktı Örneği**:
```
================================================================================
TEST SONUÇLARI KARŞILAŞTIRMASI
================================================================================

Test       Tracker  Kalman   Frames   Track%   FPS      Conf     Süre    
--------------------------------------------------------------------------------
test-1     CSRT     ✓        500      85.2%    18.5     0.82     27.0s
test-2     CSRT     ✗        450      72.1%    22.3     0.00     20.2s
test-3     KCF      ✓        520      78.5%    25.1     0.75     20.7s
test-4     MOSSE    ✓        600      65.3%    32.4     0.68     18.5s

📊 EN İYİ PERFORMANSLAR
--------------------------------------------------------------------------------
🏆 En Yüksek Tracking Oranı: test-1 - %85.2
   Tracker: CSRT, Kalman: ✓

⚡ En Yüksek FPS: test-4 - 32.4 FPS
   Tracker: MOSSE, Kalman: ✓

🎯 En Yüksek Güven Skoru: test-1 - 0.82
   Tracker: CSRT, Kalman: ✓
```

### Belirli Bir Testi İncele

```bash
python analiz_testler.py test-1
```

---

## 🎯 Tracker Karşılaştırması

| Tracker | Hız | Doğruluk | Stabilite | Önerilen Kullanım |
|---------|-----|----------|-----------|-------------------|
| **CSRT** | Yavaş | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | Yüksek doğruluk gerekli |
| **KCF** | Orta | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | Dengeli performans |
| **MOSSE** | Hızlı | ⭐⭐⭐ | ⭐⭐⭐ | Raspberry Pi, gerçek zamanlı |
| **MIL** | Orta | ⭐⭐⭐ | ⭐⭐⭐ | Genel amaçlı |

---

## 💡 Test İpuçları

### 1. İyi Bir Test İçin

**Hazırlık**:
- YouTube'da uzun bir uçak videosu açın (5+ dakika)
- Videoyu tam ekran yapın
- Arka planda başka program açmayın

**Test Süresi**:
- Minimum 30 saniye
- İdeal: 2-3 dakika
- Karşılaştırma için: Aynı videoyu kullanın

### 2. Tracker Seçimi

**CSRT Kullanın**:
- Yüksek doğruluk gerekli
- FPS önemli değil
- Bilgisayarda test

**KCF Kullanın**:
- Dengeli performans
- Orta doğruluk yeterli
- Genel kullanım

**MOSSE Kullanın**:
- Hız kritik
- Raspberry Pi deployment
- Gerçek zamanlı uygulama

### 3. Kalman Filter

**Kullanın**:
- Düzgün hareket eden hedefler
- Tahmin önemli
- Stabilite gerekli

**Kullanmayın**:
- Ani hareketler
- Maksimum hız gerekli
- Basit tracking yeterli

### 4. Güven Eşiği

**Düşük (0.3)**:
- Daha fazla tespit
- Daha fazla false positive
- Zor koşullar

**Orta (0.5)** - Varsayılan:
- Dengeli
- Çoğu durum için ideal

**Yüksek (0.7)**:
- Daha az tespit
- Daha az false positive
- Yüksek kalite gerekli

---

## 📈 Beklenen Performans

### Bilgisayarınızda (1920x1080)

| Konfigürasyon | FPS | Tracking Oranı |
|---------------|-----|----------------|
| CSRT + Kalman | 15-20 | %80-90 |
| KCF + Kalman | 20-25 | %70-80 |
| MOSSE + Kalman | 25-35 | %60-70 |

### Raspberry Pi 5'te (640x480)

| Konfigürasyon | FPS | Tracking Oranı |
|---------------|-----|----------------|
| CSRT + Kalman | 8-12 | %75-85 |
| KCF + Kalman | 12-18 | %65-75 |
| MOSSE + Kalman | 18-25 | %55-65 |

---

## 🔍 Sorun Giderme

### Tracking Oranı Düşük (<50%)

**Çözüm 1**: Daha güçlü tracker
```bash
python src\main.py --source screen --tracker CSRT --use-kalman --save
```

**Çözüm 2**: Düşük güven eşiği
```bash
python src\main.py --source screen --conf 0.3 --use-kalman --save
```

**Çözüm 3**: Robust tracker parametreleri
`src/robust_tracker.py`:
```python
max_bbox_change=0.5,  # 0.3 -> 0.5
max_consecutive_failures=5  # 3 -> 5
```

### FPS Düşük (<15)

**Çözüm 1**: Hızlı tracker
```bash
python src\main.py --source screen --tracker MOSSE --use-kalman --save
```

**Çözüm 2**: Resize aktif
`src/config.py`:
```python
PERFORMANCE_CONFIG = {
    'resize_input': True,  # False -> True
}
```

**Çözüm 3**: Kalman olmadan
```bash
python src\main.py --source screen --tracker KCF --save
```

### Çok Fazla False Positive

**Çözüm**: Yüksek güven eşiği
```bash
python src\main.py --source screen --conf 0.7 --use-kalman --save
```

---

## 📁 Test Sonuçları

Her test sonrası:

```
testler/
├── test-1/
│   ├── test_results.json    # Detaylı veri
│   ├── ÖZET.txt             # Okunabilir rapor
│   └── tracking_*.mp4       # Video
├── test-2/
└── test-3/
```

**Özet raporu görüntüle**:
```bash
type testler\test-1\ÖZET.txt
```

**Video'yu izle**:
```bash
testler\test-1\tracking_*.mp4
```

---

## 🎯 Önerilen Test Sırası

1. **Baseline Test** (CSRT + Kalman)
   ```bash
   python src\main.py --source screen --tracker CSRT --use-kalman --save
   ```

2. **Hız Testi** (MOSSE + Kalman)
   ```bash
   python src\main.py --source screen --tracker MOSSE --use-kalman --save
   ```

3. **Kalman Etkisi** (CSRT, Kalman olmadan)
   ```bash
   python src\main.py --source screen --tracker CSRT --save
   ```

4. **Karşılaştırma**
   ```bash
   python analiz_testler.py
   ```

---

**Farklı konfigürasyonları deneyin ve en iyisini bulun! 🚀**
