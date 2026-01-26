# IHA Tespit ve Takip Sistemi - Dokumantasyon Indeksi

## Hosgeldiniz! 🚀

Bu proje, gercek zamanli IHA (Insansiz Hava Araci) tespit ve takip sistemidir. YOLOv8, OpenCV ve Kalman Filter kullanarak yuksek performansli tracking saglar.

---

## Hizli Baslangic

### 1. Ilk Kurulum

```bash
# Kurulum
install.bat

# Test
python test_system.py

# Hizli baslangic
run_mosse_fast.bat
```

### 2. Ilk Test

1. YouTube'da bir ucak videosu acin
2. Tam ekran yapin
3. `run_mosse_fast.bat` calistirin
4. Izleyin! 🎉

---

## Dokumantasyon Rehberi

### Baslangiç Seviye

#### 1. [BASLARKEN.md](BASLARKEN.md)
**Ne icin**: Ilk kez kullanacaklar icin
**Icindekiler**:
- Hizli kurulum
- Ilk test
- Temel kontroller
- Ornek komutlar

**Okuma suresi**: 5 dakika

---

#### 2. [QUICKSTART.md](QUICKSTART.md)
**Ne icin**: Hizli baslangic (Ingilizce)
**Icindekiler**:
- Installation
- First test
- Basic usage
- Examples

**Okuma suresi**: 5 dakika

---

### Kullanim Rehberleri

#### 3. [TEST_REHBERI.md](TEST_REHBERI.md)
**Ne icin**: Farkli konfigurasyonlari test etmek
**Icindekiler**:
- Tracker karsilastirmasi
- Kalman filter etkisi
- Performans testleri
- Sonuc analizi

**Okuma suresi**: 15 dakika

---

#### 4. [MOSSE_OPTIMIZE_REHBER.md](MOSSE_OPTIMIZE_REHBER.md)
**Ne icin**: Maksimum hiz icin MOSSE modu
**Icindekiler**:
- MOSSE optimizasyonlari
- Performans ipuclari
- Raspberry Pi deployment
- Sorun giderme

**Okuma suresi**: 10 dakika

---

### Teknik Dokumantasyon

#### 5. [SISTEM_DOKUMANTASYONU.md](SISTEM_DOKUMANTASYONU.md) ⭐ **EN KAPSAMLI**
**Ne icin**: Tum sistemi anlamak
**Icindekiler**:
- Mimari ve moduller
- Veri akisi
- Adim adim calisma prensibi
- Optimizasyonlar
- Kullanim senaryolari

**Okuma suresi**: 30-45 dakika

---

#### 6. [TECHNICAL.md](TECHNICAL.md)
**Ne icin**: Teknik detaylar (Ingilizce)
**Icindekiler**:
- Architecture
- Algorithms
- Performance analysis
- Advanced usage

**Okuma suresi**: 20 dakika

---

### Sorun Giderme

#### 7. [SORUN_COZUMU.md](SORUN_COZUMU.md)
**Ne icin**: Tracker ve pencere boyutu sorunlari
**Icindekiler**:
- Tracker baslatma hatasi cozumu
- Pencere boyutu duzeltmesi
- Bbox validasyonu

**Okuma suresi**: 5 dakika

---

#### 8. [GORUNTU_BULANIKLIK_COZUM.md](GORUNTU_BULANIKLIK_COZUM.md)
**Ne icin**: Goruntu kalitesi sorunu
**Icindekiler**:
- Bulaniklik nedeni
- Iki frame sistemi
- Performans etkisi

**Okuma suresi**: 5 dakika

---

### Iyilestirmeler ve Degisiklikler

#### 9. [IYILESTIRMELER.md](IYILESTIRMELER.md)
**Ne icin**: Yapilan iyilestirmeleri gormek
**Icindekiler**:
- Test kayit sistemi
- Tracking stabilite iyilestirmeleri
- Config optimizasyonlari

**Okuma suresi**: 10 dakika

---

### Proje Bilgileri

#### 10. [README.md](README.md)
**Ne icin**: Proje genel bakis
**Icindekiler**:
- Proje tanitimi
- Ozellikler
- Kurulum
- Kullanim

**Okuma suresi**: 10 dakika

---

#### 11. [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)
**Ne icin**: Proje ozeti
**Icindekiler**:
- Genel bakis
- Dosya yapisi
- Moduller

**Okuma suresi**: 5 dakika

---

### Ozel Konular

#### 12. [CUSTOM_TRAINING.md](CUSTOM_TRAINING.md)
**Ne icin**: Ozel model egitimi
**Icindekiler**:
- Dataset hazirlama
- Model egitimi
- Fine-tuning

**Okuma suresi**: 20 dakika

---

## Okuma Sirasi Onerileri

### Yeni Baslayanlar Icin

```
1. BASLARKEN.md (5 dk)
   ↓
2. Ilk test yap (run_mosse_fast.bat)
   ↓
3. TEST_REHBERI.md (15 dk)
   ↓
4. Farkli tracker'lari dene
   ↓
5. SISTEM_DOKUMANTASYONU.md (30 dk)
```

### Deneyimli Kullanicilar Icin

```
1. README.md (10 dk)
   ↓
2. SISTEM_DOKUMANTASYONU.md (30 dk)
   ↓
3. TECHNICAL.md (20 dk)
   ↓
4. MOSSE_OPTIMIZE_REHBER.md (10 dk)
   ↓
5. CUSTOM_TRAINING.md (20 dk)
```

### Sorun Yasayanlar Icin

```
1. SORUN_COZUMU.md
   ↓
2. GORUNTU_BULANIKLIK_COZUM.md
   ↓
3. TEST_REHBERI.md → Sorun Giderme bolumu
   ↓
4. MOSSE_OPTIMIZE_REHBER.md → Sorun Giderme bolumu
```

---

## Hizli Erisim

### Komutlar

```bash
# Kurulum
install.bat

# Hizli test
run_mosse_fast.bat

# Test menusu
test_hizli.bat

# Sistem testi
python test_system.py

# Sonuc analizi
python analiz_testler.py
```

### Dosya Yapisi

```
Tübitak/
├── src/                           # Kaynak kodlar
│   ├── main.py                    # Ana program
│   ├── main_mosse_fast.py         # MOSSE optimize
│   ├── config.py                  # Konfigurasyon
│   ├── detector.py                # YOLOv8 tespit
│   ├── tracker.py                 # Temel tracking
│   ├── robust_tracker.py          # Gelismis tracking
│   ├── kalman_filter.py           # Kalman filter
│   ├── screen_capture.py          # Goruntu yakalama
│   └── test_logger.py             # Test kayit
│
├── testler/                       # Test sonuclari
│   ├── test-1/
│   ├── test-2/
│   └── ...
│
├── results/                       # Video kayitlari
│
├── Dokumantasyon/                 # Bu dosyalar
│   ├── BASLARKEN.md
│   ├── SISTEM_DOKUMANTASYONU.md
│   ├── TEST_REHBERI.md
│   └── ...
│
├── install.bat                    # Kurulum
├── run_mosse_fast.bat             # Hizli baslangic
├── test_hizli.bat                 # Test menusu
└── analiz_testler.py              # Sonuc analizi
```

---

## Performans Profilleri

| Profil | FPS | Dogruluk | Dokuman |
|--------|-----|----------|---------|
| **MOSSE Fast** | 25-35 | %65-75 | MOSSE_OPTIMIZE_REHBER.md |
| **KCF Balanced** | 20-25 | %70-80 | TEST_REHBERI.md |
| **CSRT Quality** | 15-20 | %80-90 | TEST_REHBERI.md |

---

## Sik Sorulan Sorular

### S: Hangi tracker'i kullanmaliyim?

**C**: 
- **Hiz onemliyse**: MOSSE (`run_mosse_fast.bat`)
- **Denge istiyorsaniz**: KCF
- **Dogruluk onemliyse**: CSRT

Detay: `TEST_REHBERI.md`

---

### S: Goruntu neden bulanik?

**C**: Resize sorunu. MOSSE optimize mod kullanin (zaten duzeltildi).

Detay: `GORUNTU_BULANIKLIK_COZUM.md`

---

### S: Tracking cok kayip veriyor?

**C**: 
1. Daha guclu tracker kullanin (CSRT)
2. Dusuk guven esigi deneyin (--conf 0.3)
3. Robust tracker parametrelerini ayarlayin

Detay: `SORUN_COZUMU.md`, `TEST_REHBERI.md`

---

### S: FPS cok dusuk?

**C**:
1. MOSSE tracker kullanin
2. Daha kucuk input boyutu (320x320)
3. Gorsellestirme azaltin

Detay: `MOSSE_OPTIMIZE_REHBER.md`

---

### S: Raspberry Pi'de nasil calistirilir?

**C**: 
```bash
# Transfer
scp -r src/ pi@raspberrypi:/home/pi/uav_tracking/

# Calistir
ssh pi@raspberrypi
cd /home/pi/uav_tracking
python src/main_mosse_fast.py --source 0 --save
```

Detay: `MOSSE_OPTIMIZE_REHBER.md` → Raspberry Pi Deployment

---

### S: Ozel model nasil egitilir?

**C**: `CUSTOM_TRAINING.md` dosyasini okuyun.

---

## Destek ve Iletisim

### Sorun Bildirme

1. `SORUN_COZUMU.md` kontrol edin
2. `TEST_REHBERI.md` → Sorun Giderme bolumune bakin
3. GitHub'da issue acin (varsa)

### Katkida Bulunma

1. Fork yapin
2. Feature branch olusturun
3. Commit yapin
4. Pull request gonderin

---

## Lisans

MIT License - Detaylar icin LICENSE dosyasina bakin.

---

## Tesekkurler

- **YOLOv8**: Ultralytics
- **OpenCV**: OpenCV Team
- **Kalman Filter**: FilterPy

---

**Basarilar! 🚀**

Herhangi bir sorunuz varsa dokumantasyonu inceleyin veya destek alin.
