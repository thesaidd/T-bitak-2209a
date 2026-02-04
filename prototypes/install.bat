@echo off
REM İHA Tespit ve Takip Sistemi - Kurulum Scripti (Windows)

echo ========================================
echo IHA TESPIT VE TAKIP SISTEMI - KURULUM
echo ========================================
echo.

REM Python kontrolü
python --version >nul 2>&1
if errorlevel 1 (
    echo HATA: Python bulunamadi!
    echo Lutfen Python 3.8+ yukleyin: https://www.python.org/downloads/
    pause
    exit /b 1
)

echo [1/5] Python versiyonu kontrol ediliyor...
python --version

echo.
echo [2/5] Sanal ortam olusturuluyor...
if exist venv (
    echo Sanal ortam zaten mevcut, atlanıyor...
) else (
    python -m venv venv
    echo Sanal ortam olusturuldu.
)

echo.
echo [3/5] Sanal ortam aktive ediliyor...
call venv\Scripts\activate.bat

echo.
echo [4/5] Pip guncelleniyor...
python -m pip install --upgrade pip

echo.
echo [5/5] Bagimliliklar yukleniyor...
echo Bu islem birkaç dakika surebilir...
pip install -r requirements.txt

echo.
echo ========================================
echo KURULUM TAMAMLANDI!
echo ========================================
echo.
echo Sistemi calistirmak icin:
echo   1. Sanal ortami aktive edin: venv\Scripts\activate
echo   2. Ana programi calistirin: python src\main.py --source screen
echo.
echo Ornek komutlar:
echo   - Ekran testi:  python src\main.py --source screen --use-kalman --save
echo   - Video testi:  python src\main.py --source video.mp4 --use-kalman
echo   - Webcam testi: python src\main.py --source 0 --use-kalman
echo.

pause
