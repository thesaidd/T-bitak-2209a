@echo off
REM Hızlı başlatma scripti

echo IHA Tespit ve Takip Sistemi baslatiliyor...
echo.

REM Sanal ortamı aktive et
call venv\Scripts\activate.bat

REM Ana programı çalıştır (ekran kaynağı, Kalman filter, kayıt)
python src\main.py --source screen --use-kalman --save

pause
