@echo off
REM MOSSE Tracker - Optimize Edilmis Hizli Mod
REM En yuksek performans icin ayarlanmis

echo ============================================================
echo IHA TESPIT VE TAKIP SISTEMI - MOSSE OPTIMIZE MOD
echo ============================================================
echo.
echo Bu mod en yuksek hiz icin optimize edilmistir:
echo   - MOSSE Tracker (En hizli)
echo   - Kalman Filter (Optimize edilmis)
echo   - Resize aktif (416x416)
echo   - Dusuk latency
echo.
echo Beklenen Performans:
echo   - FPS: 30-40
echo   - Tracking Orani: %%65-75
echo   - Latency: Cok dusuk
echo.
echo ============================================================
echo.

REM Sanal ortami aktive et
call venv\Scripts\activate.bat

REM MOSSE optimize mod ile calistir (ozel script)
python src\main_mosse_fast.py --source screen --save

echo.
echo ============================================================
echo Test tamamlandi!
echo.
echo Test sonuclari: testler\ klasoru
echo Video kaydi: results\ klasoru
echo.
echo Analiz icin: python analiz_testler.py
echo ============================================================
echo.

pause
