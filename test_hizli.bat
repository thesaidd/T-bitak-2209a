@echo off
REM Hızlı Test Komutları - Farklı Konfigürasyonlar

echo ============================================================
echo İHA TESPİT VE TAKİP SİSTEMİ - HIZLI TESTLER
echo ============================================================
echo.

:menu
echo Hangi testi çalıştırmak istersiniz?
echo.
echo 1. Kalman Filter ile (CSRT tracker)
echo 2. Kalman Filter OLMADAN (CSRT tracker)
echo 3. KCF Tracker + Kalman
echo 4. MOSSE Tracker + Kalman (En hızlı)
echo 5. MIL Tracker + Kalman
echo 6. Tüm tracker'ları karşılaştır (sırayla)
echo 7. Düşük güven eşiği (0.3) - Daha fazla tespit
echo 8. Yüksek güven eşiği (0.7) - Daha az tespit
echo 9. Çıkış
echo.

set /p choice="Seçiminiz (1-9): "

if "%choice%"=="1" goto test1
if "%choice%"=="2" goto test2
if "%choice%"=="3" goto test3
if "%choice%"=="4" goto test4
if "%choice%"=="5" goto test5
if "%choice%"=="6" goto test6
if "%choice%"=="7" goto test7
if "%choice%"=="8" goto test8
if "%choice%"=="9" goto exit

echo Geçersiz seçim!
goto menu

:test1
echo.
echo [TEST 1] CSRT Tracker + Kalman Filter
echo ============================================================
python src\main.py --source screen --tracker CSRT --use-kalman --save
goto done

:test2
echo.
echo [TEST 2] CSRT Tracker (Kalman OLMADAN)
echo ============================================================
python src\main.py --source screen --tracker CSRT --save
goto done

:test3
echo.
echo [TEST 3] KCF Tracker + Kalman Filter
echo ============================================================
python src\main.py --source screen --tracker KCF --use-kalman --save
goto done

:test4
echo.
echo [TEST 4] MOSSE Tracker + Kalman Filter (EN HIZLI)
echo ============================================================
python src\main.py --source screen --tracker MOSSE --use-kalman --save
goto done

:test5
echo.
echo [TEST 5] MIL Tracker + Kalman Filter
echo ============================================================
python src\main.py --source screen --tracker MIL --use-kalman --save
goto done

:test6
echo.
echo [TEST 6] Tüm Tracker'ları Karşılaştır
echo ============================================================
echo.
echo [1/4] CSRT Tracker...
python src\main.py --source screen --tracker CSRT --use-kalman --save
echo.
echo [2/4] KCF Tracker...
python src\main.py --source screen --tracker KCF --use-kalman --save
echo.
echo [3/4] MOSSE Tracker...
python src\main.py --source screen --tracker MOSSE --use-kalman --save
echo.
echo [4/4] MIL Tracker...
python src\main.py --source screen --tracker MIL --use-kalman --save
echo.
echo Tüm testler tamamlandı!
echo Test sonuçlarını testler\ klasöründe bulabilirsiniz.
goto done

:test7
echo.
echo [TEST 7] Düşük Güven Eşiği (0.3) - Daha Fazla Tespit
echo ============================================================
python src\main.py --source screen --conf 0.3 --use-kalman --save
goto done

:test8
echo.
echo [TEST 8] Yüksek Güven Eşiği (0.7) - Daha Az Tespit
echo ============================================================
python src\main.py --source screen --conf 0.7 --use-kalman --save
goto done

:done
echo.
echo ============================================================
echo Test tamamlandı!
echo Test sonuçları: testler\ klasörü
echo Video kayıtları: results\ klasörü
echo ============================================================
echo.
set /p again="Başka bir test yapmak ister misiniz? (E/H): "
if /i "%again%"=="E" goto menu
if /i "%again%"=="e" goto menu

:exit
echo.
echo Çıkılıyor...
