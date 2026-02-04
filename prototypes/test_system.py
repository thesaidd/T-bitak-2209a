"""
Test Scripti - Modülleri ayrı ayrı test et
"""

import sys
import os
import cv2
import numpy as np

# src klasörünü Python path'e ekle
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))


def test_imports():
    """Kütüphane importlarını test et"""
    print("="*60)
    print("KÜTÜPHANE TESTLERI")
    print("="*60)
    
    tests = {
        'OpenCV': lambda: __import__('cv2'),
        'NumPy': lambda: __import__('numpy'),
        'Ultralytics (YOLOv8)': lambda: __import__('ultralytics'),
        'MSS (Screen Capture)': lambda: __import__('mss'),
        'FilterPy (Kalman)': lambda: __import__('filterpy'),
        'Matplotlib': lambda: __import__('matplotlib'),
        'PyTorch': lambda: __import__('torch'),
    }
    
    results = {}
    for name, test_func in tests.items():
        try:
            module = test_func()
            version = getattr(module, '__version__', 'N/A')
            results[name] = ('✓', version)
            print(f"✓ {name:30s} v{version}")
        except Exception as e:
            results[name] = ('✗', str(e))
            print(f"✗ {name:30s} HATA: {e}")
    
    print()
    return all(r[0] == '✓' for r in results.values())


def test_screen_capture():
    """Ekran yakalama testi"""
    print("="*60)
    print("EKRAN YAKALAMA TESTI")
    print("="*60)
    
    try:
        from screen_capture import ScreenCapture
        
        sc = ScreenCapture()
        print("✓ ScreenCapture oluşturuldu")
        
        # Monitörleri listele
        sc.list_monitors()
        
        # FPS testi
        print("\nFPS testi (5 frame)...")
        import time
        start = time.time()
        for _ in range(5):
            frame = sc.capture()
        elapsed = time.time() - start
        fps = 5 / elapsed
        
        print(f"✓ Ekran yakalama FPS: {fps:.1f}")
        print(f"✓ Frame boyutu: {frame.shape}")
        
        sc.close()
        return True
        
    except Exception as e:
        print(f"✗ HATA: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_detector():
    """YOLOv8 detector testi"""
    print("\n" + "="*60)
    print("YOLOV8 DETECTOR TESTI")
    print("="*60)
    
    try:
        from detector import UAVDetector
        
        # Detector oluştur
        detector = UAVDetector(
            model_path='yolov8n.pt',
            conf_threshold=0.5,
            target_classes=[4, 14]  # airplane, bird
        )
        print("✓ UAVDetector oluşturuldu")
        
        # Test görüntüsü
        test_img = np.random.randint(0, 255, (640, 480, 3), dtype=np.uint8)
        
        # Tespit
        detections = detector.detect(test_img)
        print(f"✓ Tespit çalıştı (tespit sayısı: {len(detections)})")
        
        # İstatistikler
        stats = detector.get_stats()
        print(f"✓ Ortalama inference süresi: {stats['avg_inference_time']*1000:.1f} ms")
        
        return True
        
    except Exception as e:
        print(f"✗ HATA: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_tracker():
    """Tracker testi"""
    print("\n" + "="*60)
    print("TRACKER TESTI")
    print("="*60)
    
    try:
        from tracker import ObjectTracker
        
        # Tracker oluştur
        tracker = ObjectTracker('CSRT')
        print("✓ ObjectTracker oluşturuldu")
        
        # Test görüntüsü
        test_img = np.random.randint(0, 255, (640, 480, 3), dtype=np.uint8)
        
        # Tracker başlat
        bbox = (100, 100, 200, 200)
        success = tracker.init(test_img, bbox)
        print(f"✓ Tracker başlatıldı: {success}")
        
        # Güncelle
        success, new_bbox = tracker.update(test_img)
        print(f"✓ Tracker güncellendi: {success}")
        
        return True
        
    except Exception as e:
        print(f"✗ HATA: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_kalman():
    """Kalman filter testi"""
    print("\n" + "="*60)
    print("KALMAN FILTER TESTI")
    print("="*60)
    
    try:
        from kalman_filter import UAVKalmanFilter
        
        # Kalman filter oluştur
        kf = UAVKalmanFilter()
        print("✓ UAVKalmanFilter oluşturuldu")
        
        # Başlat
        kf.init(100, 100)
        print("✓ Kalman filter başlatıldı")
        
        # Güncelle
        filtered_pos = kf.update((110, 105))
        print(f"✓ Kalman filter güncellendi: {filtered_pos}")
        
        # Tahmin
        predictions = kf.predict_ahead(5)
        print(f"✓ İleri tahmin yapıldı: {len(predictions)} adım")
        
        return True
        
    except Exception as e:
        print(f"✗ HATA: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_integration():
    """Entegrasyon testi - tüm modüller birlikte"""
    print("\n" + "="*60)
    print("ENTEGRASYON TESTI")
    print("="*60)
    
    try:
        from screen_capture import ScreenCapture
        from detector import UAVDetector
        from tracker import ObjectTracker
        from kalman_filter import UAVKalmanFilter
        
        # Modüller
        sc = ScreenCapture()
        detector = UAVDetector('yolov8n.pt', target_classes=[4, 14])
        tracker = ObjectTracker('CSRT')
        kf = UAVKalmanFilter()
        
        print("✓ Tüm modüller oluşturuldu")
        
        # Test döngüsü
        for i in range(3):
            # Frame yakala
            frame = sc.capture()
            
            # Tespit
            detections = detector.detect(frame)
            
            print(f"✓ Frame {i+1}: {len(detections)} tespit")
        
        sc.close()
        print("✓ Entegrasyon testi başarılı")
        
        return True
        
    except Exception as e:
        print(f"✗ HATA: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Ana test fonksiyonu"""
    print("\n" + "="*60)
    print("İHA TESPİT VE TAKİP SİSTEMİ - TEST SÜİTİ")
    print("="*60 + "\n")
    
    tests = [
        ("Kütüphane İmportları", test_imports),
        ("Ekran Yakalama", test_screen_capture),
        ("YOLOv8 Detector", test_detector),
        ("Tracker", test_tracker),
        ("Kalman Filter", test_kalman),
        ("Entegrasyon", test_integration),
    ]
    
    results = {}
    
    for name, test_func in tests:
        try:
            result = test_func()
            results[name] = result
        except Exception as e:
            print(f"\n✗ {name} testi beklenmedik hata: {e}")
            results[name] = False
    
    # Özet
    print("\n" + "="*60)
    print("TEST ÖZETİ")
    print("="*60)
    
    for name, result in results.items():
        status = "✓ BAŞARILI" if result else "✗ BAŞARISIZ"
        print(f"{status:15s} - {name}")
    
    total = len(results)
    passed = sum(results.values())
    
    print(f"\nToplam: {passed}/{total} test başarılı")
    
    if passed == total:
        print("\n🎉 TÜM TESTLER BAŞARILI!")
        return 0
    else:
        print(f"\n⚠️  {total - passed} test başarısız oldu.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
