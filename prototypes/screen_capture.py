"""
Ekran Yakalama Modülü
Bilgisayar ekranını kamera görüntüsü olarak kullanmak için
"""

import mss
import numpy as np
import cv2
from typing import Optional, Dict, Tuple


class ScreenCapture:
    """Ekran görüntüsü yakalama sınıfı"""
    
    def __init__(self, monitor: int = 1, region: Optional[Dict] = None):
        """
        Args:
            monitor: Hangi monitör (1, 2, ...)
            region: Yakalanacak bölge {'top': y, 'left': x, 'width': w, 'height': h}
        """
        self.sct = mss.mss()
        self.monitor = monitor
        self.region = region
        
        # Monitör bilgilerini al
        if monitor == 0:
            # Tüm ekranlar
            self.monitor_info = self.sct.monitors[0]
        else:
            # Belirli bir monitör
            if monitor <= len(self.sct.monitors) - 1:
                self.monitor_info = self.sct.monitors[monitor]
            else:
                print(f"Uyarı: Monitör {monitor} bulunamadı, birincisi kullanılıyor")
                self.monitor_info = self.sct.monitors[1]
        
        # Bölge belirtilmişse kullan
        if region:
            self.capture_region = region
        else:
            self.capture_region = self.monitor_info
            
        print(f"Ekran yakalama başlatıldı: {self.capture_region}")
    
    def capture(self) -> np.ndarray:
        """
        Ekran görüntüsü yakala
        
        Returns:
            BGR formatında numpy array
        """
        # Ekran görüntüsü al
        screenshot = self.sct.grab(self.capture_region)
        
        # BGRA -> BGR dönüşümü
        frame = np.array(screenshot)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        
        return frame
    
    def get_fps(self) -> float:
        """
        Yakalama FPS'ini hesapla
        
        Returns:
            FPS değeri
        """
        import time
        
        start_time = time.time()
        frame_count = 0
        
        while frame_count < 30:
            self.capture()
            frame_count += 1
        
        elapsed = time.time() - start_time
        fps = frame_count / elapsed
        
        return fps
    
    def get_resolution(self) -> Tuple[int, int]:
        """
        Yakalama çözünürlüğünü al
        
        Returns:
            (width, height)
        """
        return (self.capture_region['width'], self.capture_region['height'])
    
    def set_region(self, x: int, y: int, width: int, height: int):
        """
        Yakalama bölgesini değiştir
        
        Args:
            x: Sol üst köşe x
            y: Sol üst köşe y
            width: Genişlik
            height: Yükseklik
        """
        self.capture_region = {
            'top': y,
            'left': x,
            'width': width,
            'height': height
        }
        print(f"Yakalama bölgesi güncellendi: {self.capture_region}")
    
    def list_monitors(self):
        """Tüm monitörleri listele"""
        print("\nMevcut Monitörler:")
        for i, monitor in enumerate(self.sct.monitors):
            if i == 0:
                print(f"  {i}: Tüm Ekranlar - {monitor}")
            else:
                print(f"  {i}: Monitör {i} - {monitor}")
    
    def close(self):
        """Kaynakları serbest bırak"""
        self.sct.close()


class VideoCapture:
    """Video dosyası veya webcam yakalama sınıfı"""
    
    def __init__(self, source):
        """
        Args:
            source: Video dosya yolu veya webcam index (0, 1, ...)
        """
        self.source = source
        self.cap = cv2.VideoCapture(source)
        
        if not self.cap.isOpened():
            raise ValueError(f"Video kaynağı açılamadı: {source}")
        
        # Video bilgileri
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        
        print(f"Video kaynağı: {source}")
        print(f"  Çözünürlük: {self.width}x{self.height}")
        print(f"  FPS: {self.fps}")
        if isinstance(source, str):
            print(f"  Frame sayısı: {self.frame_count}")
    
    def capture(self) -> Optional[np.ndarray]:
        """
        Frame yakala
        
        Returns:
            BGR formatında numpy array veya None
        """
        ret, frame = self.cap.read()
        
        if not ret:
            return None
        
        return frame
    
    def get_fps(self) -> float:
        """FPS değerini döndür"""
        return self.fps
    
    def get_resolution(self) -> Tuple[int, int]:
        """Çözünürlüğü döndür"""
        return (self.width, self.height)
    
    def is_opened(self) -> bool:
        """Video kaynağı açık mı?"""
        return self.cap.isOpened()
    
    def close(self):
        """Kaynakları serbest bırak"""
        self.cap.release()


def create_capture(source: str):
    """
    Kaynak tipine göre uygun capture nesnesi oluştur
    
    Args:
        source: 'screen', webcam index (0, 1), veya video dosya yolu
        
    Returns:
        ScreenCapture veya VideoCapture nesnesi
    """
    if source == 'screen':
        return ScreenCapture()
    elif source.isdigit():
        return VideoCapture(int(source))
    else:
        return VideoCapture(source)


if __name__ == "__main__":
    # Test
    print("=== Ekran Yakalama Testi ===\n")
    
    # Monitörleri listele
    sc = ScreenCapture()
    sc.list_monitors()
    
    # FPS testi
    print("\nFPS testi yapılıyor...")
    fps = sc.get_fps()
    print(f"Ekran yakalama FPS: {fps:.2f}")
    
    # Görüntü göster
    print("\nEkran görüntüsü gösteriliyor (ESC ile çıkış)...")
    while True:
        frame = sc.capture()
        
        # FPS bilgisi ekle
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Göster
        cv2.imshow('Screen Capture Test', frame)
        
        # ESC ile çık
        if cv2.waitKey(1) & 0xFF == 27:
            break
    
    sc.close()
    cv2.destroyAllWindows()
