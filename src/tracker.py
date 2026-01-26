"""
Nesne Takip Modülü
OpenCV trackers ile nesne takibi
"""

import cv2
import numpy as np
from typing import Optional, Tuple, List, Dict
from collections import deque


class ObjectTracker:
    """OpenCV tabanlı nesne takip sınıfı"""
    
    # Mevcut tracker tipleri
    TRACKER_TYPES = {
        'CSRT': cv2.TrackerCSRT_create,      # Yüksek doğruluk, yavaş
        'KCF': cv2.TrackerKCF_create,        # Dengeli
        'MOSSE': cv2.legacy.TrackerMOSSE_create,  # Hızlı, düşük doğruluk
        'MIL': cv2.TrackerMIL_create,        # Orta
    }
    
    def __init__(self, tracker_type: str = 'CSRT', trajectory_length: int = 30):
        """
        Args:
            tracker_type: Tracker tipi (CSRT, KCF, MOSSE, MIL)
            trajectory_length: İz uzunluğu (frame sayısı)
        """
        if tracker_type not in self.TRACKER_TYPES:
            raise ValueError(f"Geçersiz tracker tipi: {tracker_type}")
        
        self.tracker_type = tracker_type
        self.tracker = None
        self.is_tracking = False
        self.trajectory_length = trajectory_length
        
        # İz (trajectory) için deque
        self.trajectory = deque(maxlen=trajectory_length)
        
        # İstatistikler
        self.frames_tracked = 0
        self.frames_lost = 0
        self.last_bbox = None
        self.last_center = None
        
        print(f"Tracker oluşturuldu: {tracker_type}")
    
    def init(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]):
        """
        Tracker'ı başlat
        
        Args:
            frame: İlk frame
            bbox: Başlangıç bbox (x1, y1, x2, y2)
        """
        # Yeni tracker oluştur
        self.tracker = self.TRACKER_TYPES[self.tracker_type]()
        
        # Bbox formatını dönüştür: (x1,y1,x2,y2) -> (x,y,w,h)
        x1, y1, x2, y2 = bbox
        
        # Bbox'ı düzelt (x1 < x2 ve y1 < y2 olmalı)
        if x1 > x2:
            x1, x2 = x2, x1
        if y1 > y2:
            y1, y2 = y2, y1
        
        x, y, w, h = x1, y1, x2 - x1, y2 - y1
        
        # Validasyon
        if w <= 0 or h <= 0:
            print(f"⚠️  Geçersiz bbox boyutu: w={w}, h={h}, bbox={bbox}")
            return False
        
        if w < 5 or h < 5:
            print(f"⚠️  Bbox çok küçük: w={w}, h={h}")
            return False
        
        # Frame sınırlarını kontrol et
        frame_h, frame_w = frame.shape[:2]
        if x < 0 or y < 0 or x + w > frame_w or y + h > frame_h:
            print(f"⚠️  Bbox frame dışında: ({x},{y},{w},{h}), frame=({frame_w},{frame_h})")
            # Bbox'ı frame içine sığdır
            x = max(0, min(x, frame_w - 1))
            y = max(0, min(y, frame_h - 1))
            w = min(w, frame_w - x)
            h = min(h, frame_h - y)
            print(f"   Düzeltildi: ({x},{y},{w},{h})")
        
        # Tracker'ı başlat
        try:
            success = self.tracker.init(frame, (x, y, w, h))
        except Exception as e:
            print(f"⚠️  Tracker init hatası: {e}")
            return False
        
        if success:
            self.is_tracking = True
            self.last_bbox = (x, y, x + w, y + h)
            self.last_center = (x + w // 2, y + h // 2)
            self.trajectory.clear()
            self.trajectory.append(self.last_center)
            print(f"✅ Tracker başlatıldı: bbox=({x},{y},{w},{h})")
        else:
            print(f"❌ Tracker başlatılamadı: bbox=({x},{y},{w},{h})")
        
        return success
    
    def update(self, frame: np.ndarray) -> Tuple[bool, Optional[Tuple[int, int, int, int]]]:
        """
        Tracker'ı güncelle
        
        Args:
            frame: Yeni frame
            
        Returns:
            (success, bbox) - bbox formatı (x1, y1, x2, y2)
        """
        if not self.is_tracking or self.tracker is None:
            return False, None
        
        # Tracker güncelle
        success, bbox_xywh = self.tracker.update(frame)
        
        if success:
            # Bbox formatını dönüştür: (x,y,w,h) -> (x1,y1,x2,y2)
            x, y, w, h = [int(v) for v in bbox_xywh]
            bbox = (x, y, x + w, y + h)
            
            # Merkez noktası
            center = ((x + x + w) // 2, (y + y + h) // 2)
            
            # Güncelle
            self.last_bbox = bbox
            self.last_center = center
            self.trajectory.append(center)
            self.frames_tracked += 1
            
            return True, bbox
        else:
            self.frames_lost += 1
            self.is_tracking = False
            return False, None
    
    def reset(self):
        """Tracker'ı sıfırla"""
        self.tracker = None
        self.is_tracking = False
        self.trajectory.clear()
        self.last_bbox = None
        self.last_center = None
        print("Tracker sıfırlandı")
    
    def get_trajectory(self) -> List[Tuple[int, int]]:
        """İz noktalarını döndür"""
        return list(self.trajectory)
    
    def get_velocity(self) -> Optional[Tuple[float, float]]:
        """
        Hız vektörünü hesapla (piksel/frame)
        
        Returns:
            (vx, vy) veya None
        """
        if len(self.trajectory) < 2:
            return None
        
        # Son iki nokta
        p1 = self.trajectory[-2]
        p2 = self.trajectory[-1]
        
        vx = p2[0] - p1[0]
        vy = p2[1] - p1[1]
        
        return (vx, vy)
    
    def get_speed(self) -> Optional[float]:
        """
        Hız büyüklüğünü hesapla (piksel/frame)
        
        Returns:
            Hız veya None
        """
        velocity = self.get_velocity()
        if velocity is None:
            return None
        
        vx, vy = velocity
        speed = np.sqrt(vx**2 + vy**2)
        
        return speed
    
    def draw_trajectory(self, 
                       frame: np.ndarray,
                       color: Tuple[int, int, int] = (255, 0, 0),
                       thickness: int = 2) -> np.ndarray:
        """
        İzi çiz
        
        Args:
            frame: Görüntü
            color: Çizgi rengi (BGR)
            thickness: Çizgi kalınlığı
            
        Returns:
            Çizilmiş görüntü
        """
        frame_copy = frame.copy()
        
        if len(self.trajectory) < 2:
            return frame_copy
        
        # İz noktalarını çiz
        points = list(self.trajectory)
        for i in range(1, len(points)):
            # Renk gradyanı (eski -> yeni)
            alpha = i / len(points)
            current_color = tuple(int(c * alpha) for c in color)
            
            cv2.line(frame_copy, points[i-1], points[i], current_color, thickness)
        
        # Son noktayı vurgula
        if self.last_center:
            cv2.circle(frame_copy, self.last_center, 5, (0, 0, 255), -1)
        
        return frame_copy
    
    def draw_info(self,
                  frame: np.ndarray,
                  position: Tuple[int, int] = (10, 60)) -> np.ndarray:
        """
        Tracker bilgilerini çiz
        
        Args:
            frame: Görüntü
            position: Metin pozisyonu
            
        Returns:
            Çizilmiş görüntü
        """
        frame_copy = frame.copy()
        x, y = position
        
        # Durum
        status = "TRACKING" if self.is_tracking else "LOST"
        color = (0, 255, 0) if self.is_tracking else (0, 0, 255)
        cv2.putText(frame_copy, f"Status: {status}", (x, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Hız
        speed = self.get_speed()
        if speed is not None:
            cv2.putText(frame_copy, f"Speed: {speed:.1f} px/frame", (x, y + 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # İstatistikler
        cv2.putText(frame_copy, f"Tracked: {self.frames_tracked} | Lost: {self.frames_lost}",
                   (x, y + 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return frame_copy
    
    def get_stats(self) -> Dict:
        """İstatistikleri döndür"""
        return {
            'tracker_type': self.tracker_type,
            'is_tracking': self.is_tracking,
            'frames_tracked': self.frames_tracked,
            'frames_lost': self.frames_lost,
            'trajectory_length': len(self.trajectory),
            'last_bbox': self.last_bbox,
            'last_center': self.last_center,
            'velocity': self.get_velocity(),
            'speed': self.get_speed()
        }


class MultiObjectTracker:
    """Çoklu nesne takibi için wrapper sınıf"""
    
    def __init__(self, tracker_type: str = 'CSRT', max_objects: int = 5):
        """
        Args:
            tracker_type: Tracker tipi
            max_objects: Maksimum takip edilecek nesne sayısı
        """
        self.tracker_type = tracker_type
        self.max_objects = max_objects
        self.trackers = {}  # ID -> ObjectTracker
        self.next_id = 0
    
    def add_tracker(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> int:
        """
        Yeni tracker ekle
        
        Args:
            frame: Frame
            bbox: Bbox
            
        Returns:
            Tracker ID
        """
        if len(self.trackers) >= self.max_objects:
            print(f"Maksimum tracker sayısına ulaşıldı: {self.max_objects}")
            return -1
        
        tracker = ObjectTracker(self.tracker_type)
        success = tracker.init(frame, bbox)
        
        if success:
            tracker_id = self.next_id
            self.trackers[tracker_id] = tracker
            self.next_id += 1
            return tracker_id
        
        return -1
    
    def update_all(self, frame: np.ndarray) -> Dict[int, Tuple[int, int, int, int]]:
        """
        Tüm tracker'ları güncelle
        
        Args:
            frame: Frame
            
        Returns:
            {tracker_id: bbox}
        """
        results = {}
        to_remove = []
        
        for tracker_id, tracker in self.trackers.items():
            success, bbox = tracker.update(frame)
            
            if success:
                results[tracker_id] = bbox
            else:
                to_remove.append(tracker_id)
        
        # Başarısız tracker'ları kaldır
        for tracker_id in to_remove:
            del self.trackers[tracker_id]
        
        return results
    
    def remove_tracker(self, tracker_id: int):
        """Tracker'ı kaldır"""
        if tracker_id in self.trackers:
            del self.trackers[tracker_id]
    
    def reset_all(self):
        """Tüm tracker'ları sıfırla"""
        self.trackers.clear()
        self.next_id = 0


if __name__ == "__main__":
    # Test
    print("=== Tracker Testi ===\n")
    print("Kullanım:")
    print("  - 's' tuşu: Tracker başlat (mouse ile bbox seç)")
    print("  - 'r' tuşu: Tracker sıfırla")
    print("  - ESC: Çıkış\n")
    
    # Video kaynağı
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Kamera açılamadı!")
        exit()
    
    # Tracker oluştur
    tracker = ObjectTracker('CSRT')
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Tracker aktifse güncelle
        if tracker.is_tracking:
            success, bbox = tracker.update(frame)
            
            if success:
                # Bbox çiz
                x1, y1, x2, y2 = bbox
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # İz çiz
                frame = tracker.draw_trajectory(frame)
                
                # Bilgi göster
                frame = tracker.draw_info(frame)
        
        # Göster
        cv2.imshow('Tracker Test', frame)
        
        # Tuş kontrolü
        key = cv2.waitKey(1) & 0xFF
        
        if key == 27:  # ESC
            break
        elif key == ord('s'):  # Tracker başlat
            bbox = cv2.selectROI('Tracker Test', frame, False)
            if bbox[2] > 0 and bbox[3] > 0:
                x, y, w, h = bbox
                tracker.init(frame, (x, y, x+w, y+h))
        elif key == ord('r'):  # Reset
            tracker.reset()
    
    # İstatistikler
    print("\n=== Tracker İstatistikleri ===")
    stats = tracker.get_stats()
    for key, value in stats.items():
        print(f"{key}: {value}")
    
    cap.release()
    cv2.destroyAllWindows()
