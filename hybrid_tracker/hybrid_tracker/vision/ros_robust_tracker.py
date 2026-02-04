"""
ROS Robust Tracker - Gelişmiş Stabilite Özellikleri
ROS 2 paketi için uyarlanmıştır
Orijinal src/robust_tracker.py'den uyarlanmıştır
"""

import cv2
import numpy as np
from typing import Optional, Tuple
from .ros_tracker import ObjectTracker


class RobustTracker(ObjectTracker):
    """
    Stabilite özellikleri ile geliştirilmiş tracker
    
    Yeni özellikler:
    - Bbox doğrulama (çok küçük/büyük bbox kontrolü)
    - Hareket kısıtlamaları (ani sıçramaları önler)
    - Güven skoru tahmini
    - Otomatik yeniden başlatma
    - Zamansal yumuşatma
    """
    
    def __init__(self, tracker_type: str = 'CSRT', trajectory_length: int = 30,
                 max_bbox_change: float = 0.5, min_bbox_area: int = 100):
        """
        Args:
            tracker_type: Tracker tipi
            trajectory_length: Trajectory uzunluğu
            max_bbox_change: Maksimum bbox değişim oranı (0-1)
            min_bbox_area: Minimum bbox alanı (piksel^2)
        """
        super().__init__(tracker_type, trajectory_length)
        
        self.max_bbox_change = max_bbox_change
        self.min_bbox_area = min_bbox_area
        
        # Stabilite için ek değişkenler
        self.confidence_history = []
        self.bbox_history = []
        self.max_history = 10
        
        # Ardışık frame sayacı
        self.consecutive_failures = 0
        self.max_consecutive_failures = 3
    
    def init(self, frame, bbox):
        """
        Tracker'ı başlat - doğrulama olmadan
        
        Args:
            frame: İlk frame
            bbox: İlk bbox (x1, y1, x2, y2)
        """
        # Parent init'i çağır
        success = super().init(frame, bbox)
        
        if success:
            # Bbox geçmişini başlat
            self.bbox_history = [bbox]
            self.consecutive_failures = 0
        
        return success
    
    def _validate_bbox(self, bbox: Tuple[int, int, int, int], 
                      frame_shape: Tuple[int, int]) -> bool:
        """
        Bbox'ın geçerli olup olmadığını kontrol et
        
        Args:
            bbox: (x1, y1, x2, y2)
            frame_shape: (yükseklik, genişlik)
            
        Returns:
            True: geçerli, False: geçersiz
        """
        x1, y1, x2, y2 = bbox
        h, w = frame_shape[:2]
        
        # Bbox sınırlar içinde mi?
        if x1 < 0 or y1 < 0 or x2 > w or y2 > h:
            return False
        
        # Bbox boyutu geçerli mi?
        bbox_w = x2 - x1
        bbox_h = y2 - y1
        
        if bbox_w <= 0 or bbox_h <= 0:
            return False
        
        # Minimum alan kontrolü
        area = bbox_w * bbox_h
        if area < self.min_bbox_area:
            return False
        
        # Maksimum alan kontrolü (frame'in %80'inden büyük olmamalı)
        max_area = w * h * 0.8
        if area > max_area:
            return False
        
        return True
    
    def _check_bbox_change(self, new_bbox: Tuple[int, int, int, int]) -> bool:
        """
        Bbox değişiminin makul olup olmadığını kontrol et
        
        Args:
            new_bbox: Yeni bbox
            
        Returns:
            True: makul, False: çok fazla değişim
        """
        if self.last_bbox is None:
            return True
        
        # Önceki bbox
        x1_old, y1_old, x2_old, y2_old = self.last_bbox
        w_old = x2_old - x1_old
        h_old = y2_old - y1_old
        
        # Yeni bbox
        x1_new, y1_new, x2_new, y2_new = new_bbox
        w_new = x2_new - x1_new
        h_new = y2_new - y1_new
        
        # Boyut değişimi
        w_change = abs(w_new - w_old) / w_old if w_old > 0 else 0
        h_change = abs(h_new - h_old) / h_old if h_old > 0 else 0
        
        # Pozisyon değişimi
        cx_old = (x1_old + x2_old) / 2
        cy_old = (y1_old + y2_old) / 2
        cx_new = (x1_new + x2_new) / 2
        cy_new = (y1_new + y2_new) / 2
        
        pos_change = np.sqrt((cx_new - cx_old)**2 + (cy_new - cy_old)**2)
        max_pos_change = max(w_old, h_old) * self.max_bbox_change
        
        # Kontroller
        if w_change > self.max_bbox_change or h_change > self.max_bbox_change:
            return False
        
        if pos_change > max_pos_change:
            return False
        
        return True
    
    def _smooth_bbox(self, new_bbox: Tuple[int, int, int, int]) -> Tuple[int, int, int, int]:
        """
        Zamansal yumuşatma ile bbox'ı yumuşat
        
        Args:
            new_bbox: Yeni bbox
            
        Returns:
            Yumuşatılmış bbox
        """
        if len(self.bbox_history) == 0:
            return new_bbox
        
        # Son 3 bbox'ın ortalaması
        recent_bboxes = self.bbox_history[-3:] + [new_bbox]
        
        x1_avg = int(np.mean([b[0] for b in recent_bboxes]))
        y1_avg = int(np.mean([b[1] for b in recent_bboxes]))
        x2_avg = int(np.mean([b[2] for b in recent_bboxes]))
        y2_avg = int(np.mean([b[3] for b in recent_bboxes]))
        
        return (x1_avg, y1_avg, x2_avg, y2_avg)
    
    def update(self, frame: np.ndarray) -> Tuple[bool, Optional[Tuple[int, int, int, int]]]:
        """
        Geliştirilmiş tracker güncelleme
        
        Args:
            frame: Yeni frame
            
        Returns:
            (başarı, bbox)
        """
        if not self.is_tracking or self.tracker is None:
            return False, None
        
        # Tracker'ı güncelle
        success, bbox_xywh = self.tracker.update(frame)
        
        if success:
            # Bbox formatını dönüştür
            x, y, w, h = [int(v) for v in bbox_xywh]
            bbox = (x, y, x + w, y + h)
            
            # Doğrulama kontrolleri
            if not self._validate_bbox(bbox, frame.shape):
                self.consecutive_failures += 1
                
                if self.consecutive_failures >= self.max_consecutive_failures:
                    self.is_tracking = False
                    self.frames_lost += 1
                
                return False, None
            
            # Bbox değişim kontrolü
            if not self._check_bbox_change(bbox):
                self.consecutive_failures += 1
                
                if self.consecutive_failures >= self.max_consecutive_failures:
                    self.is_tracking = False
                    self.frames_lost += 1
                
                return False, None
            
            # Yumuşatma uygula
            bbox = self._smooth_bbox(bbox)
            
            # Başarılı güncelleme
            self.consecutive_failures = 0
            
            # Bbox geçmişini güncelle
            self.bbox_history.append(bbox)
            if len(self.bbox_history) > self.max_history:
                self.bbox_history.pop(0)
            
            # Merkez nokta
            center = ((bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2)
            
            # Güncelle
            self.last_bbox = bbox
            self.last_center = center
            self.trajectory.append(center)
            self.frames_tracked += 1
            
            return True, bbox
        
        else:
            # Tracker başarısız
            self.consecutive_failures += 1
            
            if self.consecutive_failures >= self.max_consecutive_failures:
                self.frames_lost += 1
                self.is_tracking = False
            
            return False, None
    
    def reset(self):
        """Tracker'ı sıfırla"""
        super().reset()
        self.confidence_history.clear()
        self.bbox_history.clear()
        self.consecutive_failures = 0
    
    def get_confidence(self) -> float:
        """
        Tracker güven skorunu tahmin et
        
        Returns:
            Güven skoru (0-1)
        """
        if not self.is_tracking:
            return 0.0
        
        # Faktörler:
        # 1. Ardışık başarı oranı
        success_factor = 1.0 - (self.consecutive_failures / self.max_consecutive_failures)
        
        # 2. Trajectory uzunluğu (uzun = daha güvenilir)
        trajectory_factor = min(len(self.trajectory) / self.trajectory_length, 1.0)
        
        # 3. Bbox stabilitesi
        stability_factor = 1.0
        if len(self.bbox_history) >= 2:
            # Son iki bbox arasındaki değişim
            last_bbox = self.bbox_history[-1]
            prev_bbox = self.bbox_history[-2]
            
            w1 = last_bbox[2] - last_bbox[0]
            h1 = last_bbox[3] - last_bbox[1]
            w2 = prev_bbox[2] - prev_bbox[0]
            h2 = prev_bbox[3] - prev_bbox[1]
            
            size_change = abs(w1 * h1 - w2 * h2) / (w2 * h2) if w2 * h2 > 0 else 0
            stability_factor = max(0, 1.0 - size_change)
        
        # Ağırlıklı ortalama
        confidence = (success_factor * 0.5 + 
                     trajectory_factor * 0.3 + 
                     stability_factor * 0.2)
        
        return confidence
