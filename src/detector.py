"""
YOLOv8 Tespit Modülü
İHA tespiti için optimize edilmiş YOLOv8 implementasyonu
"""

from ultralytics import YOLO
import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
import time


class UAVDetector:
    """YOLOv8 tabanlı İHA tespit sınıfı"""
    
    def __init__(self, 
                 model_path: str = 'yolov8n.pt',
                 conf_threshold: float = 0.5,
                 iou_threshold: float = 0.45,
                 device: str = 'cpu',
                 target_classes: Optional[List[int]] = None):
        """
        Args:
            model_path: Model dosya yolu
            conf_threshold: Güven eşiği
            iou_threshold: NMS IoU eşiği
            device: 'cpu' veya 'cuda'
            target_classes: Tespit edilecek sınıf ID'leri (None = hepsi)
        """
        print(f"YOLOv8 modeli yükleniyor: {model_path}")
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.device = device
        self.target_classes = target_classes
        
        # İstatistikler
        self.total_detections = 0
        self.total_frames = 0
        self.avg_inference_time = 0
        
        # COCO sınıf isimleri
        self.class_names = self.model.names
        
        print(f"Model yüklendi: {model_path}")
        print(f"  Device: {device}")
        print(f"  Confidence threshold: {conf_threshold}")
        if target_classes:
            print(f"  Hedef sınıflar: {[self.class_names[c] for c in target_classes]}")
    
    def detect(self, frame: np.ndarray) -> List[Dict]:
        """
        Frame'de nesne tespiti yap
        
        Args:
            frame: BGR formatında görüntü
            
        Returns:
            Tespit listesi [{'bbox': (x1,y1,x2,y2), 'conf': float, 'class': int, 'name': str}]
        """
        start_time = time.time()
        
        # YOLOv8 inference
        results = self.model(
            frame,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False
        )
        
        # Inference zamanı
        inference_time = time.time() - start_time
        self.avg_inference_time = (self.avg_inference_time * self.total_frames + inference_time) / (self.total_frames + 1)
        self.total_frames += 1
        
        # Sonuçları parse et
        detections = []
        
        for result in results:
            boxes = result.boxes
            
            for box in boxes:
                # Sınıf ID
                class_id = int(box.cls[0])
                
                # Hedef sınıf kontrolü
                if self.target_classes and class_id not in self.target_classes:
                    continue
                
                # Bbox koordinatları
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                # Güven skoru
                confidence = float(box.conf[0])
                
                # Sınıf ismi
                class_name = self.class_names[class_id]
                
                detection = {
                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                    'conf': confidence,
                    'class': class_id,
                    'name': class_name,
                    'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                }
                
                detections.append(detection)
                self.total_detections += 1
        
        return detections
    
    def get_best_detection(self, detections: List[Dict]) -> Optional[Dict]:
        """
        En yüksek güven skoruna sahip tespiti döndür
        
        Args:
            detections: Tespit listesi
            
        Returns:
            En iyi tespit veya None
        """
        if not detections:
            return None
        
        return max(detections, key=lambda x: x['conf'])
    
    def filter_by_area(self, detections: List[Dict], min_area: int = 100) -> List[Dict]:
        """
        Minimum alan filtresi uygula
        
        Args:
            detections: Tespit listesi
            min_area: Minimum alan (piksel^2)
            
        Returns:
            Filtrelenmiş tespit listesi
        """
        filtered = []
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            area = (x2 - x1) * (y2 - y1)
            
            if area >= min_area:
                det['area'] = area
                filtered.append(det)
        
        return filtered
    
    def draw_detections(self, 
                       frame: np.ndarray, 
                       detections: List[Dict],
                       color: Tuple[int, int, int] = (0, 255, 0),
                       thickness: int = 2,
                       show_conf: bool = True) -> np.ndarray:
        """
        Tespitleri frame üzerine çiz
        
        Args:
            frame: Görüntü
            detections: Tespit listesi
            color: Bbox rengi (BGR)
            thickness: Çizgi kalınlığı
            show_conf: Güven skorunu göster
            
        Returns:
            Çizilmiş görüntü
        """
        frame_copy = frame.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            conf = det['conf']
            name = det['name']
            
            # Bbox çiz
            cv2.rectangle(frame_copy, (x1, y1), (x2, y2), color, thickness)
            
            # Label oluştur
            if show_conf:
                label = f"{name}: {conf:.2f}"
            else:
                label = name
            
            # Label arka planı
            (label_w, label_h), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            cv2.rectangle(
                frame_copy,
                (x1, y1 - label_h - 10),
                (x1 + label_w, y1),
                color,
                -1
            )
            
            # Label metni
            cv2.putText(
                frame_copy,
                label,
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )
            
            # Merkez noktası
            center = det['center']
            cv2.circle(frame_copy, center, 5, (0, 0, 255), -1)
        
        return frame_copy
    
    def get_stats(self) -> Dict:
        """
        İstatistikleri döndür
        
        Returns:
            İstatistik dictionary
        """
        return {
            'total_frames': self.total_frames,
            'total_detections': self.total_detections,
            'avg_detections_per_frame': self.total_detections / max(self.total_frames, 1),
            'avg_inference_time': self.avg_inference_time,
            'avg_fps': 1.0 / max(self.avg_inference_time, 0.001)
        }
    
    def print_stats(self):
        """İstatistikleri yazdır"""
        stats = self.get_stats()
        print("\n=== Tespit İstatistikleri ===")
        print(f"Toplam frame: {stats['total_frames']}")
        print(f"Toplam tespit: {stats['total_detections']}")
        print(f"Frame başına ortalama tespit: {stats['avg_detections_per_frame']:.2f}")
        print(f"Ortalama inference süresi: {stats['avg_inference_time']*1000:.1f} ms")
        print(f"Ortalama FPS: {stats['avg_fps']:.1f}")


if __name__ == "__main__":
    # Test
    print("=== YOLOv8 Tespit Testi ===\n")
    
    # Detector oluştur (COCO'da airplane ve bird sınıfları)
    detector = UAVDetector(
        model_path='yolov8n.pt',
        conf_threshold=0.5,
        target_classes=[4, 14]  # airplane, bird
    )
    
    # Test görüntüsü (webcam veya test videosu)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Kamera açılamadı!")
        exit()
    
    print("Tespit başladı (ESC ile çıkış)...\n")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Tespit
        detections = detector.detect(frame)
        
        # Çiz
        frame_vis = detector.draw_detections(frame, detections)
        
        # FPS göster
        stats = detector.get_stats()
        cv2.putText(
            frame_vis,
            f"FPS: {stats['avg_fps']:.1f} | Detections: {len(detections)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        
        # Göster
        cv2.imshow('YOLOv8 Detection Test', frame_vis)
        
        # ESC ile çık
        if cv2.waitKey(1) & 0xFF == 27:
            break
    
    # İstatistikler
    detector.print_stats()
    
    cap.release()
    cv2.destroyAllWindows()
