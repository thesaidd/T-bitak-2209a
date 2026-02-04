"""
ROS YOLOv8 Tespit Modülü
ROS 2 entegrasyonu için optimize edilmiş
Orijinal src/detector.py'den uyarlanmıştır
"""

from ultralytics import YOLO
import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
import time


class UAVDetector:
    """ROS 2 için optimize edilmiş YOLOv8 tabanlı İHA tespit sınıfı"""
    
    def __init__(self, 
                 model_path: str = 'yolov8n.pt',
                 conf_threshold: float = 0.5,
                 iou_threshold: float = 0.45,
                 device: str = 'cpu',
                 target_classes: Optional[List[int]] = None,
                 verbose: bool = False):
        """
        Args:
            model_path: Model dosya yolu
            conf_threshold: Güven eşiği
            iou_threshold: NMS IoU eşiği
            device: 'cpu' veya 'cuda'
            target_classes: Tespit edilecek sınıf ID'leri (None = hepsi)
            verbose: Başlatma bilgilerini yazdır
        """
        if verbose:
            print(f"YOLOv8 modeli yükleniyor: {model_path}")
        
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.device = device
        self.target_classes = target_classes
        
        # Statistics
        self.total_detections = 0
        self.total_frames = 0
        self.avg_inference_time = 0
        
        # COCO sınıf isimleri
        self.class_names = self.model.names
        
        if verbose:
            print(f"Model yüklendi: {model_path}")
            print(f"  Cihaz: {device}")
            print(f"  Güven eşiği: {conf_threshold}")
            if target_classes:
                print(f"  Hedef sınıflar: {[self.class_names[c] for c in target_classes]}")
    
    def detect(self, frame: np.ndarray) -> List[Dict]:
        """
        Frame üzerinde nesne tespiti yap
        
        Args:
            frame: BGR formatında görüntü
            
        Returns:
            Tespit listesi [{'bbox': (x1,y1,x2,y2), 'conf': float, 'class': int, 'name': str}]
        """
        start_time = time.time()
        
        # YOLOv8 çıkarımı
        results = self.model(
            frame,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False
        )
        
        # Çıkarım zamanı
        inference_time = time.time() - start_time
        self.avg_inference_time = (self.avg_inference_time * self.total_frames + inference_time) / (self.total_frames + 1)
        self.total_frames += 1
        
        # Sonuçları ayrıştır
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
        Return detection with highest confidence
        
        Args:
            detections: Detection list
            
        Returns:
            Best detection or None
        """
        if not detections:
            return None
        
        return max(detections, key=lambda x: x['conf'])
    
    def filter_by_area(self, detections: List[Dict], min_area: int = 100) -> List[Dict]:
        """
        Apply minimum area filter
        
        Args:
            detections: Detection list
            min_area: Minimum area (pixels^2)
            
        Returns:
            Filtered detection list
        """
        filtered = []
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            area = (x2 - x1) * (y2 - y1)
            
            if area >= min_area:
                det['area'] = area
                filtered.append(det)
        
        return filtered
    
    def get_stats(self) -> Dict:
        """
        Get statistics
        
        Returns:
            Statistics dictionary
        """
        return {
            'total_frames': self.total_frames,
            'total_detections': self.total_detections,
            'avg_detections_per_frame': self.total_detections / max(self.total_frames, 1),
            'avg_inference_time': self.avg_inference_time,
            'avg_fps': 1.0 / max(self.avg_inference_time, 0.001)
        }
