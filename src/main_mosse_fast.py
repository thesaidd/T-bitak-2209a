"""
MOSSE Optimize Mod - Maksimum Hiz
En yuksek performans icin optimize edilmis sistem
"""

import cv2
import numpy as np
import argparse
import time
from pathlib import Path

# Kendi modullerimiz
from screen_capture import create_capture
from detector import UAVDetector
from robust_tracker import RobustTracker
from kalman_filter import UAVKalmanFilter
from test_logger import TestLogger
import config

# MOSSE profilini uygula
config.apply_profile('mosse_fast')


class MOSSEOptimizedSystem:
    """MOSSE icin optimize edilmis tracking sistemi"""
    
    def __init__(self, args):
        self.args = args
        
        print(f"\n{'='*60}")
        print("IHA TESPIT VE TAKIP SISTEMI - MOSSE OPTIMIZE MOD")
        print(f"{'='*60}\n")
        
        # Goruntu kaynagi
        print("1. Goruntu kaynagi baslatiliyor...")
        self.capture = create_capture(args.source)
        
        # Detector
        print("\n2. Tespit modulu yukleniyor...")
        target_classes = list(config.MODEL_CONFIG['target_classes'].keys())
        self.detector = UAVDetector(
            model_path=args.model,
            conf_threshold=0.4,  # Dusuk esik - daha fazla tespit
            iou_threshold=0.4,
            device='cpu',
            target_classes=target_classes
        )
        
        # Tracker (MOSSE optimize)
        print("\n3. MOSSE Tracker baslatiliyor (OPTIMIZE)...")
        self.tracker = RobustTracker(
            tracker_type='MOSSE',
            trajectory_length=20,  # Daha kisa iz - daha hizli
            max_bbox_change=0.6,   # Daha esnek
            min_bbox_area=50       # Daha kucuk minimum
        )
        
        # Kalman Filter (optimize)
        print("\n4. Kalman filter baslatiliyor (OPTIMIZE)...")
        self.kalman = UAVKalmanFilter(
            dt=1.0,
            process_noise=0.01,    # Daha yuksek - hizli degisimlere uyum
            measurement_noise=0.1
        )
        
        # Test Logger
        self.test_logger = TestLogger()
        self.test_logger.log_parameters({
            'mode': 'MOSSE_OPTIMIZED',
            'source': args.source,
            'model': args.model,
            'conf_threshold': 0.4,
            'tracker': 'MOSSE',
            'use_kalman': True,
            'resize_input': True,
            'input_size': (416, 416)
        })
        self.test_logger.log_event('START', 'MOSSE optimize mod baslatildi')
        
        # Video kayit
        self.save_video = args.save
        self.video_writer = None
        
        if self.save_video:
            output_dir = Path('results')
            output_dir.mkdir(exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            self.output_path = output_dir / f"mosse_fast_{timestamp}.mp4"
        
        # Durum degiskenleri
        self.running = True
        self.paused = False
        self.frame_count = 0
        self.detection_mode = True
        
        # FPS hesaplama
        self.fps_start_time = time.time()
        self.fps_frame_count = 0
        self.current_fps = 0
        
        print(f"\n{'='*60}")
        print("SISTEM HAZIR - MOSSE OPTIMIZE MOD!")
        print(f"{'='*60}\n")
        print("Optimizasyonlar:")
        print("  - MOSSE Tracker (En hizli)")
        print("  - Resize: 416x416 (Hizli islem)")
        print("  - Kalman: Optimize edilmis")
        print("  - Trajectory: 20 frame (Hafif)")
        print("\nBeklenen Performans:")
        print("  - FPS: 30-40")
        print("  - Tracking Orani: %65-75")
        print("  - Latency: Cok dusuk")
        print()
        self.print_controls()
    
    def print_controls(self):
        """Kontrol tuslarini yazdir"""
        print("KONTROLLER:")
        print("  SPACE : Duraklat/Devam")
        print("  'r'   : Tracker'i sifirla")
        print("  'd'   : Tespit moduna gec")
        print("  't'   : Takip moduna gec")
        print("  's'   : Manuel bbox secimi")
        print("  ESC   : Cikis")
        print()
    
    def run(self):
        """Ana dongu"""
        
        while self.running:
            # Frame yakala
            frame = self.capture.capture()
            
            if frame is None:
                print("Frame alinamadi, cikiliyor...")
                break
            
            # Pause kontrolu
            if self.paused:
                cv2.imshow('MOSSE Optimize - UAV Tracking', frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):
                    self.paused = False
                elif key == 27:
                    break
                continue
            
            # Orijinal frame'i sakla (gorsellestirme icin)
            frame_original = frame.copy()
            
            # Resize (SADECE tespit icin)
            frame_detect = cv2.resize(frame, (416, 416))
            
            # Isle (orijinal frame'de tracking, kucuk frame'de detection)
            frame_vis, detections_count, tracking_active = self.process_frame(
                frame_original, frame_detect
            )
            
            # FPS hesapla
            self.update_fps()
            
            # Test logger'a kaydet
            confidence = self.tracker.get_confidence() if hasattr(self.tracker, 'get_confidence') else None
            
            self.test_logger.log_frame_stats(
                frame_num=self.frame_count,
                detections=detections_count,
                tracking=tracking_active,
                fps=self.current_fps,
                confidence=confidence
            )
            
            # Bilgileri ciz
            frame_vis = self.draw_info(frame_vis)
            
            # Video kaydet
            if self.save_video:
                self.save_frame(frame_vis)
            
            # Goster
            cv2.imshow('MOSSE Optimize - UAV Tracking', frame_vis)
            
            # Tus kontrolu
            key = cv2.waitKey(1) & 0xFF
            if not self.handle_key(key, frame_original):
                break
            
            self.frame_count += 1
        
        # Temizlik
        self.cleanup()
    
    def process_frame(self, frame_original, frame_detect):
        """
        Frame'i isle
        
        Args:
            frame_original: Orijinal boyutta frame (tracking ve gorsellestirme icin)
            frame_detect: Kucuk boyutta frame (detection icin)
        
        Returns:
            (frame_vis, detections_count, tracking_active)
        """
        frame_vis = frame_original.copy()
        detections_count = 0
        tracking_active = False
        
        # Olcek faktoru (bbox koordinatlarini donusturmek icin)
        scale_x = frame_original.shape[1] / frame_detect.shape[1]
        scale_y = frame_original.shape[0] / frame_detect.shape[0]
        
        if self.detection_mode or not self.tracker.is_tracking:
            # TESPIT MODU (kucuk frame'de tespit)
            detections = self.detector.detect(frame_detect)
            detections_count = len(detections)
            
            if detections:
                # Bbox'lari orijinal boyuta donustur
                for det in detections:
                    x1, y1, x2, y2 = det['bbox']
                    det['bbox'] = (
                        int(x1 * scale_x),
                        int(y1 * scale_y),
                        int(x2 * scale_x),
                        int(y2 * scale_y)
                    )
                    det['center'] = (
                        int(det['center'][0] * scale_x),
                        int(det['center'][1] * scale_y)
                    )
                
                best_det = self.detector.get_best_detection(detections)
                
                # Orijinal boyutta ciz
                frame_vis = self.detector.draw_detections(
                    frame_vis, detections,
                    color=(0, 255, 0), thickness=2
                )
                
                if not self.tracker.is_tracking and best_det:
                    # Orijinal boyutta tracker baslat
                    self.tracker.init(frame_original, best_det['bbox'])
                    self.test_logger.log_event('TRACKER_INIT', f'Tracker baslatildi: {best_det["name"]}')
                    
                    center = best_det['center']
                    self.kalman.init(center[0], center[1])
                    
                    self.detection_mode = False
        
        else:
            # TAKIP MODU (orijinal boyutta tracking)
            success, bbox = self.tracker.update(frame_original)
            tracking_active = success
            
            if success:
                x1, y1, x2, y2 = bbox
                cv2.rectangle(frame_vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                frame_vis = self.tracker.draw_trajectory(frame_vis, color=(255, 0, 0), thickness=2)
                
                center = self.tracker.last_center
                if center:
                    filtered_pos = self.kalman.update(center)
                    predictions = self.kalman.predict_ahead(3)  # Sadece 3 frame
                    
                    if predictions:
                        for i, (px, py) in enumerate(predictions):
                            alpha = 1.0 - (i / len(predictions))
                            color = tuple(int(c * alpha) for c in (0, 0, 255))
                            cv2.circle(frame_vis, (int(px), int(py)), 3, color, -1)
                
                frame_vis = self.tracker.draw_info(frame_vis)
            
            else:
                print("Hedef kaybedildi, tespit moduna geciliyor...")
                self.test_logger.log_event('TRACKER_LOST', 'Hedef kaybedildi')
                self.detection_mode = True
                self.tracker.reset()
                self.kalman.reset()
        
        return frame_vis, detections_count, tracking_active
    
    def update_fps(self):
        """FPS hesapla"""
        self.fps_frame_count += 1
        
        if self.fps_frame_count >= 10:
            elapsed = time.time() - self.fps_start_time
            self.current_fps = self.fps_frame_count / elapsed
            
            self.fps_start_time = time.time()
            self.fps_frame_count = 0
    
    def draw_info(self, frame):
        """Bilgileri frame uzerine ciz"""
        mode_text = "DETECTION" if self.detection_mode else "TRACKING"
        mode_color = (0, 165, 255) if self.detection_mode else (0, 255, 0)
        
        cv2.putText(frame, f"Mode: {mode_text}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
        
        cv2.putText(frame, f"FPS: {self.current_fps:.1f}", (10, frame.shape[0] - 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.putText(frame, f"Frame: {self.frame_count}", (10, frame.shape[0] - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # MOSSE optimize badge
        cv2.putText(frame, "MOSSE OPTIMIZE", (frame.shape[1] - 200, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return frame
    
    def handle_key(self, key, frame):
        """Tus basimlarini isle"""
        if key == 27:  # ESC
            return False
        elif key == ord(' '):
            self.paused = not self.paused
        elif key == ord('r'):
            self.tracker.reset()
            self.kalman.reset()
            self.detection_mode = True
        elif key == ord('d'):
            self.detection_mode = True
        elif key == ord('t'):
            if self.tracker.is_tracking:
                self.detection_mode = False
        elif key == ord('s'):
            bbox = cv2.selectROI('MOSSE Optimize - UAV Tracking', frame, False)
            if bbox[2] > 0 and bbox[3] > 0:
                x, y, w, h = bbox
                self.tracker.init(frame, (x, y, x+w, y+h))
                self.kalman.init(x + w/2, y + h/2)
                self.detection_mode = False
        
        return True
    
    def save_frame(self, frame):
        """Frame'i video dosyasina kaydet"""
        if self.video_writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 30
            size = (frame.shape[1], frame.shape[0])
            
            self.video_writer = cv2.VideoWriter(
                str(self.output_path), fourcc, fps, size
            )
            print(f"Video kaydi baslatildi: {self.output_path}")
        
        self.video_writer.write(frame)
    
    def cleanup(self):
        """Kaynaklari temizle"""
        print("\nSistem kapatiliyor...")
        
        print("\n" + "="*60)
        print("ISTATISTIKLER")
        print("="*60)
        
        det_stats = self.detector.get_stats()
        print("\nTespit:")
        print(f"  Toplam frame: {det_stats['total_frames']}")
        print(f"  Toplam tespit: {det_stats['total_detections']}")
        print(f"  Ortalama FPS: {det_stats['avg_fps']:.1f}")
        
        self.test_logger.log_statistic('total_frames', det_stats['total_frames'])
        self.test_logger.log_statistic('total_detections', det_stats['total_detections'])
        self.test_logger.log_statistic('avg_fps', det_stats['avg_fps'])
        
        track_stats = self.tracker.get_stats()
        print("\nTakip:")
        print(f"  Takip edilen frame: {track_stats['frames_tracked']}")
        print(f"  Kayip frame: {track_stats['frames_lost']}")
        if track_stats['frames_tracked'] > 0:
            success_rate = track_stats['frames_tracked'] / (track_stats['frames_tracked'] + track_stats['frames_lost']) * 100
            print(f"  Basari orani: {success_rate:.1f}%")
            self.test_logger.log_statistic('tracking_success_rate', success_rate)
        
        self.test_logger.log_statistic('frames_tracked', track_stats['frames_tracked'])
        self.test_logger.log_statistic('frames_lost', track_stats['frames_lost'])
        
        if self.video_writer:
            self.video_writer.release()
            print(f"\nVideo kaydedildi: {self.output_path}")
            
            import shutil
            test_video_path = self.test_logger.get_test_dir() / self.output_path.name
            shutil.copy(self.output_path, test_video_path)
            self.test_logger.log_event('VIDEO_SAVED', f'Video kaydedildi: {test_video_path}')
        
        self.test_logger.log_event('END', 'Sistem kapatildi')
        self.test_logger.finalize()
        
        self.capture.close()
        cv2.destroyAllWindows()
        
        print("\nSistem kapatildi.")


def parse_args():
    """Komut satiri argumanlarini parse et"""
    parser = argparse.ArgumentParser(description='MOSSE Optimize Mod - Maksimum Hiz')
    
    parser.add_argument('--source', type=str, default='screen',
                       help='Goruntu kaynagi: "screen", webcam index, veya video dosyasi')
    parser.add_argument('--model', type=str, default='yolov8n.pt',
                       help='YOLOv8 model dosyasi')
    parser.add_argument('--save', action='store_true',
                       help='Sonuclari video olarak kaydet')
    
    return parser.parse_args()


def main():
    """Ana fonksiyon"""
    args = parse_args()
    
    system = MOSSEOptimizedSystem(args)
    system.run()


if __name__ == "__main__":
    main()
