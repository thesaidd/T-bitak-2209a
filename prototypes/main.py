"""
Ana Program - İHA Tespit ve Takip Sistemi
Tüm modülleri birleştiren ana uygulama
"""

import cv2
import numpy as np
import argparse
import time
import os
from pathlib import Path

# Kendi modüllerimiz
from screen_capture import create_capture
from detector import UAVDetector
from robust_tracker import RobustTracker
from kalman_filter import UAVKalmanFilter
from test_logger import TestLogger
import config


class UAVTrackingSystem:
    """İHA tespit ve takip sistemi ana sınıfı"""
    
    def __init__(self, args):
        """
        Args:
            args: Komut satırı argümanları
        """
        self.args = args
        
        # Görüntü kaynağı
        print(f"\n{'='*60}")
        print("İHA TESPİT VE TAKİP SİSTEMİ")
        print(f"{'='*60}\n")
        
        print("1. Görüntü kaynağı başlatılıyor...")
        self.capture = create_capture(args.source)
        
        # Detector
        print("\n2. Tespit modülü yükleniyor...")
        target_classes = list(config.MODEL_CONFIG['target_classes'].keys()) if not args.all_classes else None
        self.detector = UAVDetector(
            model_path=args.model,
            conf_threshold=args.conf,
            iou_threshold=config.MODEL_CONFIG['iou_threshold'],
            device=config.MODEL_CONFIG['device'],
            target_classes=target_classes
        )
        
        # Tracker (Robust version ile stabilite artırımı)
        print("\n3. Takip modülü başlatılıyor...")
        self.tracker = RobustTracker(
            tracker_type=args.tracker,
            trajectory_length=config.VISUALIZATION_CONFIG['trajectory_length'],
            max_bbox_change=0.3,  # %30 maksimum değişim
            min_bbox_area=100  # Minimum 10x10 piksel
        )
        
        # Kalman Filter
        self.use_kalman = args.use_kalman
        if self.use_kalman:
            print("\n4. Kalman filter başlatılıyor...")
            self.kalman = UAVKalmanFilter(
                dt=1.0,
                process_noise=config.KALMAN_CONFIG['process_noise'],
                measurement_noise=config.KALMAN_CONFIG['measurement_noise']
            )
        else:
            self.kalman = None
        
        # Video kayıt
        self.save_video = args.save
        self.video_writer = None
        
        if self.save_video:
            # Çıktı dizini oluştur
            output_dir = Path(config.RECORDING_CONFIG['output_dir'])
            output_dir.mkdir(exist_ok=True)
            
            # Video dosya adı
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            self.output_path = output_dir / f"tracking_{timestamp}.mp4"
        
        # Test Logger
        self.test_logger = TestLogger()
        self.test_logger.log_parameters({
            'source': args.source,
            'model': args.model,
            'conf_threshold': args.conf,
            'tracker': args.tracker,
            'use_kalman': args.use_kalman,
            'save_video': args.save
        })
        self.test_logger.log_event('START', 'Sistem başlatıldı')
        
        # Durum değişkenleri
        self.running = True
        self.paused = False
        self.frame_count = 0
        self.detection_mode = True  # True: tespit modu, False: takip modu
        
        # FPS hesaplama
        self.fps_start_time = time.time()
        self.fps_frame_count = 0
        self.current_fps = 0
        
        print(f"\n{'='*60}")
        print("SİSTEM HAZIR!")
        print(f"{'='*60}\n")
        self.print_controls()
    
    def print_controls(self):
        """Kontrol tuşlarını yazdır"""
        print("KONTROLLER:")
        print("  SPACE : Duraklat/Devam")
        print("  'r'   : Tracker'ı sıfırla")
        print("  'd'   : Tespit moduna geç")
        print("  't'   : Takip moduna geç")
        print("  's'   : Manuel bbox seçimi")
        print("  ESC   : Çıkış")
        print()
    
    def run(self):
        """Ana döngü"""
        
        while self.running:
            # Frame yakala
            frame = self.capture.capture()
            
            if frame is None:
                print("Frame alınamadı, çıkılıyor...")
                break
            
            # Pause kontrolü
            if self.paused:
                cv2.imshow('UAV Tracking System', frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):
                    self.paused = False
                elif key == 27:
                    break
                continue
            
            # Orijinal frame'i sakla (video kayıt için)
            frame_original = frame.copy()
            
            # Resize (sadece işleme için, gösterim için değil)
            frame_process = frame
            if config.PERFORMANCE_CONFIG['resize_input']:
                frame_process = cv2.resize(frame, config.PERFORMANCE_CONFIG['input_size'])
            
            # İşle
            frame_vis, detections_count, tracking_active = self.process_frame(frame_process)
            
            # Görselleştirmeyi orijinal boyuta getir
            if config.PERFORMANCE_CONFIG['resize_input']:
                frame_vis = cv2.resize(frame_vis, (frame_original.shape[1], frame_original.shape[0]))
            
            # FPS hesapla
            self.update_fps()
            
            # Test logger'a kaydet
            confidence = None
            if hasattr(self.tracker, 'get_confidence'):
                confidence = self.tracker.get_confidence()
            
            self.test_logger.log_frame_stats(
                frame_num=self.frame_count,
                detections=detections_count,
                tracking=tracking_active,
                fps=self.current_fps,
                confidence=confidence
            )
            
            # Bilgileri çiz
            frame_vis = self.draw_info(frame_vis)
            
            # Video kaydet
            if self.save_video:
                self.save_frame(frame_vis)
            
            # Göster
            cv2.imshow('UAV Tracking System', frame_vis)
            
            # Tuş kontrolü
            key = cv2.waitKey(1) & 0xFF
            if not self.handle_key(key, frame):
                break
            
            self.frame_count += 1
        
        # Temizlik
        self.cleanup()
    
    def process_frame(self, frame):
        """
        Frame'i işle
        
        Args:
            frame: Giriş frame'i
            
        Returns:
            (İşlenmiş frame, tespit sayısı, tracking aktif mi)
        """
        frame_vis = frame.copy()
        detections_count = 0
        tracking_active = False
        
        if self.detection_mode or not self.tracker.is_tracking:
            # TESPİT MODU
            detections = self.detector.detect(frame)
            detections_count = len(detections)
            
            if detections:
                # En iyi tespiti al
                best_det = self.detector.get_best_detection(detections)
                
                # Tespitleri çiz
                frame_vis = self.detector.draw_detections(
                    frame_vis,
                    detections,
                    color=config.VISUALIZATION_CONFIG['bbox_color'],
                    thickness=config.VISUALIZATION_CONFIG['bbox_thickness']
                )
                
                # Otomatik tracker başlat
                if not self.tracker.is_tracking and best_det:
                    self.tracker.init(frame, best_det['bbox'])
                    self.test_logger.log_event('TRACKER_INIT', f'Tracker başlatıldı: {best_det["name"]}')
                    
                    # Kalman filter başlat
                    if self.kalman:
                        center = best_det['center']
                        self.kalman.init(center[0], center[1])
                    
                    # Takip moduna geç
                    self.detection_mode = False
        
        else:
            # TAKİP MODU
            success, bbox = self.tracker.update(frame)
            tracking_active = success
            
            if success:
                # Bbox çiz
                x1, y1, x2, y2 = bbox
                cv2.rectangle(
                    frame_vis,
                    (x1, y1), (x2, y2),
                    config.VISUALIZATION_CONFIG['bbox_color'],
                    config.VISUALIZATION_CONFIG['bbox_thickness']
                )
                
                # İz çiz
                frame_vis = self.tracker.draw_trajectory(
                    frame_vis,
                    color=config.VISUALIZATION_CONFIG['trajectory_color'],
                    thickness=2
                )
                
                # Kalman filter güncelle ve tahmin
                if self.kalman:
                    center = self.tracker.last_center
                    if center:
                        # Güncelle
                        filtered_pos = self.kalman.update(center)
                        
                        # İlerideki tahminleri çiz
                        predictions = self.kalman.predict_ahead(
                            config.KALMAN_CONFIG['prediction_steps']
                        )
                        
                        if predictions:
                            # Tahmin noktalarını çiz
                            for i, (px, py) in enumerate(predictions):
                                alpha = 1.0 - (i / len(predictions))
                                color = tuple(int(c * alpha) for c in config.VISUALIZATION_CONFIG['prediction_color'])
                                cv2.circle(frame_vis, (int(px), int(py)), 3, color, -1)
                            
                            # Tahmin çizgisi
                            points = [(int(p[0]), int(p[1])) for p in predictions]
                            if center:
                                points = [center] + points
                            
                            for i in range(len(points) - 1):
                                cv2.line(
                                    frame_vis,
                                    points[i], points[i+1],
                                    config.VISUALIZATION_CONFIG['prediction_color'],
                                    1
                                )
                
                # Tracker bilgilerini göster
                frame_vis = self.tracker.draw_info(frame_vis)
            
            else:
                # Tracker kaybetti, tespit moduna dön
                print("Hedef kaybedildi, tespit moduna geçiliyor...")
                self.test_logger.log_event('TRACKER_LOST', 'Hedef kaybedildi')
                self.detection_mode = True
                self.tracker.reset()
                if self.kalman:
                    self.kalman.reset()
        
        return frame_vis, detections_count, tracking_active
    
    def update_fps(self):
        """FPS hesapla"""
        self.fps_frame_count += 1
        
        if self.fps_frame_count >= 10:
            elapsed = time.time() - self.fps_start_time
            self.current_fps = self.fps_frame_count / elapsed
            
            # Sıfırla
            self.fps_start_time = time.time()
            self.fps_frame_count = 0
    
    def draw_info(self, frame):
        """
        Bilgileri frame üzerine çiz
        
        Args:
            frame: Frame
            
        Returns:
            Çizilmiş frame
        """
        # Mod
        mode_text = "DETECTION" if self.detection_mode else "TRACKING"
        mode_color = (0, 165, 255) if self.detection_mode else (0, 255, 0)
        
        cv2.putText(
            frame,
            f"Mode: {mode_text}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            mode_color,
            2
        )
        
        # FPS
        cv2.putText(
            frame,
            f"FPS: {self.current_fps:.1f}",
            (10, frame.shape[0] - 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )
        
        # Frame sayısı
        cv2.putText(
            frame,
            f"Frame: {self.frame_count}",
            (10, frame.shape[0] - 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )
        
        # Kalman bilgisi
        if self.kalman and self.kalman.initialized:
            speed = self.kalman.get_speed()
            cv2.putText(
                frame,
                f"Kalman Speed: {speed:.1f} px/f",
                (10, frame.shape[0] - 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2
            )
        
        return frame
    
    def handle_key(self, key, frame):
        """
        Tuş basımlarını işle
        
        Args:
            key: Basılan tuş
            frame: Mevcut frame
            
        Returns:
            True: devam, False: çık
        """
        if key == 27:  # ESC
            return False
        
        elif key == ord(' '):  # SPACE - pause
            self.paused = not self.paused
            print("PAUSED" if self.paused else "RESUMED")
        
        elif key == ord('r'):  # Reset tracker
            self.tracker.reset()
            if self.kalman:
                self.kalman.reset()
            self.detection_mode = True
            print("Tracker sıfırlandı")
        
        elif key == ord('d'):  # Detection mode
            self.detection_mode = True
            print("Tespit moduna geçildi")
        
        elif key == ord('t'):  # Tracking mode
            if self.tracker.is_tracking:
                self.detection_mode = False
                print("Takip moduna geçildi")
            else:
                print("Önce bir hedef tespit edilmeli!")
        
        elif key == ord('s'):  # Manuel bbox seçimi
            bbox = cv2.selectROI('UAV Tracking System', frame, False)
            if bbox[2] > 0 and bbox[3] > 0:
                x, y, w, h = bbox
                self.tracker.init(frame, (x, y, x+w, y+h))
                
                if self.kalman:
                    self.kalman.init(x + w/2, y + h/2)
                
                self.detection_mode = False
                print("Manuel tracker başlatıldı")
        
        return True
    
    def save_frame(self, frame):
        """Frame'i video dosyasına kaydet"""
        if self.video_writer is None:
            # Video writer oluştur
            fourcc = cv2.VideoWriter_fourcc(*config.RECORDING_CONFIG['video_codec'])
            fps = config.RECORDING_CONFIG['video_fps']
            size = (frame.shape[1], frame.shape[0])
            
            self.video_writer = cv2.VideoWriter(
                str(self.output_path),
                fourcc,
                fps,
                size
            )
            print(f"Video kaydı başlatıldı: {self.output_path}")
        
        self.video_writer.write(frame)
    
    def cleanup(self):
        """Kaynakları temizle"""
        print("\nSistem kapatılıyor...")
        
        # İstatistikler
        print("\n" + "="*60)
        print("İSTATİSTİKLER")
        print("="*60)
        
        # Detector stats
        det_stats = self.detector.get_stats()
        print("\nTespit:")
        print(f"  Toplam frame: {det_stats['total_frames']}")
        print(f"  Toplam tespit: {det_stats['total_detections']}")
        print(f"  Ortalama tespit/frame: {det_stats['avg_detections_per_frame']:.2f}")
        print(f"  Ortalama FPS: {det_stats['avg_fps']:.1f}")
        
        # Test logger'a kaydet
        self.test_logger.log_statistic('total_frames', det_stats['total_frames'])
        self.test_logger.log_statistic('total_detections', det_stats['total_detections'])
        self.test_logger.log_statistic('avg_fps', det_stats['avg_fps'])
        
        # Tracker stats
        track_stats = self.tracker.get_stats()
        print("\nTakip:")
        print(f"  Takip edilen frame: {track_stats['frames_tracked']}")
        print(f"  Kayıp frame: {track_stats['frames_lost']}")
        if track_stats['frames_tracked'] > 0:
            success_rate = track_stats['frames_tracked'] / (track_stats['frames_tracked'] + track_stats['frames_lost']) * 100
            print(f"  Başarı oranı: {success_rate:.1f}%")
            self.test_logger.log_statistic('tracking_success_rate', success_rate)
        
        self.test_logger.log_statistic('frames_tracked', track_stats['frames_tracked'])
        self.test_logger.log_statistic('frames_lost', track_stats['frames_lost'])
        
        # Video kayıt
        if self.video_writer:
            self.video_writer.release()
            print(f"\nVideo kaydedildi: {self.output_path}")
            # Video'yu test klasörüne kopyala
            import shutil
            test_video_path = self.test_logger.get_test_dir() / self.output_path.name
            shutil.copy(self.output_path, test_video_path)
            self.test_logger.log_event('VIDEO_SAVED', f'Video kaydedildi: {test_video_path}')
        
        # Test sonuçlarını kaydet
        self.test_logger.log_event('END', 'Sistem kapatıldı')
        self.test_logger.finalize()
        
        # Kaynakları serbest bırak
        self.capture.close()
        cv2.destroyAllWindows()
        
        print("\nSistem kapatıldı.")


def parse_args():
    """Komut satırı argümanlarını parse et"""
    parser = argparse.ArgumentParser(
        description='İHA Tespit ve Takip Sistemi',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Örnekler:
  # Ekran görüntüsünden tespit
  python main.py --source screen
  
  # Video dosyasından tespit
  python main.py --source video.mp4
  
  # Webcam'den tespit
  python main.py --source 0
  
  # Özel model ve ayarlar
  python main.py --source screen --model custom.pt --conf 0.6 --tracker KCF
        """
    )
    
    parser.add_argument(
        '--source',
        type=str,
        default='screen',
        help='Görüntü kaynağı: "screen", webcam index (0,1,...), veya video dosyası'
    )
    
    parser.add_argument(
        '--model',
        type=str,
        default='yolov8n.pt',
        help='YOLOv8 model dosyası (yolov8n.pt, yolov8s.pt, custom.pt)'
    )
    
    parser.add_argument(
        '--conf',
        type=float,
        default=0.5,
        help='Tespit güven eşiği (0.0-1.0)'
    )
    
    parser.add_argument(
        '--tracker',
        type=str,
        default='CSRT',
        choices=['CSRT', 'KCF', 'MOSSE', 'MIL'],
        help='Tracker tipi'
    )
    
    parser.add_argument(
        '--use-kalman',
        action='store_true',
        help='Kalman filter kullan'
    )
    
    parser.add_argument(
        '--save',
        action='store_true',
        help='Sonuçları video olarak kaydet'
    )
    
    parser.add_argument(
        '--all-classes',
        action='store_true',
        help='Tüm COCO sınıflarını tespit et (sadece airplane/bird değil)'
    )
    
    return parser.parse_args()


def main():
    """Ana fonksiyon"""
    # Argümanları parse et
    args = parse_args()
    
    # Sistemi oluştur ve çalıştır
    system = UAVTrackingSystem(args)
    system.run()


if __name__ == "__main__":
    main()
