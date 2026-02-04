"""
Test Sonuçları Kayıt Modülü
Her test sonucunu otomatik olarak kaydeder
"""

import os
import json
from datetime import datetime
from pathlib import Path
from typing import Dict, Any


class TestLogger:
    """Test sonuçlarını kaydeden sınıf"""
    
    def __init__(self, base_dir: str = "testler"):
        """
        Args:
            base_dir: Test sonuçlarının kaydedileceği klasör
        """
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(exist_ok=True)
        
        # Test numarasını belirle
        self.test_number = self._get_next_test_number()
        self.test_name = f"test-{self.test_number}"
        
        # Test klasörü oluştur
        self.test_dir = self.base_dir / self.test_name
        self.test_dir.mkdir(exist_ok=True)
        
        # Test başlangıç zamanı
        self.start_time = datetime.now()
        
        # Test verileri
        self.test_data = {
            'test_number': self.test_number,
            'test_name': self.test_name,
            'start_time': self.start_time.isoformat(),
            'end_time': None,
            'duration_seconds': None,
            'parameters': {},
            'statistics': {},
            'events': [],
            'errors': []
        }
        
        print(f"📊 Test Logger başlatıldı: {self.test_name}")
        print(f"   Klasör: {self.test_dir}")
    
    def _get_next_test_number(self) -> int:
        """Bir sonraki test numarasını belirle"""
        if not self.base_dir.exists():
            return 1
        
        # Mevcut test klasörlerini bul
        test_dirs = [d for d in self.base_dir.iterdir() 
                     if d.is_dir() and d.name.startswith('test-')]
        
        if not test_dirs:
            return 1
        
        # En yüksek numarayı bul
        numbers = []
        for d in test_dirs:
            try:
                num = int(d.name.split('-')[1])
                numbers.append(num)
            except (IndexError, ValueError):
                continue
        
        return max(numbers) + 1 if numbers else 1
    
    def log_parameters(self, params: Dict[str, Any]):
        """Test parametrelerini kaydet"""
        self.test_data['parameters'].update(params)
    
    def log_statistic(self, key: str, value: Any):
        """İstatistik kaydet"""
        self.test_data['statistics'][key] = value
    
    def log_event(self, event_type: str, message: str, data: Dict = None):
        """Olay kaydet"""
        event = {
            'timestamp': datetime.now().isoformat(),
            'type': event_type,
            'message': message,
            'data': data or {}
        }
        self.test_data['events'].append(event)
    
    def log_error(self, error_type: str, message: str, traceback: str = None):
        """Hata kaydet"""
        error = {
            'timestamp': datetime.now().isoformat(),
            'type': error_type,
            'message': message,
            'traceback': traceback
        }
        self.test_data['errors'].append(error)
    
    def log_frame_stats(self, frame_num: int, detections: int, tracking: bool, 
                       fps: float, confidence: float = None):
        """Frame istatistiklerini kaydet"""
        if 'frames' not in self.test_data['statistics']:
            self.test_data['statistics']['frames'] = []
        
        frame_stat = {
            'frame': frame_num,
            'detections': detections,
            'tracking': tracking,
            'fps': fps,
            'confidence': confidence
        }
        
        self.test_data['statistics']['frames'].append(frame_stat)
    
    def finalize(self):
        """Test sonuçlarını kaydet"""
        self.test_data['end_time'] = datetime.now().isoformat()
        
        # Süre hesapla
        end_time = datetime.now()
        duration = (end_time - self.start_time).total_seconds()
        self.test_data['duration_seconds'] = duration
        
        # JSON dosyasına kaydet
        json_path = self.test_dir / 'test_results.json'
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(self.test_data, f, indent=2, ensure_ascii=False)
        
        # Özet rapor oluştur
        self._create_summary_report()
        
        print(f"\n📊 Test sonuçları kaydedildi: {self.test_dir}")
        print(f"   Süre: {duration:.1f} saniye")
    
    def _create_summary_report(self):
        """Özet rapor oluştur"""
        report_path = self.test_dir / 'ÖZET.txt'
        
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("="*60 + "\n")
            f.write(f"TEST SONUÇLARI - {self.test_name.upper()}\n")
            f.write("="*60 + "\n\n")
            
            # Genel bilgiler
            f.write("GENEL BİLGİLER\n")
            f.write("-"*60 + "\n")
            f.write(f"Test No: {self.test_data['test_number']}\n")
            f.write(f"Başlangıç: {self.test_data['start_time']}\n")
            f.write(f"Bitiş: {self.test_data['end_time']}\n")
            f.write(f"Süre: {self.test_data['duration_seconds']:.1f} saniye\n\n")
            
            # Parametreler
            if self.test_data['parameters']:
                f.write("PARAMETRELER\n")
                f.write("-"*60 + "\n")
                for key, value in self.test_data['parameters'].items():
                    f.write(f"{key}: {value}\n")
                f.write("\n")
            
            # İstatistikler
            if self.test_data['statistics']:
                f.write("İSTATİSTİKLER\n")
                f.write("-"*60 + "\n")
                
                # Frame istatistikleri hariç diğerleri
                for key, value in self.test_data['statistics'].items():
                    if key != 'frames':
                        f.write(f"{key}: {value}\n")
                
                # Frame özeti
                if 'frames' in self.test_data['statistics']:
                    frames = self.test_data['statistics']['frames']
                    if frames:
                        total_frames = len(frames)
                        tracking_frames = sum(1 for f in frames if f['tracking'])
                        avg_fps = sum(f['fps'] for f in frames) / total_frames
                        
                        f.write(f"\nToplam Frame: {total_frames}\n")
                        f.write(f"Tracking Frame: {tracking_frames}\n")
                        f.write(f"Tracking Oranı: {tracking_frames/total_frames*100:.1f}%\n")
                        f.write(f"Ortalama FPS: {avg_fps:.1f}\n")
                
                f.write("\n")
            
            # Olaylar
            if self.test_data['events']:
                f.write("ÖNEMLI OLAYLAR\n")
                f.write("-"*60 + "\n")
                for event in self.test_data['events'][-10:]:  # Son 10 olay
                    f.write(f"[{event['timestamp']}] {event['type']}: {event['message']}\n")
                f.write("\n")
            
            # Hatalar
            if self.test_data['errors']:
                f.write("HATALAR\n")
                f.write("-"*60 + "\n")
                for error in self.test_data['errors']:
                    f.write(f"[{error['timestamp']}] {error['type']}: {error['message']}\n")
                f.write("\n")
            
            f.write("="*60 + "\n")
    
    def get_test_dir(self) -> Path:
        """Test klasörünü döndür"""
        return self.test_dir
    
    def get_test_name(self) -> str:
        """Test adını döndür"""
        return self.test_name


if __name__ == "__main__":
    # Test
    logger = TestLogger()
    
    logger.log_parameters({
        'source': 'screen',
        'model': 'yolov8n.pt',
        'tracker': 'CSRT',
        'use_kalman': True
    })
    
    logger.log_event('START', 'Test başlatıldı')
    
    # Örnek frame istatistikleri
    for i in range(100):
        logger.log_frame_stats(
            frame_num=i,
            detections=1 if i % 10 == 0 else 0,
            tracking=i % 10 != 0,
            fps=20.5,
            confidence=0.85 if i % 10 == 0 else None
        )
    
    logger.log_statistic('total_detections', 10)
    logger.log_statistic('avg_confidence', 0.85)
    
    logger.log_event('END', 'Test tamamlandı')
    
    logger.finalize()
    
    print(f"\nTest klasörü: {logger.get_test_dir()}")
