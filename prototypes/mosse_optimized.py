"""
MOSSE Tracker Optimizasyon Profili
En hizli performans icin optimize edilmis ayarlar
"""

# MOSSE icin optimize edilmis konfigürasyon
MOSSE_OPTIMIZED_CONFIG = {
    # Model Ayarlari
    'model': 'yolov8n.pt',  # En kucuk ve en hizli model
    'conf_threshold': 0.4,   # Dusuk esik - daha fazla tespit
    'iou_threshold': 0.4,    # Dusuk IOU - daha agresif NMS
    'input_size': (416, 416),  # Daha kucuk input - daha hizli
    
    # Tracker Ayarlari
    'tracker_type': 'MOSSE',
    'max_bbox_change': 0.6,  # Daha esnek - hizli hareketler icin
    'min_bbox_area': 50,     # Daha kucuk minimum alan
    'max_consecutive_failures': 2,  # Daha hizli yeniden tespit
    
    # Kalman Filter Ayarlari
    'use_kalman': True,
    'kalman_dt': 1.0,
    'kalman_process_noise': 0.01,  # Daha yuksek - hizli degisimlere uyum
    'kalman_measurement_noise': 0.1,
    'prediction_steps': 3,  # Daha az tahmin - daha hizli
    
    # Performans Ayarlari
    'resize_input': True,   # Resize aktif - daha hizli
    'skip_frames': 0,       # Her frame'i isle
    'use_gpu': False,       # CPU optimizasyonu
    
    # Gorsellestime Ayarlari
    'show_trajectory': True,
    'trajectory_length': 20,  # Daha kisa iz - daha hizli
    'show_predictions': True,
    'show_info': True,
    
    # Video Kayit
    'save_video': True,
    'video_fps': 30,
    'video_codec': 'mp4v',
}


def apply_mosse_optimization():
    """MOSSE optimizasyonunu uygula"""
    import config
    
    # Model ayarlari
    config.MODEL_CONFIG['conf_threshold'] = MOSSE_OPTIMIZED_CONFIG['conf_threshold']
    config.MODEL_CONFIG['iou_threshold'] = MOSSE_OPTIMIZED_CONFIG['iou_threshold']
    
    # Tracker ayarlari
    config.TRACKER_CONFIG['tracker_type'] = 'MOSSE'
    config.TRACKER_CONFIG['reinit_threshold'] = 2
    
    # Kalman ayarlari
    config.KALMAN_CONFIG['process_noise'] = MOSSE_OPTIMIZED_CONFIG['kalman_process_noise']
    config.KALMAN_CONFIG['measurement_noise'] = MOSSE_OPTIMIZED_CONFIG['kalman_measurement_noise']
    config.KALMAN_CONFIG['prediction_steps'] = MOSSE_OPTIMIZED_CONFIG['prediction_steps']
    
    # Performans ayarlari
    config.PERFORMANCE_CONFIG['resize_input'] = True
    config.PERFORMANCE_CONFIG['input_size'] = MOSSE_OPTIMIZED_CONFIG['input_size']
    
    # Gorsellestime
    config.VISUALIZATION_CONFIG['trajectory_length'] = MOSSE_OPTIMIZED_CONFIG['trajectory_length']
    
    print("MOSSE optimizasyonu uygulandi!")
    print(f"  Input size: {MOSSE_OPTIMIZED_CONFIG['input_size']}")
    print(f"  Conf threshold: {MOSSE_OPTIMIZED_CONFIG['conf_threshold']}")
    print(f"  Trajectory length: {MOSSE_OPTIMIZED_CONFIG['trajectory_length']}")


if __name__ == "__main__":
    apply_mosse_optimization()
