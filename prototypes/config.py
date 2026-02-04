"""
Konfigurasyon Dosyasi
IHA tespit ve takip sistemi icin tum ayarlar
"""

# Model Ayarlari
MODEL_CONFIG = {
    'default_model': 'yolov8n.pt',  # Varsayilan model
    'conf_threshold': 0.5,     # Tespit guven esigi
    'iou_threshold': 0.45,           # NMS IoU esigi
    'max_det': 10,                   # Maksimum tespit sayisi
    'device': 'cpu',                 # 'cpu' veya 'cuda'
    
    # COCO dataset siniflari (IHA ile ilgili)
    'target_classes': {
        4: 'airplane',    # Ucak
        14: 'bird',       # Kus (IHA benzeri)
        # Ozel model icin: 0: 'drone', 1: 'uav' eklenebilir
    }
}

# Tracker Ayarlari
TRACKER_CONFIG = {
    'tracker_type': 'CSRT',  # CSRT, KCF, MOSSE, MIL
    'reinit_threshold': 2,    # Kac frame sonra yeniden tespit (azaltildi: 5->2)
    'min_confidence': 0.7,    # Tracker icin minimum guven (artirildi: 0.6->0.7)
}

# Kalman Filter Ayarlari
KALMAN_CONFIG = {
    'use_kalman': True,
    'process_noise': 1e-3,    # Surec gurultusu
    'measurement_noise': 1e-1, # Olcum gurultusu
    'prediction_steps': 5,     # Kac frame ilerisi tahmin
}

# Ekran Yakalama Ayarlari
SCREEN_CONFIG = {
    'monitor': 1,              # Hangi monitor (1, 2, ...)
    'region': None,            # None = tam ekran, veya {'top': 100, 'left': 100, 'width': 800, 'height': 600}
    'fps': 30,                 # Hedef FPS
}

# Video Kayit Ayarlari
RECORDING_CONFIG = {
    'save_video': True,
    'output_dir': 'results',
    'video_codec': 'mp4v',     # 'mp4v', 'XVID'
    'video_fps': 20,
}

# Gorsellestirme Ayarlari
VISUALIZATION_CONFIG = {
    'show_fps': True,
    'show_confidence': True,
    'show_trajectory': True,
    'trajectory_length': 30,   # Kac frame iz goster
    'bbox_thickness': 2,
    'text_size': 0.6,
    
    # Renkler (BGR)
    'bbox_color': (0, 255, 0),      # Yesil
    'trajectory_color': (255, 0, 0), # Mavi
    'prediction_color': (0, 0, 255), # Kirmizi
    'text_color': (255, 255, 255),   # Beyaz
}

# Performans Ayarlari
PERFORMANCE_CONFIG = {
    'resize_input': False,     # Tam cozunurluk icin False (1920x1080)
    'input_size': (640, 480),  # YOLOv8 icin optimize boyut (resize_input=True ise)
    'skip_frames': 0,          # Her N frame'de bir isle (0 = hepsini isle)
}

# Log Ayarlari
LOG_CONFIG = {
    'verbose': True,
    'save_logs': True,
    'log_file': 'results/detection_log.txt',
}


# ============================================================
# PERFORMANS PROFILLERI
# ============================================================

# MOSSE Hizli Mod - Maksimum hiz icin optimize edilmis
MOSSE_FAST_PROFILE = {
    'model_conf': 0.4,
    'model_iou': 0.4,
    'tracker_type': 'MOSSE',
    'kalman_process_noise': 0.01,
    'kalman_measurement_noise': 0.1,
    'prediction_steps': 3,
    'resize_input': True,
    'input_size': (416, 416),
    'trajectory_length': 20,
    'max_bbox_change': 0.6,
    'min_bbox_area': 50,
}

# CSRT Kalite Mod - Maksimum dogruluk icin optimize edilmis
CSRT_QUALITY_PROFILE = {
    'model_conf': 0.5,
    'model_iou': 0.5,
    'tracker_type': 'CSRT',
    'kalman_process_noise': 0.001,
    'kalman_measurement_noise': 0.1,
    'prediction_steps': 5,
    'resize_input': False,
    'input_size': (640, 480),
    'trajectory_length': 30,
    'max_bbox_change': 0.3,
    'min_bbox_area': 100,
}

# KCF Dengeli Mod - Hiz ve dogruluk dengesi
KCF_BALANCED_PROFILE = {
    'model_conf': 0.5,
    'model_iou': 0.45,
    'tracker_type': 'KCF',
    'kalman_process_noise': 0.005,
    'kalman_measurement_noise': 0.1,
    'prediction_steps': 4,
    'resize_input': True,
    'input_size': (512, 384),
    'trajectory_length': 25,
    'max_bbox_change': 0.4,
    'min_bbox_area': 75,
}


def apply_profile(profile_name='mosse_fast'):
    """
    Performans profilini uygula
    
    Args:
        profile_name: 'mosse_fast', 'csrt_quality', veya 'kfc_balanced'
    """
    profiles = {
        'mosse_fast': MOSSE_FAST_PROFILE,
        'csrt_quality': CSRT_QUALITY_PROFILE,
        'kfc_balanced': KCF_BALANCED_PROFILE,
    }
    
    if profile_name not in profiles:
        print(f"Gecersiz profil: {profile_name}")
        return
    
    profile = profiles[profile_name]
    
    # Ayarlari uygula
    MODEL_CONFIG['conf_threshold'] = profile['model_conf']
    MODEL_CONFIG['iou_threshold'] = profile['model_iou']
    
    TRACKER_CONFIG['tracker_type'] = profile['tracker_type']
    
    KALMAN_CONFIG['process_noise'] = profile['kalman_process_noise']
    KALMAN_CONFIG['measurement_noise'] = profile['kalman_measurement_noise']
    KALMAN_CONFIG['prediction_steps'] = profile['prediction_steps']
    
    PERFORMANCE_CONFIG['resize_input'] = profile['resize_input']
    PERFORMANCE_CONFIG['input_size'] = profile['input_size']
    
    VISUALIZATION_CONFIG['trajectory_length'] = profile['trajectory_length']
    
    print(f"Profil uygulandi: {profile_name}")
    print(f"  Tracker: {profile['tracker_type']}")
    print(f"  Input size: {profile['input_size']}")
    print(f"  Resize: {profile['resize_input']}")
