"""
Kalman Filter Modülü
Nesne konumu ve hızı tahmini için
"""

import numpy as np
from filterpy.kalman import KalmanFilter
from typing import Tuple, Optional, List


class UAVKalmanFilter:
    """İHA takibi için Kalman Filter implementasyonu"""
    
    def __init__(self, 
                 dt: float = 1.0,
                 process_noise: float = 1e-3,
                 measurement_noise: float = 1e-1):
        """
        Args:
            dt: Zaman adımı (frame arası süre)
            process_noise: Süreç gürültüsü (model belirsizliği)
            measurement_noise: Ölçüm gürültüsü (tespit belirsizliği)
        """
        # Kalman filter oluştur
        # State: [x, y, vx, vy] - konum ve hız
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        
        # State transition matrix (F)
        # x_new = x + vx*dt
        # y_new = y + vy*dt
        # vx_new = vx
        # vy_new = vy
        self.kf.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Measurement matrix (H)
        # Sadece konum ölçüyoruz (x, y)
        self.kf.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Process noise covariance (Q)
        self.kf.Q = np.eye(4) * process_noise
        
        # Measurement noise covariance (R)
        self.kf.R = np.eye(2) * measurement_noise
        
        # Initial state covariance (P)
        self.kf.P = np.eye(4) * 1000
        
        self.dt = dt
        self.initialized = False
        self.prediction_history = []
        
        print(f"Kalman Filter oluşturuldu (dt={dt}, Q={process_noise}, R={measurement_noise})")
    
    def init(self, x: float, y: float, vx: float = 0, vy: float = 0):
        """
        Filter'ı başlat
        
        Args:
            x: Başlangıç x pozisyonu
            y: Başlangıç y pozisyonu
            vx: Başlangıç x hızı (opsiyonel)
            vy: Başlangıç y hızı (opsiyonel)
        """
        self.kf.x = np.array([x, y, vx, vy])
        self.initialized = True
        self.prediction_history.clear()
        print(f"Kalman Filter başlatıldı: pos=({x:.1f}, {y:.1f}), vel=({vx:.1f}, {vy:.1f})")
    
    def predict(self) -> Tuple[float, float]:
        """
        Bir sonraki state'i tahmin et
        
        Returns:
            (x, y) tahmini
        """
        if not self.initialized:
            raise RuntimeError("Kalman filter başlatılmamış!")
        
        self.kf.predict()
        
        x, y = self.kf.x[0], self.kf.x[1]
        return (x, y)
    
    def update(self, measurement: Tuple[float, float]) -> Tuple[float, float]:
        """
        Ölçüm ile güncelle
        
        Args:
            measurement: (x, y) ölçümü
            
        Returns:
            (x, y) güncellenmiş tahmin
        """
        if not self.initialized:
            # İlk ölçümle başlat
            self.init(measurement[0], measurement[1])
            return measurement
        
        # Önce tahmin
        self.predict()
        
        # Sonra güncelle
        z = np.array(measurement)
        self.kf.update(z)
        
        x, y = self.kf.x[0], self.kf.x[1]
        return (x, y)
    
    def predict_ahead(self, steps: int = 5) -> List[Tuple[float, float]]:
        """
        N adım ilerisi için tahmin yap
        
        Args:
            steps: Kaç adım ilerisi
            
        Returns:
            [(x, y), ...] tahmin listesi
        """
        if not self.initialized:
            return []
        
        # Mevcut state'i kaydet
        saved_x = self.kf.x.copy()
        saved_P = self.kf.P.copy()
        
        predictions = []
        
        for _ in range(steps):
            self.kf.predict()
            x, y = self.kf.x[0], self.kf.x[1]
            predictions.append((x, y))
        
        # State'i geri yükle
        self.kf.x = saved_x
        self.kf.P = saved_P
        
        return predictions
    
    def get_state(self) -> Tuple[float, float, float, float]:
        """
        Mevcut state'i döndür
        
        Returns:
            (x, y, vx, vy)
        """
        if not self.initialized:
            return (0, 0, 0, 0)
        
        return tuple(self.kf.x)
    
    def get_position(self) -> Tuple[float, float]:
        """
        Mevcut pozisyonu döndür
        
        Returns:
            (x, y)
        """
        if not self.initialized:
            return (0, 0)
        
        return (self.kf.x[0], self.kf.x[1])
    
    def get_velocity(self) -> Tuple[float, float]:
        """
        Mevcut hızı döndür
        
        Returns:
            (vx, vy)
        """
        if not self.initialized:
            return (0, 0)
        
        return (self.kf.x[2], self.kf.x[3])
    
    def get_speed(self) -> float:
        """
        Hız büyüklüğünü döndür
        
        Returns:
            Hız (piksel/frame)
        """
        vx, vy = self.get_velocity()
        return np.sqrt(vx**2 + vy**2)
    
    def reset(self):
        """Filter'ı sıfırla"""
        self.kf.P = np.eye(4) * 1000
        self.initialized = False
        self.prediction_history.clear()
        print("Kalman Filter sıfırlandı")


class AdaptiveKalmanFilter(UAVKalmanFilter):
    """Adaptif Kalman Filter - gürültü parametrelerini otomatik ayarlar"""
    
    def __init__(self, dt: float = 1.0):
        super().__init__(dt)
        
        # İnovasyon (innovation) geçmişi
        self.innovation_history = []
        self.max_history = 10
    
    def update(self, measurement: Tuple[float, float]) -> Tuple[float, float]:
        """
        Ölçüm ile güncelle ve gürültü parametrelerini adapte et
        
        Args:
            measurement: (x, y) ölçümü
            
        Returns:
            (x, y) güncellenmiş tahmin
        """
        if not self.initialized:
            self.init(measurement[0], measurement[1])
            return measurement
        
        # Tahmin
        self.predict()
        
        # İnovasyon (ölçüm - tahmin)
        z = np.array(measurement)
        innovation = z - self.kf.H @ self.kf.x
        
        # İnovasyon geçmişi
        self.innovation_history.append(innovation)
        if len(self.innovation_history) > self.max_history:
            self.innovation_history.pop(0)
        
        # Ölçüm gürültüsünü adapte et
        if len(self.innovation_history) >= 3:
            innovations = np.array(self.innovation_history)
            innovation_cov = np.cov(innovations.T)
            
            # R'yi güncelle (yumuşak geçiş)
            alpha = 0.1
            self.kf.R = (1 - alpha) * self.kf.R + alpha * innovation_cov
        
        # Güncelle
        self.kf.update(z)
        
        x, y = self.kf.x[0], self.kf.x[1]
        return (x, y)


if __name__ == "__main__":
    # Test
    import matplotlib.pyplot as plt
    
    print("=== Kalman Filter Testi ===\n")
    
    # Simüle edilmiş hareket (dairesel)
    dt = 1.0
    num_steps = 100
    radius = 100
    center = (200, 200)
    
    # Gerçek konum
    true_positions = []
    for i in range(num_steps):
        angle = 2 * np.pi * i / num_steps
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        true_positions.append((x, y))
    
    # Gürültülü ölçümler
    noise_std = 10
    measurements = []
    for x, y in true_positions:
        mx = x + np.random.normal(0, noise_std)
        my = y + np.random.normal(0, noise_std)
        measurements.append((mx, my))
    
    # Kalman filter
    kf = UAVKalmanFilter(dt=dt, process_noise=1e-3, measurement_noise=noise_std**2)
    
    # Filtreleme
    filtered_positions = []
    predictions = []
    
    for i, measurement in enumerate(measurements):
        # Güncelle
        filtered_pos = kf.update(measurement)
        filtered_positions.append(filtered_pos)
        
        # 5 adım ilerisi tahmin
        if i % 10 == 0:
            pred = kf.predict_ahead(5)
            predictions.append((i, pred))
    
    # Görselleştir
    true_x, true_y = zip(*true_positions)
    meas_x, meas_y = zip(*measurements)
    filt_x, filt_y = zip(*filtered_positions)
    
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 1, 1)
    plt.plot(true_x, true_y, 'g-', label='Gerçek', linewidth=2)
    plt.plot(meas_x, meas_y, 'r.', label='Ölçüm', alpha=0.5)
    plt.plot(filt_x, filt_y, 'b-', label='Kalman Filter', linewidth=2)
    
    # Tahminleri göster
    for i, pred in predictions:
        pred_x, pred_y = zip(*pred)
        plt.plot([filt_x[i]] + list(pred_x), [filt_y[i]] + list(pred_y), 
                'c--', alpha=0.5, linewidth=1)
    
    plt.legend()
    plt.title('Kalman Filter - Konum Tahmini')
    plt.xlabel('X (piksel)')
    plt.ylabel('Y (piksel)')
    plt.grid(True)
    plt.axis('equal')
    
    # Hata analizi
    plt.subplot(2, 1, 2)
    
    errors_meas = [np.sqrt((m[0]-t[0])**2 + (m[1]-t[1])**2) 
                   for m, t in zip(measurements, true_positions)]
    errors_filt = [np.sqrt((f[0]-t[0])**2 + (f[1]-t[1])**2) 
                   for f, t in zip(filtered_positions, true_positions)]
    
    plt.plot(errors_meas, 'r-', label='Ölçüm Hatası', alpha=0.7)
    plt.plot(errors_filt, 'b-', label='Filter Hatası', linewidth=2)
    plt.legend()
    plt.title('Hata Analizi')
    plt.xlabel('Frame')
    plt.ylabel('Hata (piksel)')
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('kalman_test.png', dpi=150)
    print("Test grafiği kaydedildi: kalman_test.png")
    
    # İstatistikler
    print(f"\nOrtalama Ölçüm Hatası: {np.mean(errors_meas):.2f} piksel")
    print(f"Ortalama Filter Hatası: {np.mean(errors_filt):.2f} piksel")
    print(f"İyileştirme: {(1 - np.mean(errors_filt)/np.mean(errors_meas))*100:.1f}%")
