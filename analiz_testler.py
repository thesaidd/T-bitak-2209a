"""
Test Sonuçları Analiz Aracı
Testler klasöründeki tüm test sonuçlarını karşılaştırır
"""

import json
import os
from pathlib import Path
from typing import List, Dict
import sys


def load_test_results(test_dir: Path) -> Dict:
    """Test sonuçlarını yükle"""
    json_path = test_dir / 'test_results.json'
    
    if not json_path.exists():
        return None
    
    with open(json_path, 'r', encoding='utf-8') as f:
        return json.load(f)


def analyze_test(test_data: Dict) -> Dict:
    """Test verilerini analiz et"""
    if not test_data:
        return None
    
    stats = test_data.get('statistics', {})
    params = test_data.get('parameters', {})
    
    # Frame istatistikleri
    frames = stats.get('frames', [])
    
    if not frames:
        return {
            'test_name': test_data.get('test_name', 'Unknown'),
            'params': params,
            'total_frames': 0,
            'tracking_frames': 0,
            'tracking_rate': 0,
            'avg_fps': 0,
            'duration': test_data.get('duration_seconds', 0)
        }
    
    total_frames = len(frames)
    tracking_frames = sum(1 for f in frames if f.get('tracking', False))
    tracking_rate = (tracking_frames / total_frames * 100) if total_frames > 0 else 0
    
    fps_values = [f['fps'] for f in frames if f.get('fps', 0) > 0]
    avg_fps = sum(fps_values) / len(fps_values) if fps_values else 0
    
    # Güven skorları
    confidence_values = [f['confidence'] for f in frames if f.get('confidence') is not None]
    avg_confidence = sum(confidence_values) / len(confidence_values) if confidence_values else 0
    
    return {
        'test_name': test_data.get('test_name', 'Unknown'),
        'params': params,
        'total_frames': total_frames,
        'tracking_frames': tracking_frames,
        'tracking_rate': tracking_rate,
        'avg_fps': avg_fps,
        'avg_confidence': avg_confidence,
        'duration': test_data.get('duration_seconds', 0),
        'total_detections': stats.get('total_detections', 0)
    }


def compare_tests(testler_dir: str = 'testler'):
    """Tüm testleri karşılaştır"""
    testler_path = Path(testler_dir)
    
    if not testler_path.exists():
        print(f"❌ {testler_dir} klasörü bulunamadı!")
        return
    
    # Tüm test klasörlerini bul
    test_dirs = sorted([d for d in testler_path.iterdir() if d.is_dir() and d.name.startswith('test-')])
    
    if not test_dirs:
        print(f"❌ {testler_dir} klasöründe test bulunamadı!")
        return
    
    print("="*80)
    print("TEST SONUÇLARI KARŞILAŞTIRMASI")
    print("="*80)
    print()
    
    # Tüm testleri analiz et
    results = []
    for test_dir in test_dirs:
        test_data = load_test_results(test_dir)
        if test_data:
            analysis = analyze_test(test_data)
            if analysis:
                results.append(analysis)
    
    if not results:
        print("❌ Analiz edilebilir test bulunamadı!")
        return
    
    # Özet tablo
    print(f"{'Test':<10} {'Tracker':<8} {'Kalman':<8} {'Frames':<8} {'Track%':<8} {'FPS':<8} {'Conf':<8} {'Süre':<8}")
    print("-"*80)
    
    for r in results:
        params = r['params']
        tracker = params.get('tracker', 'N/A')
        kalman = '✓' if params.get('use_kalman', False) else '✗'
        
        print(f"{r['test_name']:<10} {tracker:<8} {kalman:<8} "
              f"{r['total_frames']:<8} {r['tracking_rate']:<7.1f}% "
              f"{r['avg_fps']:<7.1f} {r['avg_confidence']:<7.2f} "
              f"{r['duration']:<7.1f}s")
    
    print()
    print("="*80)
    
    # En iyi performanslar
    if len(results) > 1:
        print("\n📊 EN İYİ PERFORMANSLAR")
        print("-"*80)
        
        # En yüksek tracking oranı
        best_tracking = max(results, key=lambda x: x['tracking_rate'])
        print(f"🏆 En Yüksek Tracking Oranı: {best_tracking['test_name']} - %{best_tracking['tracking_rate']:.1f}")
        print(f"   Tracker: {best_tracking['params'].get('tracker', 'N/A')}, "
              f"Kalman: {'✓' if best_tracking['params'].get('use_kalman') else '✗'}")
        
        # En yüksek FPS
        best_fps = max(results, key=lambda x: x['avg_fps'])
        print(f"\n⚡ En Yüksek FPS: {best_fps['test_name']} - {best_fps['avg_fps']:.1f} FPS")
        print(f"   Tracker: {best_fps['params'].get('tracker', 'N/A')}, "
              f"Kalman: {'✓' if best_fps['params'].get('use_kalman') else '✗'}")
        
        # En yüksek güven
        if any(r['avg_confidence'] > 0 for r in results):
            best_conf = max((r for r in results if r['avg_confidence'] > 0), 
                          key=lambda x: x['avg_confidence'])
            print(f"\n🎯 En Yüksek Güven Skoru: {best_conf['test_name']} - {best_conf['avg_confidence']:.2f}")
            print(f"   Tracker: {best_conf['params'].get('tracker', 'N/A')}, "
                  f"Kalman: {'✓' if best_conf['params'].get('use_kalman') else '✗'}")
        
        print()
        print("="*80)
    
    # Tracker karşılaştırması
    tracker_stats = {}
    for r in results:
        tracker = r['params'].get('tracker', 'Unknown')
        if tracker not in tracker_stats:
            tracker_stats[tracker] = []
        tracker_stats[tracker].append(r)
    
    if len(tracker_stats) > 1:
        print("\n📈 TRACKER KARŞILAŞTIRMASI")
        print("-"*80)
        print(f"{'Tracker':<10} {'Testler':<8} {'Ort. Track%':<12} {'Ort. FPS':<10}")
        print("-"*80)
        
        for tracker, tests in sorted(tracker_stats.items()):
            avg_track_rate = sum(t['tracking_rate'] for t in tests) / len(tests)
            avg_fps = sum(t['avg_fps'] for t in tests) / len(tests)
            
            print(f"{tracker:<10} {len(tests):<8} {avg_track_rate:<11.1f}% {avg_fps:<9.1f}")
        
        print()
        print("="*80)
    
    # Kalman etkisi
    kalman_yes = [r for r in results if r['params'].get('use_kalman', False)]
    kalman_no = [r for r in results if not r['params'].get('use_kalman', False)]
    
    if kalman_yes and kalman_no:
        print("\n🔬 KALMAN FILTER ETKİSİ")
        print("-"*80)
        
        avg_track_yes = sum(r['tracking_rate'] for r in kalman_yes) / len(kalman_yes)
        avg_track_no = sum(r['tracking_rate'] for r in kalman_no) / len(kalman_no)
        
        avg_fps_yes = sum(r['avg_fps'] for r in kalman_yes) / len(kalman_yes)
        avg_fps_no = sum(r['avg_fps'] for r in kalman_no) / len(kalman_no)
        
        print(f"Kalman İLE    : Track Rate = %{avg_track_yes:.1f}, FPS = {avg_fps_yes:.1f}")
        print(f"Kalman OLMADAN: Track Rate = %{avg_track_no:.1f}, FPS = {avg_fps_no:.1f}")
        print(f"\nFark: Track Rate = %{avg_track_yes - avg_track_no:+.1f}, FPS = {avg_fps_yes - avg_fps_no:+.1f}")
        
        print()
        print("="*80)


def show_test_details(test_name: str, testler_dir: str = 'testler'):
    """Belirli bir testin detaylarını göster"""
    test_path = Path(testler_dir) / test_name
    
    if not test_path.exists():
        print(f"❌ {test_name} bulunamadı!")
        return
    
    test_data = load_test_results(test_path)
    if not test_data:
        print(f"❌ {test_name} için veri yüklenemedi!")
        return
    
    analysis = analyze_test(test_data)
    
    print("="*80)
    print(f"TEST DETAYLARI - {test_name.upper()}")
    print("="*80)
    print()
    
    print("PARAMETRELER:")
    for key, value in analysis['params'].items():
        print(f"  {key}: {value}")
    
    print()
    print("İSTATİSTİKLER:")
    print(f"  Toplam Frame: {analysis['total_frames']}")
    print(f"  Tracking Frame: {analysis['tracking_frames']}")
    print(f"  Tracking Oranı: %{analysis['tracking_rate']:.1f}")
    print(f"  Ortalama FPS: {analysis['avg_fps']:.1f}")
    print(f"  Ortalama Güven: {analysis['avg_confidence']:.2f}")
    print(f"  Toplam Tespit: {analysis['total_detections']}")
    print(f"  Süre: {analysis['duration']:.1f} saniye")
    
    print()
    print("="*80)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        # Belirli bir test göster
        show_test_details(sys.argv[1])
    else:
        # Tüm testleri karşılaştır
        compare_tests()
