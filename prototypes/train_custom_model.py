"""
Özel İHA Dataset Eğitim Scripti
YOLOv8 modelini özel dataset ile eğitmek için
"""

from ultralytics import YOLO
import yaml
from pathlib import Path
import argparse


def create_sample_dataset_yaml():
    """Örnek dataset.yaml dosyası oluştur"""
    
    dataset_config = {
        'path': './dataset',
        'train': 'images/train',
        'val': 'images/val',
        'test': 'images/test',
        'nc': 4,
        'names': ['drone', 'quadcopter', 'fixed_wing_uav', 'helicopter']
    }
    
    output_path = Path('dataset/data.yaml')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w') as f:
        yaml.dump(dataset_config, f, default_flow_style=False)
    
    print(f"Örnek dataset.yaml oluşturuldu: {output_path}")
    print("\nDataset yapısı:")
    print("dataset/")
    print("├── images/")
    print("│   ├── train/")
    print("│   ├── val/")
    print("│   └── test/")
    print("├── labels/")
    print("│   ├── train/")
    print("│   ├── val/")
    print("│   └── test/")
    print("└── data.yaml")


def train_model(args):
    """
    YOLOv8 modelini eğit
    
    Args:
        args: Komut satırı argümanları
    """
    print("="*60)
    print("YOLOV8 ÖZEL MODEL EĞİTİMİ")
    print("="*60)
    
    # Model yükle
    print(f"\n1. Model yükleniyor: {args.model}")
    model = YOLO(args.model)
    
    # Dataset kontrolü
    data_yaml = Path(args.data)
    if not data_yaml.exists():
        print(f"\nUYARI: {args.data} bulunamadı!")
        print("Örnek dataset.yaml oluşturuluyor...")
        create_sample_dataset_yaml()
        print("\nLütfen dataset'inizi hazırlayın ve tekrar deneyin.")
        return
    
    # Eğitim parametreleri
    print("\n2. Eğitim parametreleri:")
    print(f"   - Dataset: {args.data}")
    print(f"   - Epochs: {args.epochs}")
    print(f"   - Batch size: {args.batch}")
    print(f"   - Image size: {args.imgsz}")
    print(f"   - Device: {args.device}")
    print(f"   - Workers: {args.workers}")
    
    # Eğitim başlat
    print("\n3. Eğitim başlatılıyor...")
    print("   (Bu işlem uzun sürebilir, lütfen bekleyin...)\n")
    
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        patience=args.patience,
        save=True,
        project='runs/train',
        name=args.name,
        exist_ok=True,
        
        # Augmentation
        augment=True,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=10,
        translate=0.1,
        scale=0.5,
        flipud=0.0,
        fliplr=0.5,
        mosaic=1.0,
        
        # Diğer
        pretrained=True,
        optimizer='Adam',
        verbose=True,
        seed=42,
        deterministic=True,
    )
    
    # Sonuçlar
    print("\n" + "="*60)
    print("EĞİTİM TAMAMLANDI!")
    print("="*60)
    
    output_dir = Path(f'runs/train/{args.name}')
    print(f"\nÇıktı dizini: {output_dir}")
    print(f"En iyi model: {output_dir}/weights/best.pt")
    print(f"Son model: {output_dir}/weights/last.pt")
    
    # Metrikleri yazdır
    if hasattr(results, 'results_dict'):
        metrics = results.results_dict
        print("\nEğitim Metrikleri:")
        for key, value in metrics.items():
            print(f"  {key}: {value}")
    
    # Validasyon
    print("\n4. Validasyon yapılıyor...")
    val_results = model.val()
    
    print(f"\nValidasyon Sonuçları:")
    print(f"  mAP50: {val_results.box.map50:.4f}")
    print(f"  mAP50-95: {val_results.box.map:.4f}")
    print(f"  Precision: {val_results.box.mp:.4f}")
    print(f"  Recall: {val_results.box.mr:.4f}")
    
    # Kullanım talimatı
    print("\n" + "="*60)
    print("KULLANIM")
    print("="*60)
    print(f"\nEğitilmiş modeli kullanmak için:")
    print(f"python src/main.py --source screen --model {output_dir}/weights/best.pt")
    
    # Export talimatı
    print("\nRaspberry Pi için optimize etmek için:")
    print(f"python export_model.py --model {output_dir}/weights/best.pt --format tflite")


def export_model(args):
    """
    Modeli farklı formatlara export et
    
    Args:
        args: Komut satırı argümanları
    """
    print("="*60)
    print("MODEL EXPORT")
    print("="*60)
    
    print(f"\nModel: {args.model}")
    print(f"Format: {args.format}")
    
    model = YOLO(args.model)
    
    # Export
    print("\nExport ediliyor...")
    
    export_args = {'format': args.format}
    
    # TFLite için quantization
    if args.format == 'tflite' and args.int8:
        export_args['int8'] = True
        if args.data:
            export_args['data'] = args.data
    
    model.export(**export_args)
    
    print(f"\n✓ Model export edildi!")


def parse_args():
    """Komut satırı argümanları"""
    parser = argparse.ArgumentParser(description='YOLOv8 Özel Model Eğitimi')
    
    subparsers = parser.add_subparsers(dest='command', help='Komutlar')
    
    # Train komutu
    train_parser = subparsers.add_parser('train', help='Model eğit')
    train_parser.add_argument('--model', type=str, default='yolov8n.pt',
                            help='Başlangıç modeli')
    train_parser.add_argument('--data', type=str, default='dataset/data.yaml',
                            help='Dataset YAML dosyası')
    train_parser.add_argument('--epochs', type=int, default=100,
                            help='Epoch sayısı')
    train_parser.add_argument('--batch', type=int, default=16,
                            help='Batch size')
    train_parser.add_argument('--imgsz', type=int, default=640,
                            help='Görüntü boyutu')
    train_parser.add_argument('--device', type=str, default='cpu',
                            help='Device (cpu veya cuda)')
    train_parser.add_argument('--workers', type=int, default=4,
                            help='Worker sayısı')
    train_parser.add_argument('--patience', type=int, default=20,
                            help='Early stopping patience')
    train_parser.add_argument('--name', type=str, default='uav_detector',
                            help='Deney adı')
    
    # Export komutu
    export_parser = subparsers.add_parser('export', help='Model export et')
    export_parser.add_argument('--model', type=str, required=True,
                             help='Export edilecek model')
    export_parser.add_argument('--format', type=str, default='tflite',
                             choices=['tflite', 'onnx', 'torchscript', 'coreml'],
                             help='Export formatı')
    export_parser.add_argument('--int8', action='store_true',
                             help='INT8 quantization (TFLite için)')
    export_parser.add_argument('--data', type=str,
                             help='Kalibasyon için dataset YAML')
    
    # Setup komutu
    setup_parser = subparsers.add_parser('setup', help='Dataset yapısı oluştur')
    
    return parser.parse_args()


def main():
    """Ana fonksiyon"""
    args = parse_args()
    
    if args.command == 'train':
        train_model(args)
    elif args.command == 'export':
        export_model(args)
    elif args.command == 'setup':
        create_sample_dataset_yaml()
    else:
        print("Kullanım:")
        print("  python train_custom_model.py train --data dataset/data.yaml")
        print("  python train_custom_model.py export --model runs/train/uav_detector/weights/best.pt")
        print("  python train_custom_model.py setup")


if __name__ == "__main__":
    main()
