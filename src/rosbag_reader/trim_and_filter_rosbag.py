#!/usr/bin/env python3
"""
Script para filtrar tópicos Y recortar temporalmente un rosbag de ROS 2.
Soporta filtrado por tópicos y recorte temporal (inicio/fin/duración).
"""
import argparse
from pathlib import Path
from typing import Set, List, Optional

import rosbag2_py


def open_reader(input_uri: str) -> rosbag2_py.SequentialReader:
    """Abre un reader de rosbag2 (db3 o mcap)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_uri, storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", 
            output_serialization_format="cdr"
        ),
    )
    return reader


def get_topic_metadata(reader: rosbag2_py.SequentialReader) -> dict:
    """Obtiene metadata de todos los tópicos."""
    metadata = {}
    for topic in reader.get_all_topics_and_types():
        qos = getattr(topic, "offered_qos_profiles", "") or ""
        ser = getattr(topic, "serialization_format", "cdr") or "cdr"
        metadata[topic.name] = (topic.type, ser, qos)
    return metadata


def normalize_topic_names(topics: List[str]) -> Set[str]:
    """Normaliza nombres de tópicos."""
    normalized = set()
    for topic in topics:
        topic = topic.strip()
        if not topic:
            continue
        if not topic.startswith("/"):
            topic = "/" + topic
        normalized.add(topic)
    return normalized


def get_bag_time_info(reader: rosbag2_py.SequentialReader):
    """
    Obtiene información temporal del bag.
    Retorna: (start_time_ns, end_time_ns, duration_s)
    """
    start_time = None
    end_time = None
    count = 0
    
    while reader.has_next():
        _, _, timestamp = reader.read_next()
        if start_time is None:
            start_time = timestamp
        end_time = timestamp
        count += 1
    
    duration_s = (end_time - start_time) / 1e9 if start_time and end_time else 0
    return start_time, end_time, duration_s


def trim_and_filter_rosbag(
    input_path: str,
    output_path: str,
    topics_to_keep: Optional[List[str]] = None,
    start_offset_s: float = 0.0,
    end_offset_s: Optional[float] = None,
    duration_s: Optional[float] = None,
    output_format: str = "mcap"
):
    """
    Filtra tópicos y recorta temporalmente un rosbag.
    lol
    Args:
        input_path: Ruta al bag de entrada
        output_path: Ruta de salida
        topics_to_keep: Lista de tópicos a mantener (None = todos)
        start_offset_s: Segundos a omitir desde el inicio (default: 0)
        end_offset_s: Segundo donde terminar (desde el inicio del bag)
        duration_s: Duración en segundos a grabar (alternativa a end_offset_s)
        output_format: "db3" o "mcap"
    """
    input_uri = str(Path(input_path).resolve())
    output_uri = str(Path(output_path).resolve())
    
    print(f"\n{'='*70}")
    print("Filtrado y recorte de rosbag")
    print(f"{'='*70}")
    print(f"Input:  {input_uri}")
    print(f"Output: {output_uri}")
    print(f"Formato: {output_format}")
    
    # --- Paso 1: Analizar el bag ---
    print("\n[1/4] Analizando rosbag de entrada...")
    reader = open_reader(input_uri)
    topic_metadata = get_topic_metadata(reader)
    
    # Obtener info temporal
    print("      Escaneando timestamps...")
    start_time_ns, end_time_ns, total_duration_s = get_bag_time_info(reader)
    del reader  # Cerrar para reabrir
    
    print(f"      Duración total: {total_duration_s:.2f} segundos")
    print(f"      Start timestamp: {start_time_ns}")
    print(f"      End timestamp: {end_time_ns}")
    
    # Calcular ventana temporal
    trim_start_ns = start_time_ns + int(start_offset_s * 1e9)
    
    if duration_s is not None:
        trim_end_ns = trim_start_ns + int(duration_s * 1e9)
        actual_duration = duration_s
    elif end_offset_s is not None:
        trim_end_ns = start_time_ns + int(end_offset_s * 1e9)
        actual_duration = end_offset_s - start_offset_s
    else:
        trim_end_ns = end_time_ns
        actual_duration = total_duration_s - start_offset_s
    
    # Asegurar que no exceda el final del bag
    trim_end_ns = min(trim_end_ns, end_time_ns)
    
    print(f"\n📍 Ventana de tiempo:")
    print(f"   Inicio: +{start_offset_s:.2f}s desde el inicio del bag")
    print(f"   Duración: {actual_duration:.2f}s")
    print(f"   Fin: +{start_offset_s + actual_duration:.2f}s")
    
    # --- Paso 2: Preparar tópicos ---
    if topics_to_keep is None:
        valid_topics = set(topic_metadata.keys())
        print(f"\n✓ Manteniendo TODOS los tópicos ({len(valid_topics)})")
    else:
        keep_topics = normalize_topic_names(topics_to_keep)
        existing_topics = set(topic_metadata.keys())
        valid_topics = keep_topics.intersection(existing_topics)
        missing_topics = keep_topics - existing_topics
        
        if missing_topics:
            print(f"\n⚠️  Advertencia: {len(missing_topics)} tópicos no encontrados:")
            for topic in sorted(missing_topics):
                print(f"   ✗ {topic}")
        
        if not valid_topics:
            print("\n❌ Error: Ningún tópico válido para filtrar")
            return
        
        print(f"\n✓ Filtrando a {len(valid_topics)} tópicos:")
        for topic in sorted(valid_topics):
            print(f"   ✓ {topic}")
    
    # --- Paso 3: Crear writer ---
    print(f"\n[2/4] Creando rosbag de salida...")
    reader = open_reader(input_uri)
    
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_uri, storage_id=output_format),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        ),
    )
    
    # Crear tópicos
    for topic_name in valid_topics:
        msg_type, ser_format, qos = topic_metadata[topic_name]
        metadata = rosbag2_py.TopicMetadata(
            name=topic_name,
            type=msg_type,
            serialization_format=ser_format,
            offered_qos_profiles=qos,
        )
        writer.create_topic(metadata)
    
    # --- Paso 4: Copiar mensajes filtrados ---
    print("[3/4] Copiando mensajes...")
    total_read = 0
    total_written = 0
    skipped_time = 0
    skipped_topic = 0
    topic_counts = {t: 0 for t in valid_topics}
    
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        total_read += 1
        
        # Filtro temporal
        if timestamp < trim_start_ns:
            skipped_time += 1
            continue
        if timestamp > trim_end_ns:
            break
        
        # Filtro de tópicos
        if topic not in valid_topics:
            skipped_topic += 1
            continue
        
        writer.write(topic, data, timestamp)
        total_written += 1
        topic_counts[topic] += 1
        
        # Progreso
        if total_read % 10000 == 0:
            elapsed = (timestamp - start_time_ns) / 1e9
            print(f"   Procesados: {total_read:,} msgs | "
                  f"Tiempo: {elapsed:.1f}s | "
                  f"Escritos: {total_written:,}")
    
    # Limpiar
    del reader
    del writer
    
    # --- Resumen final ---
    print(f"\n{'='*70}")
    print("✅ Proceso completado exitosamente")
    print(f"{'='*70}")
    print(f"📊 Estadísticas:")
    print(f"   Mensajes leídos:           {total_read:,}")
    print(f"   Mensajes escritos:         {total_written:,}")
    print(f"   Omitidos (antes inicio):   {skipped_time:,}")
    print(f"   Omitidos (tópico filtrado): {skipped_topic:,}")
    
    if valid_topics != set(topic_metadata.keys()):
        print(f"\n📝 Tópicos en el nuevo bag:")
        for topic in sorted(topic_counts.keys()):
            count = topic_counts[topic]
            if count > 0:
                msg_type = topic_metadata[topic][0]
                print(f"   {topic}")
                print(f"      Tipo: {msg_type}")
                print(f"      Mensajes: {count:,}")


def main():
    parser = argparse.ArgumentParser(
        description="Filtra tópicos y recorta temporalmente un rosbag de ROS 2",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos de uso:

  # Recortar los primeros 30 segundos
  python3 trim_and_filter_rosbag.py input_bag output_bag --start 30

  # Recortar primeros 30s y mantener solo 2 minutos
  python3 trim_and_filter_rosbag.py input_bag output_bag --start 30 --duration 120

  # Mantener desde segundo 30 hasta segundo 150
  python3 trim_and_filter_rosbag.py input_bag output_bag --start 30 --end 150

  # Combinar filtro de tópicos y recorte temporal
  python3 trim_and_filter_rosbag.py input_bag output_bag \\
      --start 30 --duration 120 \\
      --topics /scan /imu \\
      --format mcap

  # Solo filtrar tópicos (sin recorte temporal)
  python3 trim_and_filter_rosbag.py input_bag output_bag --topics /scan /imu

  # Solo recorte temporal (mantener todos los tópicos)
  python3 trim_and_filter_rosbag.py input_bag output_bag --start 60 --end 300
        """
    )
    
    parser.add_argument("input", help="Ruta al rosbag de entrada")
    parser.add_argument("output", help="Ruta para el rosbag de salida")
    
    # Opciones de filtrado de tópicos
    topic_group = parser.add_argument_group("Filtrado de tópicos")
    topic_group.add_argument(
        "--topics", nargs="+", help="Tópicos a mantener (default: todos)"
    )
    topic_group.add_argument(
        "--topics-file", help="Archivo con lista de tópicos (uno por línea)"
    )
    
    # Opciones de recorte temporal
    time_group = parser.add_argument_group("Recorte temporal")
    time_group.add_argument(
        "--start", type=float, default=0.0,
        help="Segundos a omitir desde el inicio (default: 0)"
    )
    time_group.add_argument(
        "--end", type=float, default=None,
        help="Segundo donde terminar (desde inicio del bag)"
    )
    time_group.add_argument(
        "--duration", type=float, default=None,
        help="Duración en segundos a grabar (alternativa a --end)"
    )
    
    # Otras opciones
    parser.add_argument(
        "--format", choices=["db3", "mcap"], default="mcap",
        help="Formato del rosbag de salida (default: mcap)"
    )
    
    args = parser.parse_args()
    
    # Validaciones
    if args.end is not None and args.duration is not None:
        parser.error("No puedes usar --end y --duration al mismo tiempo")
    
    # Obtener lista de tópicos
    topics = None
    if args.topics or args.topics_file:
        topics = []
        if args.topics:
            topics.extend(args.topics)
        if args.topics_file:
            with open(args.topics_file, 'r') as f:
                topics.extend([line.strip() for line in f if line.strip()])
    
    # Ejecutar
    trim_and_filter_rosbag(
        args.input,
        args.output,
        topics_to_keep=topics,
        start_offset_s=args.start,
        end_offset_s=args.end,
        duration_s=args.duration,
        output_format=args.format
    )


if __name__ == "__main__":
    main()