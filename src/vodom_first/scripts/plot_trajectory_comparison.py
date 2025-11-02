#!/usr/bin/env python3
"""
Script para comparar trayectorias VO vs Ground Truth
Calcula métricas ATE y RPE y genera gráficas
Compatible con formato TUM (usado por evo)
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import argparse
import os

def load_trajectory(filename):
    """
    Carga trayectoria en formato TUM
    Formato: timestamp tx ty tz qx qy qz qw
    """
    data = []
    line_count = 0
    valid_lines = 0
    
    with open(filename, 'r') as f:
        for line in f:
            line_count += 1
            if line.startswith('#'):
                continue
            values = line.strip().split()
            if len(values) >= 8:
                timestamp = float(values[0])
                tx, ty, tz = float(values[1]), float(values[2]), float(values[3])
                qx, qy, qz, qw = float(values[4]), float(values[5]), float(values[6]), float(values[7])
                data.append([timestamp, tx, ty, tz, qx, qy, qz, qw])
                valid_lines += 1
    
    print(f"   📄 Líneas totales: {line_count}, Líneas válidas: {valid_lines}")
    if valid_lines > 0:
        arr = np.array(data)
        print(f"   📊 Primera pose: t={arr[0,0]:.6f}, pos=({arr[0,1]:.3f}, {arr[0,2]:.3f}, {arr[0,3]:.3f})")
        print(f"   📊 Última pose:  t={arr[-1,0]:.6f}, pos=({arr[-1,1]:.3f}, {arr[-1,2]:.3f}, {arr[-1,3]:.3f})")
        return arr
    else:
        return np.array([]).reshape(0, 8)

def align_trajectories_smart(traj1, traj2):
    """
    Alineación inteligente para trayectorias del mismo recorrido con diferente cantidad de datos
    Métodos: 1) Interpolación espacial, 2) Resampling temporal, 3) Alineación por distancia recorrida
    """
    print("   🧠 Alineación inteligente iniciada...")
    
    # Extraer posiciones
    pos1 = traj1[:, 1:4]  # x, y, z
    pos2 = traj2[:, 1:4]
    
    print(f"   � Trayectoria 1: {len(pos1)} puntos")
    print(f"   📊 Rayectoria 2: {len(pos2)} puntos")
    
    # Calcular distancias acumuladas para cada trayectoria
    dist1 = np.zeros(len(pos1))
    dist2 = np.zeros(len(pos2))
    
    for i in range(1, len(pos1)):
        dist1[i] = dist1[i-1] + np.linalg.norm(pos1[i] - pos1[i-1])
    for i in range(1, len(pos2)):
        dist2[i] = dist2[i-1] + np.linalg.norm(pos2[i] - pos2[i-1])
    
    total_dist1 = dist1[-1]
    total_dist2 = dist2[-1]
    
    print(f"   � Distancia total 1: {total_dist1:.2f} m")
    print(f"   📏 Distancia total 2: {total_dist2:.2f} m")
    
    # Normalizar distancias (0-1) para que ambas trayectorias tengan el mismo "progreso"
    dist1_norm = dist1 / total_dist1 if total_dist1 > 0 else dist1
    dist2_norm = dist2 / total_dist2 if total_dist2 > 0 else dist2
    
    # Interpolar ambas trayectorias en puntos comunes basados en progreso
    # Usar la trayectoria con menos puntos como referencia
    min_points = min(len(pos1), len(pos2))
    target_progress = np.linspace(0, 1, min_points)
    
    print(f"   🎯 Interpolando a {min_points} puntos comunes...")
    
    # Interpolar trayectoria 1
    interp_pos1 = np.zeros((min_points, 3))
    for dim in range(3):
        interp_pos1[:, dim] = np.interp(target_progress, dist1_norm, pos1[:, dim])
    
    # Interpolar trayectoria 2  
    interp_pos2 = np.zeros((min_points, 3))
    for dim in range(3):
        interp_pos2[:, dim] = np.interp(target_progress, dist2_norm, pos2[:, dim])
    
    # Crear timestamps sintéticos
    timestamps = np.linspace(0, max(len(traj1), len(traj2)), min_points)
    
    # Orientaciones interpoladas (simplificado - mantener última orientación conocida)
    if traj1.shape[1] >= 8 and traj2.shape[1] >= 8:
        # Interpolar orientaciones también
        interp_quat1 = np.zeros((min_points, 4))
        interp_quat2 = np.zeros((min_points, 4))
        
        for q in range(4):
            interp_quat1[:, q] = np.interp(target_progress, dist1_norm, traj1[:, 4+q])
            interp_quat2[:, q] = np.interp(target_progress, dist2_norm, traj2[:, 4+q])
        
        # Reconstruir trayectorias completas
        aligned1 = np.column_stack([timestamps, interp_pos1, interp_quat1])
        aligned2 = np.column_stack([timestamps, interp_pos2, interp_quat2])
    else:
        # Solo posiciones
        aligned1 = np.column_stack([timestamps, interp_pos1, 
                                   np.zeros((min_points, 4))])  # Quaternion dummy
        aligned2 = np.column_stack([timestamps, interp_pos2, 
                                   np.zeros((min_points, 4))])  # Quaternion dummy
    
    print(f"   ✅ Alineación completada: {len(aligned1)} puntos")
    return aligned1, aligned2

def compute_simple_metrics(traj1, traj2):
    """
    Calcula métricas simples entre dos trayectorias alineadas
    Incluye: ATE RMSE, RPE Translation RMSE, RPE Rotation RMSE
    """
    pos1 = traj1[:, 1:4]
    pos2 = traj2[:, 1:4]
    
    # ========== ATE (Absolute Trajectory Error) ==========
    # Diferencias punto a punto
    diff = pos1 - pos2
    distances = np.linalg.norm(diff, axis=1)
    
    # ATE RMSE (Root Mean Square Error)
    ate_rmse = np.sqrt(np.mean(distances**2))
    
    # ========== RPE (Relative Pose Error) ==========
    # Calcular cambios relativos entre poses consecutivas
    rpe_trans_errors = []
    rpe_rot_errors = []
    
    for i in range(len(pos1) - 1):
        # Translation RPE: diferencia en movimiento relativo
        delta_pos1 = pos1[i+1] - pos1[i]
        delta_pos2 = pos2[i+1] - pos2[i]
        trans_error = np.linalg.norm(delta_pos1 - delta_pos2)
        rpe_trans_errors.append(trans_error)
        
        # Rotation RPE: diferencia en rotación relativa
        if traj1.shape[1] >= 8 and traj2.shape[1] >= 8:
            # Extraer quaternions
            q1_i = traj1[i, 4:8]
            q1_next = traj1[i+1, 4:8]
            q2_i = traj2[i, 4:8]
            q2_next = traj2[i+1, 4:8]
            
            # Calcular rotación relativa para ambas trayectorias
            try:
                r1_i = Rotation.from_quat(q1_i)
                r1_next = Rotation.from_quat(q1_next)
                r2_i = Rotation.from_quat(q2_i)
                r2_next = Rotation.from_quat(q2_next)
                
                # Rotación relativa
                delta_rot1 = r1_i.inv() * r1_next
                delta_rot2 = r2_i.inv() * r2_next
                
                # Diferencia de rotación en grados
                rot_diff = delta_rot1.inv() * delta_rot2
                rot_error = np.linalg.norm(rot_diff.as_rotvec()) * 180.0 / np.pi
                rpe_rot_errors.append(rot_error)
            except:
                rpe_rot_errors.append(0.0)
    
    # RPE RMSE
    rpe_trans_rmse = np.sqrt(np.mean(np.array(rpe_trans_errors)**2)) if rpe_trans_errors else 0.0
    rpe_rot_rmse = np.sqrt(np.mean(np.array(rpe_rot_errors)**2)) if rpe_rot_errors else 0.0
    
    # Estadísticas completas
    metrics = {
        # ATE metrics
        'ate_rmse': ate_rmse,
        'mean_error': np.mean(distances),
        'max_error': np.max(distances), 
        'std_error': np.std(distances),
        'errors': distances,
        
        # RPE metrics
        'rpe_translation_rmse': rpe_trans_rmse,
        'rpe_rotation_rmse': rpe_rot_rmse,
        'rpe_trans_errors': np.array(rpe_trans_errors),
        'rpe_rot_errors': np.array(rpe_rot_errors),
        
        # Trajectory lengths
        'total_length_1': np.sum(np.linalg.norm(np.diff(pos1, axis=0), axis=1)),
        'total_length_2': np.sum(np.linalg.norm(np.diff(pos2, axis=0), axis=1)),
    }
    
    return metrics

def plot_simple_trajectories(traj1, traj2, metrics, labels=['Trayectoria 1', 'Trayectoria 2'], 
                           colors=['red', 'blue'], output_file='trajectory_comparison.png'):
    """
    Genera gráfica simple de las dos trayectorias con métricas básicas
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # 1. Vista superior (X-Y)
    ax1.plot(traj1[:, 1], traj1[:, 2], color=colors[0], linewidth=2, 
             label=labels[0], alpha=0.8)
    ax1.plot(traj2[:, 1], traj2[:, 2], color=colors[1], linewidth=2, 
             label=labels[1], alpha=0.8)
    
    # Marcar inicio y fin
    ax1.plot(traj1[0, 1], traj1[0, 2], 'go', markersize=8, label='Inicio')
    ax1.plot(traj1[-1, 1], traj1[-1, 2], 'ro', markersize=8, label='Fin')
    
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title('Comparación de Trayectorias (Vista Superior)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 2. Análisis de error a lo largo del recorrido
    ax2.plot(metrics['errors'], color='purple', linewidth=1.5, alpha=0.7)
    ax2.axhline(y=metrics['mean_error'], color='red', linestyle='--', 
                label=f'Error promedio: {metrics["mean_error"]:.3f}m')
    ax2.axhline(y=metrics['max_error'], color='orange', linestyle=':', 
                label=f'Error máximo: {metrics["max_error"]:.3f}m')
    
    ax2.set_xlabel('Punto de la trayectoria')
    ax2.set_ylabel('Error de distancia [m]')
    ax2.set_title('Error entre Trayectorias')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Añadir tabla de métricas con formato solicitado
    metrics_text = f"""
╔═══════════════════════════════════════╗
║      MÉTRICAS DE EVALUACIÓN           ║
╚═══════════════════════════════════════╝

📊 ATE (Absolute Trajectory Error)
   RMSE:              {metrics['ate_rmse']:.4f} m

📏 RPE (Relative Pose Error)
   Translation RMSE:  {metrics['rpe_translation_rmse']:.4f} m
   Rotation RMSE:     {metrics['rpe_rotation_rmse']:.4f} deg

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Estadísticas Adicionales
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Error promedio:    {metrics['mean_error']:.4f} m
Error máximo:      {metrics['max_error']:.4f} m
Desv. estándar:    {metrics['std_error']:.4f} m

Longitud {labels[0]}: {metrics['total_length_1']:.2f} m
Longitud {labels[1]}: {metrics['total_length_2']:.2f} m
Diferencia:        {abs(metrics['total_length_1'] - metrics['total_length_2']):.2f} m

Puntos comparados: {len(metrics['errors'])}
    """
    
    fig.text(0.02, 0.98, metrics_text, fontsize=10, verticalalignment='top',
             fontfamily='monospace', bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
    
    plt.tight_layout()
    plt.subplots_adjust(left=0.25)  # Hacer espacio para la tabla
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✅ Gráfica guardada en: {output_file}")
    plt.show()

def plot_trajectories(traj_est, traj_gt, ate_results, rpe_results, output_file='trajectory_comparison.png'):
    """
    Genera gráficas de comparación
    """
    fig = plt.figure(figsize=(20, 12))
    
    # 1. Trayectorias 2D (Top view)
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(traj_gt[:, 1], traj_gt[:, 2], 'g-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax1.plot(traj_est[:, 1], traj_est[:, 2], 'r--', linewidth=2, label='Visual Odometry', alpha=0.7)
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title('Trajectory Comparison (Top View)')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # 2. Trayectorias X-Z (Side view)
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(traj_gt[:, 1], traj_gt[:, 3], 'g-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax2.plot(traj_est[:, 1], traj_est[:, 3], 'r--', linewidth=2, label='Visual Odometry', alpha=0.7)
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Z [m]')
    ax2.set_title('Trajectory Comparison (Side View)')
    ax2.legend()
    ax2.grid(True)
    
    # 3. ATE a lo largo del tiempo
    ax3 = plt.subplot(2, 3, 3)
    timestamps = traj_est[:, 0] - traj_est[0, 0]
    ax3.plot(timestamps, ate_results['errors'], 'b-', linewidth=1)
    ax3.axhline(y=ate_results['mean'], color='r', linestyle='--', label=f'Mean: {ate_results["mean"]:.3f}m')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('ATE [m]')
    ax3.set_title('Absolute Trajectory Error')
    ax3.legend()
    ax3.grid(True)
    
    # 4. RPE Translation
    ax4 = plt.subplot(2, 3, 4)
    ax4.plot(rpe_results['trans_errors'], 'b-', linewidth=1)
    ax4.axhline(y=rpe_results['trans_mean'], color='r', linestyle='--', 
                label=f'Mean: {rpe_results["trans_mean"]:.3f}m')
    ax4.set_xlabel('Frame Index')
    ax4.set_ylabel('RPE Translation [m]')
    ax4.set_title('Relative Pose Error (Translation)')
    ax4.legend()
    ax4.grid(True)
    
    # 5. RPE Rotation
    ax5 = plt.subplot(2, 3, 5)
    ax5.plot(rpe_results['rot_errors'], 'g-', linewidth=1)
    ax5.axhline(y=rpe_results['rot_mean'], color='r', linestyle='--', 
                label=f'Mean: {rpe_results["rot_mean"]:.3f}°')
    ax5.set_xlabel('Frame Index')
    ax5.set_ylabel('RPE Rotation [deg]')
    ax5.set_title('Relative Pose Error (Rotation)')
    ax5.legend()
    ax5.grid(True)
    
    # 6. Tabla de métricas
    ax6 = plt.subplot(2, 3, 6)
    ax6.axis('off')
    
    metrics_text = f"""
    ABSOLUTE TRAJECTORY ERROR (ATE)
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    Mean:        {ate_results['mean']:.4f} m
    RMSE:        {ate_results['rmse']:.4f} m
    Std Dev:     {ate_results['std']:.4f} m
    Max:         {ate_results['max']:.4f} m
    Min:         {ate_results['min']:.4f} m
    
    RELATIVE POSE ERROR (RPE)
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    Translation Mean:  {rpe_results['trans_mean']:.4f} m
    Translation RMSE:  {rpe_results['trans_rmse']:.4f} m
    Translation Std:   {rpe_results['trans_std']:.4f} m
    
    Rotation Mean:     {rpe_results['rot_mean']:.4f} deg
    Rotation RMSE:     {rpe_results['rot_rmse']:.4f} deg
    Rotation Std:      {rpe_results['rot_std']:.4f} deg
    
    TRAJECTORY INFO
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    Number of poses:   {len(traj_est)}
    Total distance:    {np.sum(np.linalg.norm(np.diff(traj_gt[:, 1:4], axis=0), axis=1)):.2f} m
    """
    
    ax6.text(0.1, 0.9, metrics_text, transform=ax6.transAxes, fontsize=11,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✅ Gráfica guardada en: {output_file}")
    plt.show()

def main():
    parser = argparse.ArgumentParser(description='Comparar trayectorias VO vs Ground Truth')
    parser.add_argument('--vo', default='/home/jorgelop/Documents/VantTec_SDV_SWARM/SDV_Proyect/SDV_Software_Workspaces/vo_ws/src/vodom_first/trayectories/vo_trajectory.txt', help='Archivo de trayectoria VO')
    parser.add_argument('--gt', default='/home/jorgelop/Documents/VantTec_SDV_SWARM/SDV_Proyect/SDV_Software_Workspaces/vo_ws/src/vodom_first/trayectories/gt_trajectory.txt', help='Archivo de ground truth')
    parser.add_argument('--output', default='trajectory_comparison.png', help='Archivo de salida')
    args = parser.parse_args()
    
    # Expandir ~ a home directory
    vo_file = os.path.expanduser(args.vo)
    gt_file = os.path.expanduser(args.gt)
    
    print("🚀 Análisis de Odometría Visual")
    print("=" * 50)
    
    # Cargar trayectorias
    print(f"📂 Cargando VO: {vo_file}")
    traj_vo = load_trajectory(vo_file)
    print(f"   ✓ {len(traj_vo)} poses cargadas")
    
    print(f"📂 Cargando GT: {gt_file}")
    traj_gt = load_trajectory(gt_file)
    print(f"   ✓ {len(traj_gt)} poses cargadas")
    
    # Alinear trayectorias con algoritmo inteligente
    print("🔄 Alineando trayectorias con interpolación espacial...")
    traj_vo_aligned, traj_gt_aligned = align_trajectories_smart(traj_vo, traj_gt)
    print(f"   ✓ {len(traj_vo_aligned)} poses alineadas")
    
    if len(traj_vo_aligned) < 2:
        print("❌ Error: No hay suficientes poses alineadas")
        return
    
    # Calcular métricas completas
    print("\n📊 Calculando métricas de evaluación...")
    metrics = compute_simple_metrics(traj_vo_aligned, traj_gt_aligned)
    
    print("\n" + "="*50)
    print("📊 RESULTADOS DE EVALUACIÓN")
    print("="*50)
    print(f"\n✅ ATE (Absolute Trajectory Error)")
    print(f"   RMSE: {metrics['ate_rmse']:.4f} m")
    
    print(f"\n✅ RPE (Relative Pose Error)")
    print(f"   Translation RMSE: {metrics['rpe_translation_rmse']:.4f} m")
    print(f"   Rotation RMSE:    {metrics['rpe_rotation_rmse']:.4f} deg")
    
    print(f"\n📈 Estadísticas adicionales:")
    print(f"   Error promedio:   {metrics['mean_error']:.4f} m")
    print(f"   Error máximo:     {metrics['max_error']:.4f} m")
    print(f"   Desv. estándar:   {metrics['std_error']:.4f} m")
    print("="*50)
    
    # Generar gráficas simplificadas
    print("\n📈 Generando gráficas de comparación...")
    plot_simple_trajectories(traj_vo_aligned, traj_gt_aligned, metrics, 
                            labels=['Visual Odometry', 'Ground Truth'],
                            colors=['red', 'blue'],
                            output_file=args.output)
    
    print("\n✅ Análisis completado!")

if __name__ == '__main__':
    main()
