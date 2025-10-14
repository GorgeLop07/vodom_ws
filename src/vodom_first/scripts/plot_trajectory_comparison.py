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

def align_trajectories(traj1, traj2):
    """
    Alinea trayectorias por timestamp (sincronización)
    Si los timestamps son todos 0, alinea por índice
    """
    timestamps1 = traj1[:, 0]
    timestamps2 = traj2[:, 0]
    
    print(f"   🔍 Debug: VO timestamps: min={np.min(timestamps1):.6f}, max={np.max(timestamps1):.6f}, std={np.std(timestamps1):.6f}")
    print(f"   🔍 Debug: GT timestamps: min={np.min(timestamps2):.6f}, max={np.max(timestamps2):.6f}, std={np.std(timestamps2):.6f}")
    
    # Verificar si CUALQUIERA de los dos tiene timestamps todos iguales (o ambos)
    vo_constant = np.std(timestamps1) < 0.001
    gt_constant = np.std(timestamps2) < 0.001
    
    if vo_constant or gt_constant:
        print("   ⚠️  Timestamps inconsistentes, alineando por índice...")
        # Alinear por índice (usar el tamaño más pequeño)
        min_len = min(len(traj1), len(traj2))
        print(f"   📏 Usando {min_len} poses de {len(traj1)} VO y {len(traj2)} GT")
        return traj1[:min_len], traj2[:min_len]
    
    # Encontrar timestamps comunes (con tolerancia)
    aligned1 = []
    aligned2 = []
    
    for i, t1 in enumerate(timestamps1):
        # Buscar timestamp más cercano en traj2
        idx = np.argmin(np.abs(timestamps2 - t1))
        if np.abs(timestamps2[idx] - t1) < 0.1:  # Tolerancia de 100ms
            aligned1.append(traj1[i])
            aligned2.append(traj2[idx])
    
    print(f"   📏 Alineación por timestamp: {len(aligned1)} poses comunes")
    return np.array(aligned1), np.array(aligned2)

def compute_ate(traj_est, traj_gt):
    """
    Calcula Absolute Trajectory Error (ATE)
    """
    # Extraer posiciones
    pos_est = traj_est[:, 1:4]
    pos_gt = traj_gt[:, 1:4]
    
    # Calcular errores
    errors = np.linalg.norm(pos_est - pos_gt, axis=1)
    
    ate_mean = np.mean(errors)
    ate_rmse = np.sqrt(np.mean(errors**2))
    ate_std = np.std(errors)
    ate_max = np.max(errors)
    ate_min = np.min(errors)
    
    return {
        'mean': ate_mean,
        'rmse': ate_rmse,
        'std': ate_std,
        'max': ate_max,
        'min': ate_min,
        'errors': errors
    }

def compute_rpe(traj_est, traj_gt, delta=1):
    """
    Calcula Relative Pose Error (RPE)
    delta: distancia entre poses para calcular error relativo
    """
    pos_est = traj_est[:, 1:4]
    pos_gt = traj_gt[:, 1:4]
    
    trans_errors = []
    rot_errors = []
    
    for i in range(len(traj_est) - delta):
        # Error de traslación relativa
        trans_est = pos_est[i+delta] - pos_est[i]
        trans_gt = pos_gt[i+delta] - pos_gt[i]
        trans_error = np.linalg.norm(trans_est - trans_gt)
        trans_errors.append(trans_error)
        
        # Error de rotación relativa
        q_est_1 = traj_est[i, 4:8]
        q_est_2 = traj_est[i+delta, 4:8]
        q_gt_1 = traj_gt[i, 4:8]
        q_gt_2 = traj_gt[i+delta, 4:8]
        
        # Rotación relativa
        R_est_1 = Rotation.from_quat(q_est_1).as_matrix()
        R_est_2 = Rotation.from_quat(q_est_2).as_matrix()
        R_gt_1 = Rotation.from_quat(q_gt_1).as_matrix()
        R_gt_2 = Rotation.from_quat(q_gt_2).as_matrix()
        
        R_est_rel = R_est_2 @ R_est_1.T
        R_gt_rel = R_gt_2 @ R_gt_1.T
        R_error = R_gt_rel.T @ R_est_rel
        
        # Convertir a ángulo
        angle_error = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))
        rot_errors.append(np.degrees(angle_error))
    
    trans_errors = np.array(trans_errors)
    rot_errors = np.array(rot_errors)
    
    return {
        'trans_mean': np.mean(trans_errors),
        'trans_rmse': np.sqrt(np.mean(trans_errors**2)),
        'trans_std': np.std(trans_errors),
        'rot_mean': np.mean(rot_errors),
        'rot_rmse': np.sqrt(np.mean(rot_errors**2)),
        'rot_std': np.std(rot_errors),
        'trans_errors': trans_errors,
        'rot_errors': rot_errors
    }

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
    
    # Alinear trayectorias
    print("🔄 Alineando trayectorias...")
    traj_vo_aligned, traj_gt_aligned = align_trajectories(traj_vo, traj_gt)
    print(f"   ✓ {len(traj_vo_aligned)} poses alineadas")
    
    if len(traj_vo_aligned) < 2:
        print("❌ Error: No hay suficientes poses alineadas")
        return
    
    # Calcular métricas
    print("\n📊 Calculando métricas...")
    ate_results = compute_ate(traj_vo_aligned, traj_gt_aligned)
    print(f"   ATE RMSE: {ate_results['rmse']:.4f} m")
    
    rpe_results = compute_rpe(traj_vo_aligned, traj_gt_aligned, delta=1)
    print(f"   RPE Trans RMSE: {rpe_results['trans_rmse']:.4f} m")
    print(f"   RPE Rot RMSE: {rpe_results['rot_rmse']:.4f} deg")
    
    # Generar gráficas
    print("\n📈 Generando gráficas...")
    plot_trajectories(traj_vo_aligned, traj_gt_aligned, ate_results, rpe_results, args.output)
    
    print("\n✅ Análisis completado!")

if __name__ == '__main__':
    main()
