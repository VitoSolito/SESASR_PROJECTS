from utils import sample_velocity_motion_model, velocity_mm_Gt, velocity_mm_Vt,evaluate_sampling_dist
import numpy as np
from math import sin, cos
import matplotlib.pyplot as plt
import matplotlib as mpl
from math import degrees
arrow = u'$\u2191$'
def main():
    x0=np.array([0.0,0.0,0.0])
    u=np.array([1.0,0.5])
    dt=1.0
    n_samples=1000
    samples_angular = []
    samples_linear = []
    #High angular velocity noise
    a_angular = np.array([0.05, 0.5, 0.05, 0.5, 0.05, 0.5])
    #High linear velocity noise
    a_linear = np.array([0.5, 0.05, 0.5, 0.05, 0.5, 0.05])

    #sampling 
    for i in range(n_samples):
        x_angular = sample_velocity_motion_model(np.array([x0]), u, a_angular, dt)
        x_linear = sample_velocity_motion_model(np.array([x0]), u, a_linear, dt)
        samples_angular.append(x_angular)
        samples_linear.append(x_linear)

    samples_angular = np.array(samples_angular)
    samples_linear = np.array(samples_linear)

    #jacobians
    Gt = velocity_mm_Gt(x0, u, dt)
    Vt = velocity_mm_Vt(x0, u, dt)

    #statistics
    mean_angular = np.mean(samples_angular, axis=0)
    mean_linear = np.mean(samples_linear, axis=0)

    std_angular = np.std(samples_angular, axis=0)
    std_linear = np.std(samples_linear, axis=0)
    print("High Angular Velocity Noise:")
    print(f"Mean: {mean_angular}, Std Dev: {std_angular}")

    print("High Linear Velocity Noise:")
    print(f"Mean: {mean_linear}, Std Dev: {std_linear}")

    '''
    #Evaluate sampling distribution for angular noise
    print("\nEvaluating sampling distribution for high angular velocity noise:")
    for i in range(3):
        print(f"State dimension {i}:")
        evaluate_sampling_dist(mean_angular[i], a_angular[i], n_samples,np.random.normal)

    #Evaluate sampling distribution for linear noise
    print("\nEvaluating sampling distribution for high linear velocity noise:")
    
    for i in range(3):
        print(f"State dimension {i}:")
        evaluate_sampling_dist(mean_linear[i], a_linear[i], n_samples,np.random.normal) '''


    print("\n[PASSO 6] Generazione plot...")

    # Plot 1: Distribuzione spaziale campioni (x, y)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # Subplot 1: Alta incertezza angolare
    axes[0].scatter(samples_angular[:, 0], samples_angular[:, 1], alpha=0.5, s=20, c='blue', edgecolors='none', label='Campioni')
    axes[0].scatter(x0[0], x0[1], c='red', s=200, marker='*', 
                    label='Stato iniziale', edgecolors='black', linewidths=1.5, zorder=5)
    axes[0].set_xlabel('x [m]', fontsize=12)
    axes[0].set_ylabel('y [m]', fontsize=12)
    axes[0].set_title('Alta incertezza ANGOLARE (ω)', fontsize=14, fontweight='bold')
    axes[0].grid(True, alpha=0.3, linestyle='--')
    axes[0].legend(fontsize=11)
    axes[0].axis('equal')

    # Subplot 2: Alta incertezza lineare
    axes[1].scatter(samples_linear[:, 0], samples_linear[:, 1], 
                    alpha=0.5, s=20, c='green', edgecolors='none', label='Campioni')
    axes[1].scatter(x0[0], x0[1], c='red', s=200, marker='*', 
                    label='Stato iniziale', edgecolors='black', linewidths=1.5, zorder=5)
    axes[1].set_xlabel('x [m]', fontsize=12)
    axes[1].set_ylabel('y [m]', fontsize=12)
    axes[1].set_title('Alta incertezza LINEARE (v)', fontsize=14, fontweight='bold')
    axes[1].grid(True, alpha=0.3, linestyle='--')
    axes[1].legend(fontsize=11)
    axes[1].axis('equal')

    plt.tight_layout()
    plt.savefig('velocity_motion_samples_xy.png', dpi=300, bbox_inches='tight')
    print("  ✓ Plot salvato: 'velocity_motion_samples_xy.png'")

    # Plot 2: Distribuzione angolo theta
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    axes[0].hist(samples_angular[:, 2], bins=30, alpha=0.7, color='blue', edgecolor='black')
    axes[0].axvline(mean_angular[2], color='red', linestyle='--', linewidth=2, label=f'Media = {mean_angular[2]:.3f}')
    axes[0].set_xlabel('θ [rad]', fontsize=12)
    axes[0].set_ylabel('Frequenza', fontsize=12)
    axes[0].set_title('Distribuzione θ - Alta incertezza angolare', fontsize=13, fontweight='bold')
    axes[0].grid(True, alpha=0.3, axis='y')
    axes[0].legend(fontsize=11)

    axes[1].hist(samples_linear[:, 2], bins=30, alpha=0.7, color='green', edgecolor='black')
    axes[1].axvline(mean_linear[2], color='red', linestyle='--', linewidth=2, label=f'Media = {mean_linear[2]:.3f}')
    axes[1].set_xlabel('θ [rad]', fontsize=12)
    axes[1].set_ylabel('Frequenza', fontsize=12)
    axes[1].set_title('Distribuzione θ - Alta incertezza lineare', fontsize=13, fontweight='bold')
    axes[1].grid(True, alpha=0.3, axis='y')
    axes[1].legend(fontsize=11)

    plt.tight_layout()
    plt.savefig('velocity_motion_theta_distribution.png', dpi=300, bbox_inches='tight')
    print("  ✓ Plot salvato: 'velocity_motion_theta_distribution.png'")

    plt.show()
    

    #PLOT AVANZATO
    

    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x0[2])-90)
    plt.scatter(x0[0], x0[1], marker=rotated_marker, s=100, facecolors='none', edgecolors='b')

    for x_ in samples_linear[:200]:
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x_[2])-90)
        plt.scatter(x_[0], x_[1], marker=rotated_marker, s=40, facecolors='none', edgecolors='r')

    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title("velocity motion model sampling with high linear velocity noise")
    plt.show()

    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x0[2])-90)
    plt.scatter(x0[0], x0[1], marker=rotated_marker, s=100, facecolors='none', edgecolors='b')

    for x_ in samples_angular[:200]:
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x_[2])-90)
        plt.scatter(x_[0], x_[1], marker=rotated_marker, s=40, facecolors='none', edgecolors='r')

    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title("velocity motion model sampling with high angular velocity noise")
    plt.show()    

main()