import numpy as np
from math import sin, cos
import matplotlib.pyplot as plt
import matplotlib as mpl
from math import degrees
from utils import sample_velocity_motion_model, velocity_mm_Gt, velocity_mm_Vt,evaluate_sampling_dist, landmark_range_bearing_model
from math import atan2, sqrt, sin, cos

def sample_measurement_landmark_model(x0,z, landmark, sigma_r, sigma_phi, sigma_theta):
    """
    Infere the correct robot pose knowing its initial pose, landmark position and measurement z w.r.t. the landmark

    """
    r = z[0]+np.random.normal(0, sigma_r)
    phi=z[1]+np.random.normal(0, sigma_phi)
    theta = x0[2] +np.random.normal(0, sigma_theta)
    theta_absolute= phi + theta

    x = landmark[0] - r * cos(theta_absolute)
    y = landmark[1] - r * sin(theta_absolute)

    return np.array([x, y, theta])

def sample_measurement_model_no_landmark(x0, z, sigma_r, sigma_phi, sigma_theta):
    """
    Infere the correct robot pose knowing its initial pose and measurement z.
    Estimates landmark position from x0 and z.
    """
     # Campiona theta con MAGGIORE incertezza
    #theta_sample = x0[2] + np.random.normal(0, sigma_theta)
    theta_sample = np.random.uniform(-np.pi, np.pi) 

    # Campiona misura con rumore
    r_sample = z[0] + np.random.normal(0, sigma_r)
    phi_sample = z[1] + np.random.normal(0, sigma_phi)
    
    # Stima UNICO landmark (stabile) usando valori NOMINALI
    theta_abs_nominal = x0[2] + z[1]
    landmark_est = np.array([
        x0[0] + z[0] * cos(theta_abs_nominal),
        x0[1] + z[0] * sin(theta_abs_nominal)
    ])
    
    # Ora: dato questo landmark stimato, genera pose sul cerchio
    # usando i valori CAMPIONATI
    theta_abs_pose = theta_sample + phi_sample
    
    x_sample = landmark_est[0] - r_sample * cos(theta_abs_pose)
    y_sample = landmark_est[1] - r_sample * sin(theta_abs_pose)
    
    return np.array([x_sample, y_sample, theta_sample]), landmark_est


def mesaurement_jacobian_H(x,landmark):
    '''
    z = [r={sqrt((x-xl)**2 + (y-yl)**2)}, phi={atan((y-yl)/(x-xl)-theta)} ]

    dr/dx = delta_x/d
    dr/dy = delta_y/d
    dr/dtheta = 0

    dphi/dx = -delta_y/q
    dphi/dy = delta_x/q
    dphi/dtheta=-1

    delta_x = x-xl
    delta_y=y-yl
    q= delta_x**2 + delta_y **2

    '''

    delta_x = (x[0]-landmark[0])
    delta_y=(x[1]-landmark[1])
    q = delta_x**2 + delta_y**2
    r = sqrt(q)

    H = np.array([[delta_x/r , delta_y/r , 0],[-delta_y/q , delta_x/q, -1]])

    return H


def main():
    x0 = np.array([2.0,1.0,np.pi/6])
    landmark = np.array([5.0,3.0])
    sigma_r= 1.0
    sigma_phi=0.5
    sigma_theta=0.3

    n_samples=5000

   

    #Forward measurement model
    z_measured= landmark_range_bearing_model(x0, landmark, np.array([sigma_r, sigma_phi]))

    
    #Inverse measurement model with known landmark
    samples_with_landmark=[]
    for i in range(n_samples):
        
        x_sample = sample_measurement_landmark_model(x0, z_measured, landmark, sigma_r, sigma_phi, sigma_theta)
        samples_with_landmark.append(x_sample)
    samples_with_landmark = np.array(samples_with_landmark)

    #Inverse measurement model without unknown landmark
    samples_no_landmark=[]
    landmarks_estimated=[]
    for i in range(n_samples):
        x_sample, landmark_estimated = sample_measurement_model_no_landmark(x0, z_measured, sigma_r, sigma_phi, sigma_theta)
        samples_no_landmark.append(x_sample)
        landmarks_estimated.append(landmark_estimated)
    samples_no_landmark = np.array(samples_no_landmark)
    landmarks_estimated = np.array(landmarks_estimated)

    #Statistics
    mean_with_landmark = np.mean(samples_with_landmark, axis=0)
    std_with_landmark = np.std(samples_with_landmark, axis=0)

    mean_no_landmark = np.mean(samples_no_landmark, axis=0)
    std_no_landmark = np.std(samples_no_landmark, axis=0)

    print("With Known Landmark:")
    print(f"Mean: {mean_with_landmark}, Std Dev: {std_with_landmark}")

    print("Without Known Landmark:")
    print(f"Mean: {mean_no_landmark}, Std Dev: {std_no_landmark}")



    fig = plt.figure(figsize=(16, 10))
    
    # ========== PLOT 1: Distribuzione spaziale CON landmark ==========
    ax1 = plt.subplot(2, 3, 1)
    ax1.scatter(samples_with_landmark[:, 0], samples_with_landmark[:, 1], 
                alpha=0.3, s=10, c='blue', label='Campioni')
    ax1.scatter(landmark[0], landmark[1], 
                c='red', s=250, marker='*', label='Landmark vero',
                edgecolors='black', linewidths=2, zorder=5)
    ax1.scatter(x0[0], x0[1], 
                c='green', s=150, marker='o', label='Stato iniziale',
                edgecolors='black', linewidths=1.5, zorder=4)
    ax1.set_xlabel('x [m]', fontsize=11)
    ax1.set_ylabel('y [m]', fontsize=11)
    ax1.set_title('CASO A: Landmark NOTO', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=9)
    ax1.axis('equal')
    
    # ========== PLOT 2: Distribuzione spaziale SENZA landmark ==========
    ax2 = plt.subplot(2, 3, 2)
    ax2.scatter(samples_no_landmark[:, 0], samples_no_landmark[:, 1], 
                alpha=0.3, s=10, c='orange', label='Campioni')
    ax2.scatter(landmark_estimated[0], landmark_estimated[1], 
                c='purple', s=250, marker='*', label='Landmark stimato',
                edgecolors='black', linewidths=2, zorder=5)
    ax2.scatter(landmark[0], landmark[1], 
                c='red', s=200, marker='*', label='Landmark vero',
                edgecolors='black', linewidths=1, alpha=0.5, zorder=4)
    ax2.scatter(x0[0], x0[1], 
                c='green', s=150, marker='o', label='Stato iniziale',
                edgecolors='black', linewidths=1.5, zorder=4)
    ax2.set_xlabel('x [m]', fontsize=11)
    ax2.set_ylabel('y [m]', fontsize=11)
    ax2.set_title('CASO B: Landmark NON NOTO', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=9)
    ax2.axis('equal')
    
    # ========== PLOT 3: Sovrapposizione ==========
    ax3 = plt.subplot(2, 3, 3)
    ax3.scatter(samples_with_landmark[:, 0], samples_with_landmark[:, 1], 
                alpha=0.2, s=8, c='blue', label='Con landmark')
    ax3.scatter(samples_no_landmark[:, 0], samples_no_landmark[:, 1], 
                alpha=0.2, s=8, c='orange', label='Senza landmark')
    ax3.scatter(landmark[0], landmark[1], 
                c='red', s=250, marker='*', label='Landmark vero',
                edgecolors='black', linewidths=2, zorder=5)
    ax3.scatter(x0[0], x0[1], 
                c='green', s=150, marker='o', label='Stato iniziale',
                edgecolors='black', linewidths=1.5, zorder=4)
    ax3.set_xlabel('x [m]', fontsize=11)
    ax3.set_ylabel('y [m]', fontsize=11)
    ax3.set_title('CONFRONTO', fontsize=13, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=9)
    ax3.axis('equal')
    
    # ========== PLOT 4: Distribuzione theta CON landmark ==========
    ax4 = plt.subplot(2, 3, 4)
    ax4.hist(samples_with_landmark[:, 2], bins=30, alpha=0.7, 
             color='blue', edgecolor='black', density=True)
    ax4.axvline(mean_with_landmark[2], color='red', linestyle='--', 
                linewidth=2, label=f'Media={mean_with_landmark[2]:.3f}')
    ax4.axvline(x0[2], color='green', linestyle='--', 
                linewidth=2, label=f'Iniziale={x0[2]:.3f}')
    ax4.set_xlabel('θ [rad]', fontsize=11)
    ax4.set_ylabel('Densità', fontsize=11)
    ax4.set_title('Distribuzione θ - Landmark NOTO', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3, axis='y')
    ax4.legend(fontsize=9)
    
    # ========== PLOT 5: Distribuzione theta SENZA landmark ==========
    ax5 = plt.subplot(2, 3, 5)
    ax5.hist(samples_no_landmark[:, 2], bins=30, alpha=0.7, 
             color='orange', edgecolor='black', density=True)
    ax5.axvline(mean_no_landmark[2], color='purple', linestyle='--', 
                linewidth=2, label=f'Media={mean_no_landmark[2]:.3f}')
    ax5.axvline(x0[2], color='green', linestyle='--', 
                linewidth=2, label=f'Iniziale={x0[2]:.3f}')
    ax5.set_xlabel('θ [rad]', fontsize=11)
    ax5.set_ylabel('Densità', fontsize=11)
    ax5.set_title('Distribuzione θ - Landmark NON NOTO', fontsize=12, fontweight='bold')
    ax5.grid(True, alpha=0.3, axis='y')
    ax5.legend(fontsize=9)
    
    # ========== PLOT 6: Confronto deviazioni standard ==========
    ax6 = plt.subplot(2, 3, 6)
    labels = ['x', 'y', 'θ']
    x_pos = np.arange(len(labels))
    width = 0.35
    
    ax6.bar(x_pos - width/2, std_with_landmark, width, label='Con landmark', 
            color='blue', alpha=0.7, edgecolor='black')
    ax6.bar(x_pos + width/2, std_no_landmark, width, label='Senza landmark', 
            color='orange', alpha=0.7, edgecolor='black')
    
    ax6.set_ylabel('Deviazione Standard', fontsize=11)
    ax6.set_title('Confronto Incertezza', fontsize=13, fontweight='bold')
    ax6.set_xticks(x_pos)
    ax6.set_xticklabels(labels)
    ax6.legend(fontsize=9)
    ax6.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig('measurement_model_complete_comparison.png', dpi=300, bbox_inches='tight')
    print("  ✓ Plot salvato: 'measurement_model_complete_comparison.png'")
    
    plt.show()



main()
