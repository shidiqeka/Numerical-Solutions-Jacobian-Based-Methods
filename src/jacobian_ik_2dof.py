import numpy as np
import matplotlib.pyplot as plt

class Planar2DOF:
    def __init__(self, l1, l2):
        """Inisialisasi panjang link lengan robot."""
        self.l1 = l1
        self.l2 = l2
        self.q = np.array([0.0, 0.0]) # Sudut awal [theta1, theta2] dalam radian

    def forward_kinematics(self, q=None):
        """Menghitung posisi end-effector (x, y) berdasarkan sudut sendi."""
        if q is None:
            q = self.q
        x = self.l1 * np.cos(q[0]) + self.l2 * np.cos(q[0] + q[1])
        y = self.l1 * np.sin(q[0]) + self.l2 * np.sin(q[0] + q[1])
        return np.array([x, y])

    def jacobian(self, q=None):
        """Menghitung matriks Jacobian 2x2 secara analitik."""
        if q is None:
            q = self.q
        
        j11 = -self.l1 * np.sin(q[0]) - self.l2 * np.sin(q[0] + q[1])
        j12 = -self.l2 * np.sin(q[0] + q[1])
        j21 =  self.l1 * np.cos(q[0]) + self.l2 * np.cos(q[0] + q[1])
        j22 =  self.l2 * np.cos(q[0] + q[1])
        
        return np.array([[j11, j12], 
                         [j21, j22]])

    def inverse_kinematics(self, target, alpha=0.05, max_iter=200, tol=1e-3):
        """
        Menyelesaikan Inverse Kinematics menggunakan Pseudo-inverse Jacobian.
        Mengembalikan list riwayat sudut untuk keperluan animasi.
        """
        history = [self.q.copy()]
        
        for _ in range(max_iter):
            current_pos = self.forward_kinematics()
            error = target - current_pos
            
            # Jika error sudah sangat kecil, iterasi berhenti
            if np.linalg.norm(error) < tol:
                break
                
            J = self.jacobian()
            
            # Menggunakan Pseudo-inverse (pinv) untuk menangani singularitas
            J_pinv = np.linalg.pinv(J)
            
            # Update posisi sudut: q_new = q_old + alpha * J_pinv * error
            dq = alpha * np.dot(J_pinv, error)
            self.q = self.q + dq
            
            history.append(self.q.copy())
            
        return history

def animate_robot(robot, history, target):
    """Fungsi pembantu untuk memvisualisasikan pergerakan robot."""
    plt.ion()
    fig, ax = plt.subplots()
    
    for i, q_step in enumerate(history):
        ax.clear()
        ax.set_xlim(-(robot.l1 + robot.l2 + 50), robot.l1 + robot.l2 + 50)
        ax.set_ylim(-(robot.l1 + robot.l2 + 50), robot.l1 + robot.l2 + 50)
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_title(f"Iterasi IK Jacobian: {i}/{len(history)-1}")
        
        # Hitung posisi joint
        x0, y0 = 0, 0
        x1 = robot.l1 * np.cos(q_step[0])
        y1 = robot.l1 * np.sin(q_step[0])
        x2, y2 = robot.forward_kinematics(q_step)
        
        # Plot target
        ax.plot(target[0], target[1], 'r*', markersize=10, label='Target')
        
        # Plot link robot
        ax.plot([x0, x1, x2], [y0, y1, y2], '-o', lw=4, color='blue', label='Lengan Robot')
        
        ax.legend()
        plt.pause(0.01)
        
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    # Setup Robot: Panjang L1 = 150, L2 = 100
    robot = Planar2DOF(l1=150.0, l2=100.0)
    
    # Set Titik Target (x, y)
    target_pos = np.array([100, 150])
    
    print(f"Memulai Inverse Kinematics menuju target: {target_pos}")
    # Jalankan algoritma IK
    q_history = robot.inverse_kinematics(target=target_pos, alpha=0.5)
    
    print(f"Selesai dalam {len(q_history)-1} iterasi.")
    print(f"Sudut akhir (radian): {q_history[-1]}")
    
    # Tampilkan Animasi
    animate_robot(robot, q_history, target_pos)