import numpy as np
import matplotlib.pyplot as plt

class Planar3DOF:
    def __init__(self, l1, l2, l3):
        """Inisialisasi panjang link lengan robot."""
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.q = np.array([0.0, 0.0, 0.0]) # Sudut awal [q1, q2, q3] dalam radian

    def forward_kinematics(self, q=None):
        """Menghitung posisi (x, y) dan orientasi (phi) end-effector."""
        if q is None:
            q = self.q
            
        q1, q2, q3 = q[0], q[1], q[2]
        
        x = self.l1 * np.cos(q1) + self.l2 * np.cos(q1 + q2) + self.l3 * np.cos(q1 + q2 + q3)
        y = self.l1 * np.sin(q1) + self.l2 * np.sin(q1 + q2) + self.l3 * np.sin(q1 + q2 + q3)
        phi = q1 + q2 + q3
        
        return np.array([x, y, phi])

    def jacobian(self, q=None):
        """Menghitung matriks Jacobian 3x3 secara analitik."""
        if q is None:
            q = self.q
            
        q1, q2, q3 = q[0], q[1], q[2]
        q12 = q1 + q2
        q123 = q1 + q2 + q3
        
        j11 = -self.l1 * np.sin(q1) - self.l2 * np.sin(q12) - self.l3 * np.sin(q123)
        j12 = -self.l2 * np.sin(q12) - self.l3 * np.sin(q123)
        j13 = -self.l3 * np.sin(q123)
        
        j21 =  self.l1 * np.cos(q1) + self.l2 * np.cos(q12) + self.l3 * np.cos(q123)
        j22 =  self.l2 * np.cos(q12) + self.l3 * np.cos(q123)
        j23 =  self.l3 * np.cos(q123)
        
        j31, j32, j33 = 1.0, 1.0, 1.0
        
        return np.array([[j11, j12, j13], 
                         [j21, j22, j23],
                         [j31, j32, j33]])

    def _wrap_angle(self, angle):
        """Normalisasi sudut ke rentang [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def inverse_kinematics(self, target, alpha=0.1, max_iter=200, tol=1e-3, position_only=True):
        """
        Menyelesaikan IK. 
        position_only=True memotong Jacobian menjadi 2x3 (Mode Redundan).
        """
        history = [self.q.copy()]
        
        for _ in range(max_iter):
            current_pos = self.forward_kinematics()
            
            if position_only:
                # Hanya hitung error X dan Y
                error = target[:2] - current_pos[:2]
                # Potong Jacobian menjadi 2x3
                J = self.jacobian()[:2, :] 
            else:
                error = target - current_pos
                error[2] = self._wrap_angle(error[2]) 
                J = self.jacobian()
            
            if np.linalg.norm(error) < tol:
                break
                
            J_pinv = np.linalg.pinv(J)
            dq = alpha * np.dot(J_pinv, error)
            self.q = self.q + dq
            
            # --- TAMBAHKAN BARIS INI ---
            # Normalisasi semua sudut q agar tetap di rentang -180 hingga 180 derajat (-pi hingga pi)
            self.q = self._wrap_angle(self.q)
            # ---------------------------
            
            history.append(self.q.copy())
            
        return history

class InteractiveVisualizer:
    def __init__(self, robot):
        self.robot = robot
        self.target = np.array([0.0, 150.0]) 
        self.is_moving = False
        
        self.fig, self.ax = plt.subplots(figsize=(9, 8)) # Sedikit diperlebar untuk teks
        self.max_reach = self.robot.l1 + self.robot.l2 + self.robot.l3 + 20
        
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.draw_plot("Mode Redundan Aktif! Klik di mana saja pada grid.")
        plt.show() 

    def on_click(self, event):
        if event.xdata is None or event.ydata is None or self.is_moving:
            return
            
        self.target[0] = event.xdata
        self.target[1] = event.ydata
        self.run_ik_and_animate()

    def run_ik_and_animate(self):
        self.is_moving = True
        self.draw_plot("Menghitung IK Redundan (2x3)...")
        
        history = self.robot.inverse_kinematics(self.target, alpha=0.3, max_iter=200, position_only=True)
        
        for i, q_step in enumerate(history):
            if i % 2 != 0 and i != len(history) - 1:
                continue
                
            self.robot.q = q_step
            self.draw_plot(f"Iterasi IK: {i}/{len(history)-1}")
            plt.pause(0.01) 
            
        self.draw_plot("Selesai! Klik titik target yang baru.")
        self.is_moving = False

    def draw_plot(self, title_text):
        self.ax.clear()
        self.ax.set_xlim(-self.max_reach, self.max_reach)
        self.ax.set_ylim(-self.max_reach, self.max_reach)
        self.ax.set_aspect('equal')
        self.ax.grid(True, linestyle='--', alpha=0.6)
        self.ax.set_title(title_text, pad=15, fontweight='bold')
        
        q1, q2, q3 = self.robot.q
        x0, y0 = 0, 0
        x1, y1 = self.robot.l1 * np.cos(q1), self.robot.l1 * np.sin(q1)
        x2, y2 = x1 + self.robot.l2 * np.cos(q1 + q2), y1 + self.robot.l2 * np.sin(q1 + q2)
        x3, y3 = self.robot.forward_kinematics(self.robot.q)[:2]
        
        # ---------------------------------------------------------
        # FITUR BARU: PANEL TEKS REAL-TIME UNTUK SUDUT (DERAJAT)
        # ---------------------------------------------------------
        info_text = (
            f"Status Sendi (Real-Time):\n"
            f"----------------------\n"
            f"θ1 (Bahu)  : {np.rad2deg(q1):6.1f}°\n"
            f"θ2 (Siku)  : {np.rad2deg(q2):6.1f}°\n"
            f"θ3 (Ujung) : {np.rad2deg(q3):6.1f}°"
        )
        
        # Tempatkan teks di pojok kiri atas (x=0.03, y=0.97 relatif terhadap ukuran kotak plot)
        self.ax.text(0.03, 0.97, info_text, transform=self.ax.transAxes, 
                     fontsize=10, verticalalignment='top', family='monospace',
                     bbox=dict(boxstyle='round,pad=0.5', facecolor='#f0f8ff', edgecolor='#4682b4', alpha=0.9))
        # ---------------------------------------------------------

        # Gambar target
        self.ax.plot(self.target[0], self.target[1], 'r*', markersize=12, label='Target Posisi')
        
        # Gambar Lengan Robot
        self.ax.plot([x0, x1, x2, x3], [y0, y1, y2, y3], '-o', lw=5, markersize=8, color='#2c3e50', label='Lengan Robot')
        
        self.ax.legend(loc='upper right')
        self.fig.canvas.draw()

if __name__ == "__main__":
    robot = Planar3DOF(l1=100.0, l2=80.0, l3=60.0)
    app = InteractiveVisualizer(robot)