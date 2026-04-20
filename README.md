# 🤖 Praktikum Kinematika: Jacobian-Based IK untuk Redundant Manipulator

Repositori ini berisi modul praktikum **Inverse Kinematics (IK)** menggunakan metode numerik **Jacobian Pseudo-inverse** untuk lengan robot planar 3-DOF. 

Berbeda dengan sistem robot standar, modul ini mendemonstrasikan konsep **Robot Redundan**. Karena robot memiliki 3 sendi (3-DOF) namun targetnya hanya mempedulikan 2 sumbu koordinat (X dan Y), robot memiliki derajat kebebasan ekstra. Ini memungkinkan robot bergerak luwes mencari posisi target tanpa dibatasi oleh orientasi ujung lengannya.

---

## 📸 Demo Visualisasi

![Demo Pergerakan Lengan 3-DOF](img/result.gif)

> **Keterangan:** Lengan robot 3-DOF secara dinamis menghitung matriks Jacobian untuk mengejar titik target (bintang merah) yang diklik oleh pengguna secara *real-time*.

---

## 🧮 Landasan Matematika

### 1. Forward Kinematics (FK)
Posisi ujung lengan $(x, y)$ dihitung berdasarkan sudut ketiga sendi $(q_1, q_2, q_3)$:
* $x = L_1 \cos(q_1) + L_2 \cos(q_1 + q_2) + L_3 \cos(q_1 + q_2 + q_3)$
* $y = L_1 \sin(q_1) + L_2 \sin(q_1 + q_2) + L_3 \sin(q_1 + q_2 + q_3)$

### 2. Matriks Jacobian Redundan (2x3)
Karena orientasi diabaikan, matriks Jacobian direduksi menjadi matriks $2 \times 3$. Matriks ini memetakan kecepatan dari 3 sendi input ke kecepatan 2 sumbu output:
$$J = \begin{bmatrix} \frac{\partial x}{\partial q_1} & \frac{\partial x}{\partial q_2} & \frac{\partial x}{\partial q_3} \\ \frac{\partial y}{\partial q_1} & \frac{\partial y}{\partial q_2} & \frac{\partial y}{\partial q_3} \end{bmatrix}$$

### 3. Iterasi Inverse Kinematics
Pembaruan sudut dilakukan secara iteratif menggunakan **Moore-Penrose Pseudo-inverse** ($J^{\dagger}$). Langkah kecil ($\alpha$) digunakan untuk mencegah *overshoot* akibat pergerakan lengan yang bersifat non-linear:
$$\Delta q = \alpha \cdot J^{\dagger} \cdot (X_{target} - X_{current})$$

---

## 🚀 Cara Menjalankan Simulasi

**1. Persiapan Kebutuhan**
Pastikan Anda menggunakan Python 3.x dan telah menginstal *library* komputasi numerik dan plotting:
```bash
pip install numpy matplotlib
