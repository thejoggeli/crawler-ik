
from MathUtils import MathUtils
import numpy as np

class Kinematics:

    def __init__(self):
        
        # hips offsets
        diagonal = 0.155*0.5
        dx = diagonal / np.sqrt(2)
        dy = diagonal / np.sqrt(2)

        # hips translation vectors
        self.hips_translation = np.array([
            [+dx, +dy, 0.0, 1.0],
            [-dx, +dy, 0.0, 1.0],
            [-dx, -dy, 0.0, 1.0],
            [+dx, -dy, 0.0, 1.0],
        ], dtype=np.float64)

        # hips rotations around y
        self.hips_rotation_z = np.deg2rad(np.array([45.0, 135.0, -135.0, -45.0], dtype=np.float64))

        # hips transformation matrices
        self.hips_matrix = np.zeros(shape=(self.hips_translation.shape[0],4,4), dtype=np.float64)
        self.hips_matrix_inv = np.zeros(shape=(self.hips_translation.shape[0],4,4), dtype=np.float64)
        for i in range(self.hips_translation.shape[0]):
            Ti = MathUtils.matrix4x4()
            Ri = MathUtils.matrix4x4()
            MathUtils.matrix_translation(Ti, self.hips_translation[i])
            MathUtils.matrix_rotation_z(Ri, self.hips_rotation_z[i])
            self.hips_matrix[i] = Ti@Ri
            self.hips_matrix_inv[i] = np.linalg.inv(self.hips_matrix[i])

        # phi min and max angles
        self.phi_min = np.deg2rad(-50.0)
        self.phi_max = np.deg2rad(50.0)

        # limb lengths
        self.lengths = np.array([0.07, 0.07, 0.07, 0.096], dtype=np.float64)

        # joint angle limits
        self.limits_min = np.deg2rad([-90.0, -90.0, -90.0, -90.0], dtype=np.float64)
        self.limits_max = np.deg2rad([+90.0, +90.0, +90.0, +90.0], dtype=np.float64)


    def ik(self, Q, phi, angles_out):

        L0 = self.lengths[0]
        L1 = self.lengths[1]
        L2 = self.lengths[2]
        L3 = self.lengths[3]

        xy = np.sqrt(Q[0] * Q[0] + Q[1] * Q[1]) - L0 - L3 * np.sin(phi)
        z = Q[2] + L3 * np.cos(phi)

        v = (xy * xy + z * z - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)

        if(v < -1.0 or v > 1.0):
            return False
        
        a2 = -np.arccos(v)
        a2 = MathUtils.angle_360_to_180(a2)
        if(a2 < self.limits_min[2] or a2 > self.limits_max[2]):
            return False

        a1 = np.arctan2(z, xy) - np.arctan2(L2 * np.sin(a2), L1 + L2 * np.cos(a2))
        a1 = MathUtils.angle_360_to_180(a1)
        if(a1 < self.limits_min[1] or a1 > self.limits_max[1]):
            return False

        a3 = phi - a1 - a2 - np.pi * 0.5
        a3 = MathUtils.angle_360_to_180(a3)
        if(a3 < self.limits_min[3] or a3 > self.limits_max[3]):
            return False

        a0 = np.arctan2(Q[1], Q[0])
        a0 = MathUtils.angle_360_to_180(a0)
        if(a0 < self.limits_min[0] or a0 > self.limits_max[0]):
            return False
        
        angles_out[0] = a0
        angles_out[1] = a1
        angles_out[2] = a2
        angles_out[3] = a3
        
        return True


    def fk(self, angles, pos_out):

        pos_out[0][0] = self.lengths[0]
        pos_out[0][1] = 0.0
        pos_out[0][2] = 0.0

        r_x = np.cos(angles[0])
        r_y = np.sin(angles[0])

        angle_cum = 0.0
        for i in range(1,4):
            angle_cum += angles[i]
            d_xy = np.cos(angle_cum) * self.lengths[i]
            pos_out[i][0] = pos_out[i-1][0] + d_xy * r_x
            pos_out[i][1] = pos_out[i-1][1] + d_xy * r_y
            pos_out[i][2] = pos_out[i-1][2] + np.sin(angle_cum) * self.lengths[i]

    def loss(self, phi_target, phi_actual, phi_weight=1.0, angles_old=None, angles_new=None, angles_weight=0.5):

        loss = 0.0

        if(phi_target is not None):
            phi_delta = phi_target - phi_actual
            loss += (np.square(phi_delta) + np.abs(phi_delta)) * phi_weight

        if(angles_old is not None):
            angles_delta = angles_old-angles_new
            loss += np.amax(np.square(angles_delta) + np.abs(angles_delta)) * angles_weight

        return loss

    
    def search(self, Q, phi_target, angles_out, angles_old=None, num_phi_vals=255):

        phi_vals = np.linspace(self.phi_min, self.phi_max, num_phi_vals)
        best_loss = np.inf
        best_loss_idx = -1
        angles_ik = np.zeros(4, dtype=np.float64)

        for idx, phi in enumerate(phi_vals):

            result = self.ik(Q, phi, angles_ik)

            if(result):
                loss_val = self.loss(phi_target=phi_target, phi_actual=phi, angles_old=angles_old, angles_new=angles_ik)
                if(loss_val < best_loss):
                    best_loss_idx = idx
                    best_loss = loss_val

        if(best_loss_idx == -1):
            has_result = False
            best_phi = np.nan

        else:
            has_result = True
            best_phi = phi_vals[best_loss_idx]
            self.ik(Q, best_phi, angles_out)

        return has_result, best_phi

