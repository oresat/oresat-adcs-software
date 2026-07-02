"""Test Quaternion Identities"""

import quaternion
import numpy as np
import timeit

class TestQuaternion:
    def diff_test(self, a_quat, b_quat):
        return np.sum(np.abs(a_quat - b_quat)) <= 1e-12

    def test_rotation(self):
        # scalar first
        
        quat_zero = np.array([1, 0, 0, 0])

        quat_p90x = np.array([np.sqrt(2)/2, np.sqrt(2)/2, 0, 0])
        quat_p90y = np.array([np.sqrt(2)/2, 0, np.sqrt(2)/2, 0])
        quat_p90z = np.array([np.sqrt(2)/2, 0, 0, np.sqrt(2)/2])
        
        quat_n90x = np.array([np.sqrt(2)/2, -np.sqrt(2)/2, 0, 0])
        quat_n90y = np.array([np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0])
        quat_n90z = np.array([np.sqrt(2)/2, 0, 0, -np.sqrt(2)/2])

        vect_px = np.array([1, 0, 0])
        vect_py = np.array([0, 1, 0])
        vect_pz = np.array([0, 0, 1])
        
        vect_nx = np.array([-1, 0, 0])
        vect_ny = np.array([0, -1, 0])
        vect_nz = np.array([0, 0, -1])


        assert self.diff_test(quaternion.conjugate(quat_zero), quat_zero)
        
        assert self.diff_test(quaternion.conjugate(quat_p90x), quat_n90x)
        assert self.diff_test(quaternion.conjugate(quat_p90y), quat_n90y)
        assert self.diff_test(quaternion.conjugate(quat_p90z), quat_n90z)
        
        assert self.diff_test(quaternion.conjugate(quat_n90x), quat_p90x)
        assert self.diff_test(quaternion.conjugate(quat_n90y), quat_p90y)
        assert self.diff_test(quaternion.conjugate(quat_n90z), quat_p90z)



        print("hamiltonian")
        assert self.diff_test(quaternion.hamiltonian(quat_zero, quat_zero), quat_zero)
        
        assert self.diff_test(quaternion.hamiltonian(quat_zero, quat_p90x), quat_p90x)
        assert self.diff_test(quaternion.hamiltonian(quat_zero, quat_p90y), quat_p90y)
        assert self.diff_test(quaternion.hamiltonian(quat_zero, quat_p90z), quat_p90z)
        
        assert self.diff_test(quaternion.hamiltonian(quat_zero, quat_n90x), quat_n90x)
        assert self.diff_test(quaternion.hamiltonian(quat_zero, quat_n90y), quat_n90y)
        assert self.diff_test(quaternion.hamiltonian(quat_zero, quat_n90z), quat_n90z)


        print("shuster")
        assert self.diff_test(quaternion.shuster(quat_zero, quat_zero), quat_zero)
        
        assert self.diff_test(quaternion.shuster(quat_zero, quat_p90x), quat_p90x)
        assert self.diff_test(quaternion.shuster(quat_zero, quat_p90y), quat_p90y)
        assert self.diff_test(quaternion.shuster(quat_zero, quat_p90z), quat_p90z)
        
        assert self.diff_test(quaternion.shuster(quat_zero, quat_n90x), quat_n90x)
        assert self.diff_test(quaternion.shuster(quat_zero, quat_n90y), quat_n90y)
        assert self.diff_test(quaternion.shuster(quat_zero, quat_n90z), quat_n90z)



        print("hamiltonian sandwiches")
        assert self.diff_test(quaternion.ham_sandwich(vect_px, quat_p90x), vect_px)
        assert self.diff_test(quaternion.ham_sandwich(vect_px, quat_p90y), vect_nz)
        assert self.diff_test(quaternion.ham_sandwich(vect_px, quat_p90z), vect_py)
        
        assert self.diff_test(quaternion.ham_sandwich(vect_nx, quat_p90x), vect_nx)
        assert self.diff_test(quaternion.ham_sandwich(vect_nx, quat_p90y), vect_pz)
        assert self.diff_test(quaternion.ham_sandwich(vect_nx, quat_p90z), vect_ny)
        
        assert self.diff_test(quaternion.ham_sandwich(vect_py, quat_p90x), vect_pz)
        assert self.diff_test(quaternion.ham_sandwich(vect_py, quat_p90y), vect_py)
        assert self.diff_test(quaternion.ham_sandwich(vect_py, quat_p90z), vect_nx)
        
        assert self.diff_test(quaternion.ham_sandwich(vect_ny, quat_p90x), vect_nz)
        assert self.diff_test(quaternion.ham_sandwich(vect_ny, quat_p90y), vect_ny)
        assert self.diff_test(quaternion.ham_sandwich(vect_ny, quat_p90z), vect_px)
 

        print("shuster sandwiches")
        assert self.diff_test(quaternion.shu_sandwich(vect_px, quat_p90x), vect_px)
        assert self.diff_test(quaternion.shu_sandwich(vect_px, quat_p90y), vect_nz)
        assert self.diff_test(quaternion.shu_sandwich(vect_px, quat_p90z), vect_py)
        
        assert self.diff_test(quaternion.shu_sandwich(vect_nx, quat_p90x), vect_nx)
        assert self.diff_test(quaternion.shu_sandwich(vect_nx, quat_p90y), vect_pz)
        assert self.diff_test(quaternion.shu_sandwich(vect_nx, quat_p90z), vect_ny)
        
        assert self.diff_test(quaternion.shu_sandwich(vect_py, quat_p90x), vect_pz)
        assert self.diff_test(quaternion.shu_sandwich(vect_py, quat_p90y), vect_py)
        assert self.diff_test(quaternion.shu_sandwich(vect_py, quat_p90z), vect_nx)
        
        assert self.diff_test(quaternion.shu_sandwich(vect_ny, quat_p90x), vect_nz)
        assert self.diff_test(quaternion.shu_sandwich(vect_ny, quat_p90y), vect_ny)
        assert self.diff_test(quaternion.shu_sandwich(vect_ny, quat_p90z), vect_px)
 




if __name__ == "__main__":
    test = QuaternionTest()
    test.rotation_test()
