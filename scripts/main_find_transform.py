import numpy as np
import pandas as pd
import argparse
from scipy.spatial.transform import Rotation as R

class Transform:
    def __init__(self, tf):
        self.origin = tf[:3]
        self.rotation = R.from_quat(tf[3:])

    def get(self):
        result = np.eye(4)
        result[:3, :3] = self.rotation.as_matrix()
        result[:3, 3] = self.origin
        return result

    def inverse(self):
        mat = self.get()
        return np.linalg.pinv(mat)

    @classmethod
    def from_tf_mat(cls, mat):
        rot = mat[:3, :3]
        vec = mat[:3, 3]
        q = R.from_matrix(rot)
        tf = np.hstack((vec, q.as_quat()))
        return cls(tf)

    def __repr__(self):
        msg = ", ".join(map(lambda x: "{:.3f}".format(x), self.origin)) + ", "
        msg += ", ".join(map(lambda x: "{:.3f}".format(x), self.rotation.as_quat()))
        return msg

def main(args):
    logitec_csv = pd.read_csv(args.logitec)
    nexigo_csv = pd.read_csv(args.nexigo)
    logitec_to_map = np.array(logitec_csv.mean(axis=0))
    nexigo_to_map = np.array(nexigo_csv.mean(axis=0))

    L_t_M = Transform(logitec_to_map).get()
    N_t_M = Transform(nexigo_to_map).get()

    # Compute the relative transformation T_logitech_to_nexigo
    #  L_t_N = np.dot(L_t_M, np.linalg.inv(N_t_M))
    L_t_N = L_t_M @ np.linalg.inv(N_t_M)
    N_t_L = N_t_M @ np.linalg.inv(L_t_M)
    logitech_to_nexigo = Transform.from_tf_mat(L_t_N)
    nexigo_to_logitec = Transform.from_tf_mat(N_t_L)
    print("logitec to map transform")
    print(", ".join(map(lambda x:"{:.3f}".format(x), logitec_to_map)))
    print("logitec to nexigo transform")
    print(logitech_to_nexigo)

    print("nexigo to map transform")
    print(", ".join(map(lambda x:"{:.3f}".format(x), nexigo_to_map)))
    print("nexigo to logitec transform")
    print(nexigo_to_logitec)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--logitec", type=str, default="tf_logitec_cam_tag36h11:7.csv")
    parser.add_argument("--nexigo", type=str, default="tf_nexigo_cam_tag36h11:7.csv")
    args = parser.parse_args()
    main(args)