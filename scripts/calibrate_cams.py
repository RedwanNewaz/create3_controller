import subprocess
import os
import re
import signal
from time import sleep, time
import argparse
import numpy as np
import yaml
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
    
    def tolist(self):
        origin_data = [float(x) for x in self.origin]
        rotation_data = [float(x) for x in self.rotation.as_quat()]
        return origin_data + rotation_data

def start_apriltag(node_name):
    assert node_name in ["logitec", "nexigo"]
    # The os.setsid() is passed in the argument preexec_fn so
    # it's run after the fork() and before  exec() to run the shell.
    
    cmd = "ros2 launch create3_controller create3_apriltag.launch.py node_name:={}"
    cmd = cmd.format(node_name)

    pro = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                        shell=True, preexec_fn=os.setsid) 

    print("process started pid = ", pro.pid)
    return pro


class DFA:
    def __init__(self):
        self.state = 0
        self.accept = 2
        self.data = []

    def __call__(self, data):
        N = len(data)
        if self.state == 0 and N == 3:
            self.state = 1
            self.data.append(data)
        if self.state == 1 and N == 4:
            self.state = 2
            self.data.append(data)

    def get(self):
        assert len(self.data) == 2
        return self.data[0] + self.data[1]
    def reset(self):
        self.data = []
    def isAccepted(self):
        if self.state == 2:
            self.state = 0
            return True
        return False


def record_transformation(command):
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
        shell=True
    )
    pattern = r'\[([-0-9., ]+)\]'
    dfa = DFA()
    while True:
        output = process.stdout.readline()
        if output == '' and process.poll() is not None:
            break
        if output:
            matches = re.findall(pattern, output)
            # Check if there are any matches
            if matches:
                # Get the first match (assuming there's only one match in the text)
                matched_string = matches[0]

                # Split the matched string into a list of strings
                number_strings = matched_string.split(',')

                # Convert the list of strings to a set of float numbers
                number_list = list(float(num) for num in number_strings)

                # Print the set of float numbers
                dfa(number_list)
                if dfa.isAccepted():
                    yield dfa.get()
                    dfa.reset()

def get_transformation(frm, to, samples):
    data = []
    command = "ros2 run tf2_ros tf2_echo {} {}".format(frm, to)
    print(command)
    for i, val in enumerate(record_transformation(command)):

        print(i + 1 , val)

        data.append(val)

        # check termination
        if i >= samples - 1:
            break
    return data

def calibrate_map_coords(nexigo_data, logitec_data):

    logitec_to_map = np.array(logitec_data).mean(axis=0)
    nexigo_to_map = np.array(nexigo_data).mean(axis=0)

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

    logitec_param = {"logitec_cam": {"tf":{
        "map": [float(x) for x in logitec_to_map],
        "nexigo": logitech_to_nexigo.tolist()
        }}}
    
    nexigo_param = {"nexigo_cam": {"tf":{
        "map": [float(x) for x in nexigo_to_map],
        "logitec": nexigo_to_logitec.tolist()
        }}}

    return (nexigo_param, logitec_param)

def get_tf_cam_to_map(camName, tagID):
    print(f"start {camName} node")
    pro = start_apriltag(camName)

    try:
        cam_to_map = get_transformation(f"{camName}_cam", tagID, 15)
    except:
        pass

    sleep(2)
    print(f"killing {camName} node")
    os.killpg(os.getpgid(pro.pid), signal.SIGTERM)  # Send the signal to all the process groups
    print(f"{camName} node terminated")
    return cam_to_map

def recursive_update(yaml_dict, update_dict):
    for key, value in update_dict.items():
        if key in yaml_dict:
            if isinstance(value, dict) and isinstance(yaml_dict[key], dict):
                recursive_update(yaml_dict[key], value)
            else:
                yaml_dict[key] = value
        else:
            yaml_dict[key] = value

def main(args):

    N_T_M = get_tf_cam_to_map("nexigo", args.frame_id)
    L_T_M = get_tf_cam_to_map("logitec", args.frame_id)

    yaml_file_path = "../config/cfg_joint_cameras.yaml"

    with open(yaml_file_path) as file:
        ydata = yaml.safe_load(file)

    for update in calibrate_map_coords(N_T_M, L_T_M):
        recursive_update(ydata, update)
    


    with open(yaml_file_path, 'w') as yaml_file:
        yaml.dump(ydata, yaml_file, default_flow_style=False)
    
    print("[Updated] ", yaml_file_path)



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--frame-id", type=str, default="tag36h11:7", choices=['tag36h11:7', 'tag36h11:32'])
    args = parser.parse_args()
    main(args)