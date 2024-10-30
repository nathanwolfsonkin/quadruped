import numpy as np

from energy_model.kinematics_data.angle_converter import get_positions

def get_scaling_factor():
    filepath = "/workspace/src/energy_model/kinematics_data/unitree_a1/front_hip.csv"
    front_hip_x = get_positions(filepath)

    filepath = "/workspace/src/energy_model/kinematics_data/unitree_a1/rear_hip.csv"
    rear_hip_x = get_positions(filepath)

    hip_dist_cv_units = abs(front_hip_x[0][0] - rear_hip_x[0][0])
    hip_dist_mm = 450.
    
    return hip_dist_mm/hip_dist_cv_units

def get_frame_time():
    total_time = 4. # [s] video time
    total_time /= 4 # account for video running at .25 speed
    total_time *= 2 # account for skipping every other frame

    # Get total number of frames from csv with full length
    total_frames = len(get_positions("/workspace/src/energy_model/kinematics_data/unitree_a1/front_hip.csv")[0])

    frame_time = (total_time/total_frames)

    return frame_time, total_time, total_frames

def main():
    filepath = "/workspace/src/energy_model/kinematics_data/unitree_a1/fix_point.csv"
    x , y = get_positions(filepath)

    scaling_factor = get_scaling_factor()

    frame_time = get_frame_time()

    inst_speed = []
    for i in range(len(x)-1):
        fp_dist = abs(x[i+1] - x[i]) #opencv units
        fp_dist *= scaling_factor # convert to mm
        fp_dist /= 1000           # convert to m
        
        speed_i = fp_dist/frame_time
        inst_speed.append(speed_i) # m/s

    print(inst_speed)
    print(np.mean(inst_speed)) # m/s


if __name__ == "__main__":
    main()