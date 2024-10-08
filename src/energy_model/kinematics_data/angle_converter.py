import csv
import numpy as np

def angle_calc(x, y):
    # x = [hip, knee, foot]
    # y = [hip, knee, foot]
    t1 = np.arctan2(np.abs(y[2]-y[1])/np.abs(x[2]-x[1]))
    alpha = np.arctan2(np.abs(y[1]-y[0])/np.abs(x[1]-x[0]))
    t2 = t1 + alpha
    return t1, t2

def get_positions(file_path):
    with open(file_path, mode='r', newline='') as csvfile:        
        csv_reader = csv.reader(csvfile)

        # Optionally, skip the header if your CSV has one
        header = next(csv_reader)
        x = []
        y = []

        # Read each row in the CSV file
        for row in csv_reader:
            x.append(row[0])
            y.append(row[1])
        
        return x, y

def main():
    # Specify joint csv names
    joint_names = ["front_hip.csv",
                   "front_knee.csv",
                   "front_foot.csv",
                   "rear_hip.csv",
                   "rear_knee.csv",
                   "rear_foot.csv"]
    
    front_joints = joint_names[:3]
    rear_joints = joint_names[4:]
    
    # Specify filepath to joint csv
    filepath = "/workspace/src/energy_model/kinematics_data/unitree_a1/"

    num_joints = len(joint_names)
    xlist = [[] for _ in range(num_joints)]
    ylist = [[] for _ in range(num_joints)]

    for i, joint in enumerate(joint_names):
        x, y = get_positions(filepath + joint)
        xlist[i] = x
        ylist[i] = y

    leg_angles = [[],[]]
    for i in range(2):
        for j in range(len(xlist[0])):
            # data is sent in form [hip knee foot]
            x = [xlist[0][j], xlist[1][j], xlist[2][j]]
            y = [ylist[0][j], ylist[1][j], ylist[2][j]]
            t1, t2 = angle_calc(x,y)
            # Leg, i, at time instace, j, has angles [t1, t2] 
            leg_angles[i][j] = [t1, t2]

    


if __name__ == "__main__":
    main()