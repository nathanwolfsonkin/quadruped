import csv
import numpy as np

def angle_calc(x, y):
    # x = [hip, knee, foot]
    # y = [hip, knee, foot]
    t1 = (np.arctan2((x[1]-x[0]),(y[1]-y[0]))).item()
    alpha = (np.arctan2((x[2]-x[1]),(y[2]-y[1]))).item()
    t2 = - t1 + alpha
    return t1, t2

def get_positions(file_path):
    with open(file_path, mode='r', newline='') as csvfile:        
        csv_reader = csv.reader(csvfile)

        # Optionally, skip the header if your CSV has one
        next(csv_reader)
        x = []
        y = []

        # Read each row in the CSV file
        for row in csv_reader:
            x.append(row[0])
            y.append(row[1])

        for i in range(len(x)):
            x[i] = float(x[i])
            y[i] = float(y[i])
        
        return x, y

def get_angle_lists():
    # Specify joint csv names
    joint_names = ["front_hip.csv",
                   "front_knee.csv",
                   "front_foot.csv",
                   "rear_hip.csv",
                   "rear_knee.csv",
                   "rear_foot.csv"]
    
    # Specify filepath to joint csv
    filepath = "/workspace/src/energy_model/kinematics_data/unitree_a1/"

    xlist = []
    ylist = []
    for joint in joint_names:
        x, y = get_positions(filepath + joint)
        xlist.append(x)
        ylist.append(y)


    leg_angles = [[[],[]],[[],[]]]
    for i in range(len(leg_angles)):
        for j in range(len(xlist[0])):
            # data is sent in form [hip knee foot]
            for k in range(len(xlist)):
                xlist[k][j] = float(xlist[k][j])
                ylist[k][j] = float(ylist[k][j])
            
            if i == 0:
                x = [xlist[0][j], xlist[1][j], xlist[2][j]]
                y = [ylist[0][j], ylist[1][j], ylist[2][j]]
            elif i == 1:
                x = [xlist[3][j], xlist[4][j], xlist[5][j]]
                y = [ylist[3][j], ylist[4][j], ylist[5][j]]
            hip, knee = angle_calc(x,y)

            leg_angles[i][0].append(hip)
            leg_angles[i][1].append(knee)
    
    return leg_angles

def moving_filter(x, y): #using this for velocity data
    new_x = []
    new_y = []
    for i in range(len(x)):
        if i > 0 and i < len(x)-2:
            x_temp = np.mean([x[i-1],x[i],x[i+1]])
            y_temp = np.mean([y[i-1],y[i],y[i+1]])
        else:
            x_temp = x[i]
            y_temp = y[i]
        new_x.append(x_temp)
        new_y.append(y_temp)
    return new_x, new_y

def main():
    moving_filter()

if __name__ == "__main__":
    main()