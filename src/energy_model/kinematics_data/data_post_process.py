import csv
import numpy as np

class VideoDataProcess:
    def __init__(self, unit="unitree_a1"):
        default_filepath = "/workspace/src/energy_model/kinematics_data/"
        
        self.filepath = default_filepath + unit + "/"
    
    def get_frame_data(self):
        total_time = 4. # [s] video time
        total_time /= 4 # account for video running at .25 speed
        total_time *= 2 # account for skipping every other frame

        # Get total number of frames from csv with full length
        total_frames = len(self.get_positions(self.filepath + "front_hip.csv")[0])

        frame_time = (total_time/total_frames)

        return frame_time, total_time, total_frames
    
    @staticmethod
    def angle_calc(x, y):
        # x = [hip, knee, foot]
        # y = [hip, knee, foot]
        t1 = (np.arctan2((x[1]-x[0]),(y[1]-y[0]))).item()
        alpha = (np.arctan2((x[2]-x[1]),(y[2]-y[1]))).item()
        t2 = - t1 + alpha
        return t1, t2
    
    def get_positions(self, file_path):
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

    def get_angle_lists(self):
        # Specify joint csv names
        joint_names = ["front_hip.csv",
                       "front_knee.csv",
                       "front_foot.csv",
                       "rear_hip.csv",
                       "rear_knee.csv",
                       "rear_foot.csv"]

        xlist = []
        ylist = []
        for joint in joint_names:
            x, y = self.get_positions(self.filepath + joint)
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
                hip, knee = self.angle_calc(x,y)

                leg_angles[i][0].append(hip)
                leg_angles[i][1].append(knee)
        
        return leg_angles
    
    # INPUT: list of hip, knee, and foot positions.
    # OUTPUT: list of t1, t2 
    def get_angle_list(self, hip, knee, foot):
        # Confirm list sizes are equal
        if len(hip[0]) != len(knee[0]) or len(knee[0]) != len(foot[0]):
            raise ValueError("The lists are not of equal size")
        
        # Calculate t1 and t2 at each frame
        t1 = []
        t2 = []
        for i, _ in enumerate(hip[0]):
            x = [hip[0][i], knee[0][i], foot[0][i]]
            y = [hip[1][i], knee[1][i], foot[1][i]]
            temp_t1, temp_t2  = self.angle_calc(x,y)
            t1.append(temp_t1)
            t2.append(temp_t2)

        return t1, t2

    def angle_list_to_timeseries(self):
        legs = self.get_angle_lists()
        
        # Get timing information
        frame_time, total_time, total_frames = self.get_frame_data()

        frames = len(legs[0][0])
        time = np.linspace(0, total_time, frames)

        timeseries = np.zeros([total_frames, 1+2*len(legs)*len(legs[0])])
        timeseries[:, 0] = np.linspace(0, total_time, total_frames)
        
        # convert theta_list to a timeseries
        index = 1
        for i, link in enumerate(legs):
            for j, angle_list in enumerate(link):
                # Create timeseries with time in the first column and the angle in the second column
                temp_timeseries = np.array([timeseries[:,0],angle_list]).reshape(150,2)
                timeseries[:,index] = temp_timeseries

    @staticmethod
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
    dataset = VideoDataProcess("unitree_a1")
    hip = dataset.get_positions(dataset.filepath + "front_hip.csv")
    knee = dataset.get_positions(dataset.filepath + "front_knee.csv")
    foot = dataset.get_positions(dataset.filepath + "front_foot.csv")
    angles = dataset.get_angle_list(hip,knee,foot)
    pass

if __name__ == "__main__":
    main()