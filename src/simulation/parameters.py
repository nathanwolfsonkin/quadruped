from datetime import datetime
import re
import os
from ament_index_python.packages import get_package_share_directory

# Time related simulation parameters (All in seconds)
start_walking_time = 3.0 # Time that the quadruped starts walking
start_recording_time = 10.0
stop_recording_time = 20.0
shutdown_time = 25.0

# Initilization related simulation parameters
init_time = 5.0 # Time that the initialization force stops being appllied
init_force = 15.0 # Newtons

# Gait generation parameters
gait_command_publishing_rate = .001 # seconds

# Filter Parameters
pos_filt_cutoff_freq = 20 # Hz
vel_filt_cutoff_freq = 20 # Hz
tor_filt_cutoff_freq = 20 # Hz

# Logging related parameters
def get_latest_log(directory):
    pattern = re.compile(r"joint_states_(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})\.csv")
    log_files = [file for file in os.listdir(directory) if pattern.match(file)]
    
    if not log_files:
        raise LookupError("No logs currently found in " + directory)
    
    return directory + max(log_files, key=lambda file: datetime.strptime(pattern.match(file).group(1), "%Y-%m-%d_%H-%M-%S"))

# Synthetic Logging interval is limited because 
raw_logging_interval = .005 # Seconds

raw_logging_directory = "/workspace/src/simulation/data_logs/raw_data_log/"
postprocess_filtered_logging_directory = "/workspace/src/simulation/data_logs/postprocess_filtered_data_log/"

# Quadruped params filepath
quadruped_params_file = os.path.join(get_package_share_directory('quadruped_description'), 'config', 'params.yaml')
analytical_gait_params_file = os.path.join(get_package_share_directory('gait_generation'), 'gait_trajectory', 'gait_sin_waves.yaml')


# Parameters no longer using in active nodes

# No Longer logging desired, instead calculating directly based on analytical functions 
# synthetic_logging_interval = .0125
# desired_logging_directory = "/workspace/src/simulation/data_logs/desired_data_log/"

# No longer using realtime filtering
# filt_sampling_rate = 1000 # Hz
# realtime_filtered_logging_directory = "/workspace/src/simulation/data_logs/filtered_data_log/"