import subprocess
import time
import os
import signal

def run_simulation_until_logger_crashes(package, launch_file):
    ros_setup = "/opt/ros/jazzy/setup.bash"
    command = f"source {ros_setup} && ros2 launch {package} {launch_file}"

    # Launch the process in a new process group
    process = subprocess.Popen(
        ["bash", "-c", command],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        executable="/bin/bash",
        preexec_fn=os.setsid  # Start the process in a new session
    )

    try:
        for line in process.stdout:
            print(line, end='')

            # Check if raw_data_logger has crashed
            if "raw_data_logger" in line and "process has died" in line:
                print("Detected crash of raw_data_logger, sending SIGINT to process group.")
                os.killpg(os.getpgid(process.pid), signal.SIGINT)  # Like Ctrl+C
                print('killed process')
                process.wait()
                print("wait finished")
                return True

    except KeyboardInterrupt:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait()

    return False

def configure_and_run_simulations():
    simulation_runs = 1

    for i in range(simulation_runs):
        print(f"--- Running simulation #{i+1} ---")
        crashed = run_simulation_until_logger_crashes(
            "quadruped_bringup", "quadruped_sim.launch.py"
        )

        print(f"Simulation #{i+1} stopped due to logger crash.")
        time.sleep(2)

if __name__ == "__main__":
    configure_and_run_simulations()
