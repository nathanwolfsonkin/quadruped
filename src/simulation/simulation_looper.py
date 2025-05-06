import subprocess
import time
import os
import signal
import yaml

import simulation.parameters as sim_params

def rebuild_workspace(workspace_path):
    print("Rebuilding workspace...")
    build_command = f"source /opt/ros/jazzy/setup.bash && cd {workspace_path} && colcon build"
    result = subprocess.run(["bash", "-c", build_command], capture_output=True, text=True)

    print(result.stdout)
    if result.returncode != 0:
        print(result.stderr)
        raise RuntimeError("colcon build failed.")

def run_simulation_until_logger_crashes(package, launch_file):
    ros_setup = "/opt/ros/jazzy/setup.bash"
    command = f"source {ros_setup} && ros2 launch {package} {launch_file}"

    process = subprocess.Popen(
        ["bash", "-c", command],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        executable="/bin/bash",
        preexec_fn=os.setsid  # Start new process group
    )

    try:
        for line in process.stdout:
            print(line, end='')

            if "raw_data_logger" in line and "process has died" in line:
                print("Detected crash of raw_data_logger. Sending SIGINT...")
                os.killpg(os.getpgid(process.pid), signal.SIGINT)

                try:
                    # Wait for clean shutdown (up to 10 seconds)
                    process.wait(timeout=10)
                    print("Process exited cleanly.")
                except subprocess.TimeoutExpired:
                    print("Process didn't exit in time, sending SIGKILL...")
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.wait()  # Ensure process is reaped

                return True

    except KeyboardInterrupt:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait()

    return False

def recursive_update(original, updates):
    for key, value in updates.items():
        if isinstance(value, dict) and key in original:
            recursive_update(original[key], value)
        else:
            original[key] = value

def update_yaml_file(existing_yaml_path, new_data_dict, output_path=None):
    with open(existing_yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    recursive_update(data, new_data_dict)

    if output_path is None:
        output_path = existing_yaml_path

    with open(output_path, 'w') as f:
        yaml.dump(data, f, sort_keys=False)

def main():
    workspace_path = "/workspace"  # Set your workspace root here

    for i, index in enumerate([5.660, 5.760, 5.860, 5.960]):
        new_quad_params = {
            'base': {
                'm': index
            },
            # other leg updates...
        }

        update_yaml_file(
            existing_yaml_path=f'{workspace_path}/src/quadruped_description/config/params_unitree_a1.yaml',
            new_data_dict=new_quad_params,
            output_path=f'{workspace_path}/src/quadruped_description/config/params.yaml'
        )

        rebuild_workspace(workspace_path)

        print(f"--- Running simulation with mass = {index} ---")
        crashed = run_simulation_until_logger_crashes(
            "quadruped_bringup", "quadruped_sim.launch.py"
        )

        print(f"Simulation #{i+1} stopped due to logger crash.")
        time.sleep(2)

if __name__ == "__main__":
    main()
