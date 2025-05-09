import subprocess
import time
import os
import signal
import yaml
import shutil

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

def copy_simulation_data(filename, dest_dir):
    latest_log_path = sim_params.get_latest_log(sim_params.raw_logging_directory)
    dest_path = os.path.join(dest_dir, filename)
    shutil.copy(latest_log_path, dest_path)
    print(f"Copied simulation data to: {dest_path}")

def main():
    test_name = "body_mass_variations_3"
    workspace_path = "/workspace"  # Set your workspace root here

    def define_leg_config(index):
        rule = 1.013 + index*.075 * (-1) ** index
        leg_config = {
                'FR': {
                    'm2': rule
                },
                'FL': {
                    'm2': rule
                },
                'RR': {
                    'm2': rule
                },
                'RL': {
                    'm2': rule
                },
            }
        return leg_config
    
    def define_base_config(index):
        rule = 5.660 + index * .5 * (-1) ** index
        body_config = {
                'base': {
                    'm': rule
                },
            }
        return body_config
    
    param_config_list = []
    for i in range(10):
        param_config_list.append(define_base_config(i))


    results_root_dir = f"{workspace_path}/src/simulation/data_logs/{test_name}"
    os.makedirs(results_root_dir, exist_ok=True)
    
    for i, value in enumerate(param_config_list):
        quadruped_params = param_config_list[i]

        # 1. Update the YAML file used by the simulation
        base_config_path = f'{workspace_path}/src/quadruped_description/config/params_unitree_a1.yaml'
        active_config_path = f'{workspace_path}/src/quadruped_description/config/params.yaml'
        update_yaml_file(
            existing_yaml_path=base_config_path,
            new_data_dict=quadruped_params,
            output_path=active_config_path
        )

        # 2. Rebuild
        rebuild_workspace(workspace_path)

        # 3. Run simulation
        print(f"--- Running simulation with quadruped configuration #{i} ---")
        crashed = run_simulation_until_logger_crashes(
            "quadruped_bringup", "quadruped_sim.launch.py"
        )
        print(f"Simulation #{i+1} stopped due to logger crash.")
        time.sleep(2)

        # 4. Create subdirectory for this simulation
        sim_name = f"test_{i}"
        sim_dir = os.path.join(results_root_dir, sim_name)
        os.makedirs(sim_dir, exist_ok=True)

        # 5. Copy updated YAML to that simulation folder
        shutil.copy(src=active_config_path, dst=os.path.join(sim_dir, "params.yaml"))

        # 6. Copy log data to that simulation folder
        copy_simulation_data(filename="raw_data.csv", dest_dir=sim_dir)

if __name__ == "__main__":
    main()
