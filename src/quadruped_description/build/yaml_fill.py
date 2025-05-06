import sys
import yaml

# Load YAML file
def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

# Save YAML file
def save_yaml(data, file_path):
    with open(file_path, 'w') as file:
        yaml.dump(data, file, default_flow_style=False)

# Compute inertia tensor elements
def compute_inertia(mass, length, radius, shape):
    if shape == "cylinder":
        Ixx = 0.5 * mass * radius ** 2
        Iyy = (1/12) * mass * (length ** 2 + 3 * radius ** 2)
        Izz = (1/12) * mass * (length ** 2 + 3 * radius ** 2)
    elif shape == "hemisphere":
        Ixx = (2/5) * mass * radius ** 2
        Iyy = (83/320) * mass * radius ** 2
        Izz = (83/320) * mass * radius ** 2
    elif shape == "rectangular_prism":
        Ixx = (1/12) * mass * (length[1] ** 2 + length[2] ** 2)
        Iyy = (1/12) * mass * (length[0] ** 2 + length[2] ** 2)
        Izz = (1/12) * mass * (length[0] ** 2 + length[1] ** 2)
    else:
        return {}
    
    return {"Ixx": Ixx, "Iyy": Iyy, "Izz": Izz}

# Process robot configuration
def process_robot_config(data):
    # Process base inertia
    if "base" in data:
        mass = data["base"]["m"]
        dimensions = [data["base"]["Lx"], data["base"]["Ly"], data["base"]["Lz"]]
        if "I" not in data["base"]:
            data["base"]["I"] = {}
        inertia_values = compute_inertia(mass, dimensions, None, "rectangular_prism")
        for component in ["Ixx", "Iyy", "Izz"]:
            if data["base"]["I"].get(component) is None:
                data["base"]["I"][component] = inertia_values[component]
    
    # Process leg inertia
    for leg in ["FR", "FL", "RL", "RR"]:
        rad = data[leg]["rad"]
        for i in range(1, 5):  # Loop through I1 to I4
            inertia_key = f"I{i}"
            mass_key = f"m{i}"
            
            if inertia_key in data[leg]:
                # Ensure the inertia dictionary exists
                if not isinstance(data[leg][inertia_key], dict):
                    data[leg][inertia_key] = {}
                
                if i < 4:
                    shape = "cylinder"
                    length_key = f"L{i}"
                    length = data[leg].get(length_key, 0)
                else:
                    shape = "hemisphere"
                    length = 0
                
                mass = data[leg][mass_key]
                inertia_values = compute_inertia(mass, length, rad, shape)
                
                for component in ["Ixx", "Iyy", "Izz"]:
                    if data[leg][inertia_key].get(component) is None:
                        data[leg][inertia_key][component] = inertia_values[component]
    
    return data

def main():
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    yaml_data = load_yaml(input_file)
    updated_data = process_robot_config(yaml_data)
    save_yaml(updated_data, output_file)

if __name__ == "__main__":
    main()
