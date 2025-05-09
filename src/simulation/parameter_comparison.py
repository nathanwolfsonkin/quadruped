import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

from simulation.post_processing.model_comparison import ModelComparison

def main():
    test_name = "thigh_mass_variations_3"
    root_dir = f"/workspace/src/simulation/data_logs/{test_name}"

    model_dict = {}

    for subdir in os.listdir(root_dir):
        subdir_path = os.path.join(root_dir, subdir)
        if os.path.isdir(subdir_path):
            data_path = os.path.join(subdir_path, "raw_data.csv")
            quad_param_path = os.path.join(subdir_path, "params.yaml")
            if os.path.exists(data_path):
                model_dict[subdir] = ModelComparison(data_path, quad_param_path)
            else:
                print(f"Warning: No raw_data.csv in {subdir_path}")

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xscale('log')
    ax.set_yscale('log')
    ax.set_xlim((.1, 1000))
    ax.set_ylim((.1, 10))
    ax.set_xlabel("Body Mass (kg)")
    ax.set_ylabel("Cost of Transport")
    ax.grid(visible=True, which='both')
    ax.set_axisbelow(True)

    # Plot simulation results
    for i, (name, model) in enumerate(model_dict.items()):
        # if name == 'test_0':
            # Dynamic
            mass = model.model_dict['raw']['gz'].mass
            cot = model.model_dict['raw']['gz'].cost_of_transport()
            ax.scatter(mass, cot, color='green', label='_')

            # Kinematic
            mass = model.model_dict['raw']['py'].quadruped_data.quadruped.m
            cot = model.model_dict['raw']['py'].quadruped_data.cost_of_transport()
            ax.scatter(mass, cot, color='orange', label='_')

            # Synthetic
            mass = model.model_dict['synth']['py'].quadruped_data.quadruped.m
            cot = model.model_dict['synth']['py'].quadruped_data.cost_of_transport()
            ax.scatter(mass, cot, color='blue', label='_')

    # Mass and CoT of various legged beings (mass: kg, minimum CoT: dimensionless)
    cot_dict = {
        'trend1': {
            'mass': .316,
            'cot': 2.63,
            'type': 'trend'
        },
        'trend2': {
            'mass': 750,
            'cot': .223,
            'type': 'trend'
        },
        'horse': {
            'mass': 750,
            'cot': .225,
            'type': 'bio'
        },
        'dog': {
            'mass': 2.5,
            'cot': 1.25,
            'type': 'bio'
        },
        'hunting dog': {
            'mass': 8,
            'cot': .7,
            'type': 'bio'
        },
        'sheep': {
            'mass': 31.6,
            'cot': .44,
            'type': 'bio'
        },
        'cheetah': {
            'mass': 56.2,
            'cot': .398,
            'type': 'bio'
        },
        'rat': {
            'mass': .31,
            'cot': 3.98,
            'type': 'bio'
        },
        'MIT Cheetah': {
            'mass': 31,
            'cot': .5,
            'type': 'robot'
        },
        'Unitree A1': {
            'mass': 12,
            'cot': .88,
            'type': 'robot'
        },
        'Big Dog': {
            'mass': 101,
            'cot': 5.7,
            'type': 'robot'
        },
        'ANYmal': {
            'mass': 30,
            'cot': 1.2,
            'type': 'robot'
        },
    }

    # Plot trend line
    ax.plot([cot_dict['trend1']['mass'], cot_dict['trend2']['mass']],[cot_dict['trend1']['cot'], cot_dict['trend2']['cot']], color='k', lw=2)
    
    # Plot bio data
    for i, (name, data) in enumerate(cot_dict.items()):
        if name != 'trend1' and name != 'trend2':
            x = data['mass']
            y = data['cot']
            ax.scatter(x, y, color='k', label='_')
            ax.annotate(name, (x, y), xycoords='data',
                        xytext=(-np.log(10), -np.log(10)), textcoords='offset points',
                        ha='right', va='top')
    

    ax.legend(('Dynamic', 'Kinematic', 'Synthetic'))

    plt.show()

if __name__ == "__main__":
    main()
