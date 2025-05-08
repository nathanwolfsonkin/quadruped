import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from simulation.post_processing.model_comparison import ModelComparison

def main():
    test_name = "thigh_mass_variations"
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

    # Show background image (assumed to be in linear space)
    background_path = "/workspace/src/simulation/cost_of_transport_comparison.png"
    img = mpimg.imread(background_path)
    ax.imshow(img, extent=[.01, 1000, .1, 10], aspect='auto', alpha=0.5)
    ax.axis('off') 

    # Create a second axes on top, sharing position
    ax2 = fig.add_axes(ax.get_position(), frameon=False)
    ax2.set_xscale('log')
    ax2.set_yscale('log')
    ax2.set_xlim((.01, 1000))
    ax2.set_ylim((.1, 10))

    # Plot simulation results
    for i, (name, model) in enumerate(model_dict.items()):
        # Dynamic
        mass = model.model_dict['raw']['gz'].mass
        cot = model.model_dict['raw']['gz'].cost_of_transport()
        ax2.scatter(mass, cot, label=name+"_dynam")

        # Kinematic
        mass = model.model_dict['raw']['py'].quadruped_data.quadruped.m
        cot = model.model_dict['raw']['py'].quadruped_data.cost_of_transport()
        ax2.scatter(mass, cot, label=name+"_kinem")

        # Synthetic
        mass = model.model_dict['synth']['py'].quadruped_data.quadruped.m
        cot = model.model_dict['synth']['py'].quadruped_data.cost_of_transport()
        ax2.scatter(mass, cot, label=name+"_synth")

    ax2.legend()
    ax2.set_xlabel("Body Mass (kg)")
    ax2.set_ylabel("Cost of Transport")

    plt.show()

if __name__ == "__main__":
    main()
