import yaml
import matplotlib.pyplot as plt
import glob
import os
import numpy as np

def load_path_from_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        paths = yaml.safe_load(file)
    return paths

def extract_positions(path_data):
    x_coords = []
    y_coords = []
    for pose in path_data:
        x_coords.append(pose['pose']['position']['x'])
        y_coords.append(pose['pose']['position']['y'])
    return x_coords, y_coords

def extract_positions_odom(path_data):
    x_coords = []
    y_coords = []
    for pose in path_data:
        x_coords.append(pose['position']['x'])
        y_coords.append(pose['position']['y'])
    return x_coords, y_coords

def main():
    plt.figure(figsize=(8, 6))
    # Adjust the path pattern to your specific files
    yaml_file1 = 'zab0_refine_groundtruth.yaml'
    x_coords1, y_coords1 = extract_positions(load_path_from_yaml(yaml_file1))
    yaml_file2 = 'liosam_zab0_refine_traj.yaml'
    x_coords2, y_coords2 = extract_positions(load_path_from_yaml(yaml_file2))
    yaml_file3 = 'lidarslamros2_zab0_refine_traj.yaml'
    x_coords3, y_coords3 = extract_positions(load_path_from_yaml(yaml_file3))
    yaml_file4 = 'kissicp_zab0_refine_traj.yaml'
    x_coords4, y_coords4 = extract_positions(load_path_from_yaml(yaml_file4))
    yaml_file5 = 'fastlio2_zab0_refine_traj.yaml'
    x_coords5, y_coords5 = extract_positions(load_path_from_yaml(yaml_file5))
    #yaml_file6 = 'lego_32.yaml'
    #x_coords6, y_coords6 = extract_positions(load_path_from_yaml(yaml_file6))
    yaml_file7 = 'hdl_zab0_refine_traj.yaml'
    x_coords7, y_coords7 = extract_positions(load_path_from_yaml(yaml_file7))
    plt.plot(x_coords1, y_coords1, color='grey', label='Ground Truth')
    plt.plot(x_coords2, y_coords2, color='red', label='LIO-SAM')
    plt.plot(x_coords3, y_coords3, color='blue', label='LidarSLAMROS2')
    plt.plot(x_coords4, y_coords4, color='green', label='KISS-ICP')
    plt.plot(x_coords5, y_coords5, color='pink', label='FastLIO2')
    #plt.plot(x_coords6, y_coords6, color='cyan', label='LeGO-LOAM')
    plt.plot(x_coords7, y_coords7, color='yellow', label='HDL-Graph')



    # Setting axis labels
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    # Setting axis limits
    #plt.xlim(0, 350)  # X-axis range: 0 to 350 meters
    #plt.ylim(0, 100)  # Y-axis range: 0 to 100 meters
    plt.gca().set_aspect(100/100, adjustable='box')

    # Adding grid, title, and legend
    plt.grid(True)
    plt.title('Trajectory Comparison')
    plt.legend()

    plt.savefig('zab0_nolego.png', dpi=300)

    plt.show()

if __name__ == '__main__':
    main()
