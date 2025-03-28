# AI generated content included
import yaml
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from nav_msgs.msg import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

def path_to_dict(path):
    """
    Convert a Path message to a dictionary.
    """
    path_dict = {
        'header': {
            'stamp': path.header.stamp.sec + path.header.stamp.nanosec * 1e-9,
            'frame_id': path.header.frame_id,
        },
        'poses': []
    }

    for pose in path.poses:
        pose_dict = {
            'header': {
                'stamp': pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9,
                'frame_id': pose.header.frame_id,
            },
            'pose': {
                'position': {
                    'x': pose.pose.position.x,
                    'y': pose.pose.position.y,
                    'z': pose.pose.position.z,
                },
                'orientation': {
                    'x': pose.pose.orientation.x,
                    'y': pose.pose.orientation.y,
                    'z': pose.pose.orientation.z,
                    'w': pose.pose.orientation.w,
                }
            }
        }
        path_dict['poses'].append(pose_dict)
    
    return path_dict


def odom_to_dict(odom):
    """
    Convert an Odometry message to a dictionary.
    """
    odom_dict = {
        'header': {
            'stamp': odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9,
            'frame_id': odom.header.frame_id,
        },
        'child_frame_id': odom.child_frame_id,
        'pose': {
            'position': {
                'x': odom.pose.pose.position.x,
                'y': odom.pose.pose.position.y,
                'z': odom.pose.pose.position.z,
            },
            'orientation': {
                'x': odom.pose.pose.orientation.x,
                'y': odom.pose.pose.orientation.y,
                'z': odom.pose.pose.orientation.z,
                'w': odom.pose.pose.orientation.w,
            }
        }
    }
    return odom_dict


def convert_bag_to_yaml(bag_file, yaml_file, topic_name):
    """
    Convert a ROS 2 bag file to a YAML file for a specific topic.
    """
    storage_options = StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    paths = []
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == topic_name:
            msg_type = get_message('nav_msgs/msg/Path') #nav_msgs/msg/Path  nav_msgs/msg/Odometry
            msg = deserialize_message(data, msg_type)
            paths.append(path_to_dict(msg))  #odom_to_dict

    with open(yaml_file, 'w') as outfile:
        yaml.dump(paths[-1], outfile, default_flow_style=False)

if __name__ == "__main__":
    rclpy.init()
    bag_file = 'fastlio2_zab0_3'  # replace with your input ROS 2 bag file
    yaml_file = 'fastlio2_zab0_3.yaml'  # replace with your desired output YAML file
    topic_name = '/path'  # replace with your topic name

    convert_bag_to_yaml(bag_file, yaml_file, topic_name)

    #bag_file2 = 'kissicp_64'  # replace with your input ROS 2 bag file
    #yaml_file2 = 'kissicp_64_traj.yaml'  # replace with your desired output YAML file

    #convert_bag_to_yaml(bag_file2, yaml_file2, topic_name)

    #bag_file3 = 'kissicp_faster'  # replace with your input ROS 2 bag file
   # yaml_file3 = 'kissicp_faster_traj.yaml'  # replace with your desired output YAML file

    #convert_bag_to_yaml(bag_file3, yaml_file3, topic_name)

    #bag_file4 = 'kissicp_big'  # replace with your input ROS 2 bag file
    #yaml_file4 = 'kissicp_big_traj.yaml'  # replace with your desired output YAML file

    #convert_bag_to_yaml(bag_file4, yaml_file4, topic_name)

    rclpy.shutdown()
