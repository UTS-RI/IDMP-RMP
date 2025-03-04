"""
helper functions for loading robot urdf and rmp configurations
"""

import yaml
import os

def load_robot_config(robot_name=None, config_path=None):
    if config_path is None:
        if robot_name == 'franka':
            config_path = os.path.join(os.path.dirname(__file__), '..', 'configs', 'franka_config.yaml')
        elif robot_name == '3link':
            config_path = os.path.join(os.path.dirname(__file__), '..', 'configs', '3link_config.yaml')
        elif robot_name == 'ur5e':
            config_path = os.path.join(os.path.dirname(__file__), '..', 'configs', 'ur5e_config.yaml')
        elif robot_name == 'ur5e_hande':
            config_path = os.path.join(os.path.dirname(__file__), '..', 'configs', 'ur5e_hande_config.yaml')
        elif robot_name == 'ur3e_hande':
            config_path = os.path.join(os.path.dirname(__file__), '..', 'configs', 'ur3e_hande_config.yaml')
        elif robot_name == 'ur3e':
            config_path = os.path.join(os.path.dirname(__file__), '..', 'configs', 'ur3e_config.yaml')
        elif robot_name == 'iiwa':
            config_path = os.path.join(os.path.dirname(__file__), '..', 'configs', 'iiwa_config.yaml')
        else:
            raise ValueError

    print(config_path)

    with open(config_path) as f:
        config = yaml.safe_load(f)
    if robot_name is not None:
        assert(robot_name == config['robot_name'])
    else:
        robot_name = config['robot_name']

    return robot_name, config


def get_robot_urdf_path(robot_name):
    if robot_name == 'franka':
        urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'panda.urdf')
    elif robot_name == '3link':
        urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'three_link_planar_robot.urdf')
    elif robot_name == 'ur5e':
        urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'ur5e.urdf')
    elif robot_name == 'ur5e_hande':
        urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'ur5e_hande.urdf')
    elif robot_name == 'ur3e_hande':
        urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'ur3e_hande.urdf')
    elif robot_name == 'ur3e':
        urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'ur3e.urdf')
    elif robot_name == 'iiwa':
        urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'iiwa.urdf')
    else:
        raise ValueError

    return urdf_path

def get_robot_eef_uid(robot_name):
    if robot_name == "franka":
        eef_uid = 14
    elif robot_name == "3link":
        eef_uid = 6
    elif robot_name == "ur5e":
        eef_uid = 8
    elif robot_name == "ur5e_hande":
        eef_uid = 8
    elif robot_name == "ur3e_hande":
        eef_uid = 8
    elif robot_name == "ur3e":
        eef_uid = 8
    elif robot_name == "iiwa":
        eef_uid = 8
    else:
        raise ValueError
    return eef_uid
