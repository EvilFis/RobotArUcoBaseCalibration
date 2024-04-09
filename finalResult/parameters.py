import yaml
from enum import Enum
from dataclasses import dataclass

@dataclass
class CameraParameters:
    capture_id: int
    calibration_data_path: str
    calibration_image_path: str
    calibration_type: str
    save_type: str
    calibration_pattern_size: tuple
    time_limit: int
    img_count: int
    
@dataclass
class ArUcoParameters:
    dictionary: str
    sidePixels: int
    ids: list
    
@dataclass
class DetectionParameters:
    aruco_real_size: int
    axis_lenght: int
    robot_position_xyz: list
    w: int
    l: int
    rTorch: int
    details_path: str
    
class ArUcoDict(Enum):
    DICT_4X4_50=0
    DICT_4X4_100=1
    DICT_4X4_250=2
    DICT_4X4_1000=3
    DICT_5X5_50=4
    DICT_5X5_100=5
    DICT_5X5_250=6
    DICT_5X5_1000=7
    DICT_6X6_50=8
    DICT_6X6_100=9
    DICT_6X6_250=10
    DICT_6X6_1000=11
    DICT_7X7_50=12
    DICT_7X7_100=13
    DICT_7X7_250=14
    DICT_7X7_1000=15
    DICT_ARUCO_ORIGINAL=16
    DICT_APRILTAG_16h5=17
    DICT_APRILTAG_25h9=18
    DICT_APRILTAG_36h10=19
    DICT_APRILTAG_36h11=20
    
def generate_config():
    camera_parameters = {
        "capture_id": 0,
          "calibration_data_path": "./camera_param/chessboard_calib.npz",
          "calibration_image_path": "./calibration_img",
          "calibration_type": "chessboard",
          "save_type": "keyboard",
          "calibration_pattern_size": [6, 9],
          "time_limit": None,
          "img_count": None	
    }
    
    aruco_parameters = {
          "dictionary": "DICT_4X4_50",
          "sidePixels": 300,
          "ids": [8]
    }
    
    detection_parameters = {
        "aruco_real_size": 80,
          "axis_lenght": 50,
          "robot_position_xyz": [437.39, 12.45, 440.83, 1],
          "w": 5,
          "l": 10,
          "rTorch": 50,
          "details_path": "./workpiece_models"
    }
    
    config = {
        "camera_parameters": camera_parameters,
        "aruco_parameters": aruco_parameters,
        "detection_parameters": detection_parameters
    }
    
    with open("./config.yaml", 'w') as stream:
        yaml.safe_dump(config, stream, default_flow_style=False)
        