import cv2
import yaml
import os.path
import subprocess
import numpy as np
import cv2.aruco as aruco

from operator import itemgetter
from parameters import CameraParameters, ArUcoParameters, DetectionParameters, generate_config, ArUcoDict


def get_center_corner(corner: np.ndarray) -> tuple:
   x = (corner[2][1] - corner[0][1]) // 2 + corner[0][1]
   y = (corner[2][0] - corner[0][0]) // 2 + corner[0][0]
   
   return x, y
   
    
def drawBboxes(img, corners, ids):
    
    if len(corners) > 0:
        
        for corner, id in zip(corners, ids.flatten()):
           
            corner = corner.reshape((4, 2)).astype(np.int32)
            
            top_left, top_right, bottom_right, bottom_left = corner
            
            x, y = get_center_corner(corner)
            
            for i in range(len(corner)):
                if i == len(corner) - 1:
                    cv2.line(img, corner[i], corner[0], (0, 0, 255), 2)
                    continue
                
                cv2.line(img, corner[i], corner[i+1], (0, 0, 255), 2)
            
            img = cv2.circle(img, corner[0], 5, (255, 0, 0), 2)        
            img = cv2.circle(img, (y, x), 5, (255, 0, 0), cv2.FILLED)
            
    return img


def areaRectangle(frame, corners, ids):
    ids_workpiece = {k: [] for k in range(1, 51)}
    
    if len(corners) > 0:
        for corner, id in zip(corners, ids.flatten()):
            ids_workpiece[id].append(get_center_corner(np.squeeze(corner).astype(np.int32)))
        
        ids_workpiece_result = dict()
        
        keys = list(filter(lambda x: ids_workpiece[x] != [] and len(ids_workpiece[x]) > 1, ids_workpiece))
        
        for key in keys:
            square_max = 0
            position_max = []
            for pos_id in range(len(ids_workpiece[key])):
                positions = ids_workpiece[key][pos_id:pos_id+2]
                
                if len(positions) == 1:
                    break
                
                x1, x2 = positions[0][0], positions[1][0]
                y1, y2 = positions[0][1], positions[1][1]
                
                square = ((pow(x2 - x1, 2) + pow(y2 - y1, 2)) ** 0.5) / 2
                if square > square_max:
                    square_max = square
                    position_max = positions
                    
            ids_workpiece_result[key] = position_max
    
    return frame, ids_workpiece_result  


def main():
    if not os.path.exists("./config.yaml"):
        generate_config()
    
    with open("./config.yaml") as stream:
        config = yaml.safe_load(stream)
        
    cam_param = CameraParameters(
        capture_id=config['camera_parameters']["capture_id"],
        calibration_data_path=config['camera_parameters']["calibration_data_path"],
        calibration_image_path=config['camera_parameters']["calibration_image_path"],
        calibration_type=config['camera_parameters']["calibration_type"],
        save_type=config['camera_parameters']["save_type"],
        calibration_pattern_size=config['camera_parameters']["calibration_pattern_size"],
        time_limit=config['camera_parameters']["time_limit"],
        img_count=config['camera_parameters']["img_count"],
    )
    
    aruco_param = ArUcoParameters(
        dictionary=config['aruco_parameters']["dictionary"],
        sidePixels=config['aruco_parameters']["sidePixels"],
        ids=config['aruco_parameters']["ids"]
    )
    
    detect_param = DetectionParameters(
        aruco_real_size=config["detection_parameters"]["aruco_real_size"],
        axis_lenght=config["detection_parameters"]["axis_lenght"],
        robot_position_xyz=config["detection_parameters"]["robot_position_xyz"],
        w=config["detection_parameters"]["w"],
        l=config["detection_parameters"]["l"],
        rTorch=config["detection_parameters"]["rTorch"],
        details_path=config["detection_parameters"]["details_path"]
    )
    
    with np.load(cam_param.calibration_data_path) as file:
        mtx, dist, rvecs, tvecs = [file[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
    
    dictionary = aruco.getPredefinedDictionary(ArUcoDict[aruco_param.dictionary].value)
    parameters = aruco.DetectorParameters()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
    
    axis_length = detect_param.axis_lenght
    axis_points = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]]).reshape(-1, 3)

    cap = cv2.VideoCapture(cam_param.capture_id, cv2.CAP_DSHOW)
    
    while cap.isOpened():
        markers_position = dict()
        # markers_position = {8: [[494.5293968795116, -193.71436968432238, 162.0], [263.53859807507854, 207.79872019751875, 162.0]],}
                            # 2: [[494.5293968795116, -193.71436968432238, 162.0], [263.53859807507854, 207.79872019751875, 162.0]]}
        
        _, frame = cap.read()
        # frame = cv2.imread("img_1.jpg")
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, dictionary)
        
        robot_pose = np.array(detect_param.robot_position_xyz).reshape((4,1))
        robotRotationMatrix = np.array([[1,		0,		0],
                                        [0,		1, 		0],
                                        [0, 	0, 		1],
                                        [0, 	0, 		0]
                                       ])
        robotMatrix = np.hstack((robotRotationMatrix, robot_pose))
        h, w, _ = frame.shape
        
        cv2.circle(frame, (int(mtx[0][2]), int(mtx[1][2])), 10, (0, 0, 255), cv2.FILLED)
        
        if len(corners):
            frame = drawBboxes(frame, corners, ids)
            frame, ids_workpiece = areaRectangle(frame, corners, ids)
            keys = list(filter(lambda x: ids_workpiece[x] != [] and len(ids_workpiece[x]) > 1, ids_workpiece))
            
            for key in keys:
                position_marker_to_robot = []
                for i, corner in enumerate(corners):
                    pos = get_center_corner(corner.reshape((4, 2)).astype(np.int32))
                    
                    if not pos in [ids_workpiece[key][0], ids_workpiece[key][1]]:
                        continue

                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, detect_param.aruco_real_size, mtx, dist)
                    corner = corner.astype(np.int32)
                    tvec = np.squeeze(tvec)
                    rvec = np.squeeze(rvec)
                    
                    rvec_matrix = cv2.Rodrigues(rvec)[0]
                    transpose_tvec = tvec[np.newaxis, :].T
                    proj_matrix = np.hstack((rvec_matrix, transpose_tvec))
                    euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6]
                    
                    h_m = (corner[0][2][0] - corner[0][0][0]) // 2 + corner[0][0][0]
                    w_m = (corner[0][2][1] - corner[0][0][1]) // 2 + corner[0][0][1]
                    
                    marker_pose = np.array([tvec[1], tvec[0], 0, 1]).reshape(4,1)
                    
                    transpose_tvec = np.append(transpose_tvec, 1)

                    marker_robot_position = np.squeeze(robotMatrix @ marker_pose)
                    position_marker_to_robot.append(marker_robot_position.tolist()[:3])
                    
                    # TODO: Убрать и перенести в другое место 
                    cv2.putText(frame, f"x: {round(tvec[0], 2)}, y: {round(tvec[1], 2)}, z: {round(tvec[2], 2)}",
                                (h_m, w_m), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)
                    cv2.putText(frame, f"x: {round(marker_robot_position[0], 2)}, y: {round(marker_robot_position[1], 2)}, z: {round(marker_robot_position[2], 2)}",
                                (h_m, w_m + 20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)
                    
                    axis_point_2d, _ = cv2.projectPoints(axis_points, rvec, tvec, mtx, dist)
                    axis_point_2d = tuple(axis_point_2d.reshape(4, 2).astype("int"))
                    
                    cv2.line(frame, axis_point_2d[0], axis_point_2d[1], (0, 0, 255), 3)  # X-Axis (Red)
                    cv2.line(frame, axis_point_2d[0], axis_point_2d[2], (0, 255, 0), 3)  # Y-Axis (Green)
                    cv2.line(frame, axis_point_2d[0], axis_point_2d[3], (255, 0, 0), 3)  # Z-Axis (Blue)
                
                markers_position[key] = sorted(position_marker_to_robot, key=itemgetter(1))
    
        cv2.imshow("Frame", frame)
        print(markers_position)
        
        key = cv2.waitKey(1)
        
        if key == ord('g'):
            print("Generate kuka code")
            
            cv2.destroyAllWindows()
            
            for k in markers_position.keys():
                # command = f'matlab -batch "d_num={k}; x1={markers_position[k][1][0]}; y1={markers_position[k][1][1]}; x2={markers_position[k][0][0]}; y2={markers_position[k][0][1]}; matlab_app;"'
                command = f'matlab -nosplash -nodesktop -r "d_num={k}; x1={markers_position[k][1][0]}; y1={markers_position[k][1][1]}; x2={markers_position[k][0][0]}; y2={markers_position[k][0][1]}; matlab_app;"'
                process = subprocess.run(command, capture_output=True, text=True)
                print(process.stdout)
                print(process.stderr)
            break
        
        if key == ord('q'):
            print("ShutDown")
            break
        
    cv2.destroyAllWindows()
    cap.release()

if __name__ == "__main__":
    main()