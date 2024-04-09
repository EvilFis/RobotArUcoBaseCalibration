import cv2
import yaml
import glob
import time
import os.path
import numpy as np
import cv2.aruco as aruco

from typing import List, Union, Tuple
from parameters import CameraParameters, ArUcoParameters, generate_config, ArUcoDict

def stream_camera(capture_id: int=0, 
                  calib_type:str="chessboard",
                  calib_size: Union[Tuple[int, int], None] = None,
                  aruco_dict: Union[aruco.Dictionary, None] = None,
                  charuco_board: Union[aruco.CharucoBoard, None] = None,
                  save_type: str="keyboard", 
                  save_folder: str="calibration_img",
                  time_limit: Union[int, None] = None,
                  img_count: Union[int, None] = None) -> None:
    
    
    print("STREAM_CAMERA")
    
    calib_size = (6, 9) if calib_size is None else calib_size
    time_limit = 5 if time_limit is None else time_limit 
    img_count = 10 if img_count is None else img_count
    
    cap = cv2.VideoCapture(capture_id, cv2.CAP_DSHOW)
    count = 0
    start_time = time.time()
    
    # while False:
    while cap.isOpened():
        
        key = cv2.waitKey(1)
        
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img = frame.copy()
        
        # Configurate preview calibration template
        if calib_type == "chessboard":
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            ret, corners = cv2.findChessboardCorners(gray, calib_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    
            if ret:
                corners = cv2.cornerSubPix(gray, corners, (3, 3), (-1,-1), criteria)
                frame = cv2.drawChessboardCorners(frame, calib_size, corners, ret)
        elif calib_type == "charuco":
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)
        
            if len(corners) > 0:
                for corner in corners:
                    cv2.cornerSubPix(gray, corner,
                                    winSize=(3,3),
                                    zeroZone=(-1, -1),
                                    criteria=criteria)
                
                interpolate = aruco.interpolateCornersCharuco(corners, ids, gray, charuco_board)
                
                if interpolate[1] is not None and interpolate[2] is not None and len(interpolate[1]) > 3:
                
                    frame = aruco.drawDetectedCornersCharuco(frame, interpolate[1], interpolate[2], (255, 0, 0))
                    frame = aruco.drawDetectedMarkers(frame, corners, ids, (0, 0, 255))
                
        # Save configuration
        if save_type=="time":
        
            time_value = int(time_limit - (time.time() - start_time))
        
            frame = cv2.putText(frame, str(time_value),
                                (10, 50), cv2.FONT_HERSHEY_COMPLEX, 1,
                                (0, 0, 255), 1)
        
            if time_value == 0:
                print("Save")
                count += 1
                cv2.imwrite(f"{save_folder}/calib_{count}.jpg", img)
                start_time = time.time()
                
            if img_count == count:
                break
        
        
        if key == ord('s') and save_type.lower() == "keyboard":
            count +=1
            cv2.imwrite(f"{save_folder}/calib_{count}.jpg", img)
        
        # Destroy stream
        if key == ord('q'):
            break
        
        # frame show
        cv2.imshow("Detection frame: ", frame)
    
    cv2.destroyAllWindows
    cap.release()  
    

def calibrate_camera_chessboard(images_fname: List[str],
                                calib_size: Tuple[int, int] = (6, 9)) -> Tuple:
    
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    objp = np.zeros((calib_size[0]*calib_size[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:calib_size[0],0:calib_size[1]].T.reshape(-1,2)

    imgpoints = []
    objpoints = []
    
    for fname in images_fname:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        ret, corners = cv2.findChessboardCorners(gray, calib_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        if ret:
            corners = cv2.cornerSubPix(gray, corners, (3, 3), (-1,-1), criteria)
    
            objpoints.append(objp)
            imgpoints.append(corners)
            
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        if ret:
            return mtx, dist, rvecs, tvecs
    
        raise Exception("Не удалось выполнить каллибровку Chessboard") 


def read_chessboards_charuco(images_fname: List[str], 
                     aruco_dict: aruco.Dictionary, 
                     board: aruco.CharucoBoard) -> Tuple:
    
    print("POSE ESTIMATION START")
    
    all_corners = []
    all_ids = []
    decimator = 0
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
    
    for fname in images_fname:
        print(f"=> Processing image: `{fname}`")
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)
        
        if len(corners) > 0:
            for corner in corners:
                cv2.cornerSubPix(gray, corners,
                                 winSize=(3,3),
                                 zeroZone=(-1, -1),
                                 criteria=criteria)
            
            interpolate = aruco.interpolateCornersCharuco(corners, ids, gray, board)
            
            if interpolate[1] is not None and interpolate[2] is not None and len(interpolate[1]) > 3 and decimator%1==0:
                all_corners.append(interpolate[1])
                all_ids.append(interpolate[2])
                            
        decimator += 1
        
    img_size = gray.shape
    
    return all_corners, all_ids, img_size


def calibrate_camera_charuco(all_corners: List, 
                            all_ids: List, 
                            img_size: Tuple[int, int, int], 
                            board: aruco.CharucoBoard) -> Tuple:
    print("CAMERA CALIBRATION")
    
    criteria = (cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9)
    
    ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraCharuco(charucoCorners=all_corners,
                                                                charucoIds=all_ids,
                                                                board=board,
                                                                imageSize=img_size,
                                                                cameraMatrix=None,
                                                                distCoeffs=None,
                                                                criteria=criteria)
    
    if ret:
        return mtx, dist, rvecs, tvecs
    
    raise Exception("Не удалось выполнить каллибровку Charuco доски")    

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
            
    # dictionary = aruco.getPredefinedDictionary(ArUcoDict[aruco_param.dictionary].value)
    # board = aruco.CharucoBoard(cam_param.calibration_pattern_size, 1, .7, dictionary)

    # stream_camera(capture_id=cam_param.capture_id,
    #     		  calib_type=cam_param.calibration_type,
    #               aruco_dict=dictionary,
    #               charuco_board=board,
    #               save_folder=cam_param.calibration_image_path)

    stream_camera(capture_id=cam_param.capture_id,
                  calib_type=cam_param.calibration_type,
                  calib_size=cam_param.calibration_pattern_size,
                  save_folder=cam_param.calibration_image_path)

    # img_fnames = glob.glob("./charuco_img/calib_*.jpg")
    # chessboard_imgs = glob.glob("./calibration_img/calib_*.jpg")

    # # all_corners, all_ids, img_size = read_chessboards_charuco(img_fnames, dictionary, board)
    # # mtx, dist, rvecs, tvecs = calibrate_camera_charuco(all_corners, all_ids, img_size, board)
    # mtx, dist, rvecs, tvecs = calibrate_camera_chessboard(chessboard_imgs, (6, 9))

    # print(mtx)
    # print(dist)

    np.savez(cam_param.calibration_data_path, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
    # # np.savez(cam_param.calibration_data_path, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

if __name__ == "__main__":
    main()