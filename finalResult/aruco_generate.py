import cv2
import yaml
import os.path
import numpy as np
import cv2.aruco as aruco

from parameters import ArUcoParameters, generate_config, ArUcoDict


def main():

    if not os.path.exists("./config.yaml"):
        generate_config()
    
    with open("./config.yaml") as stream:
        config = yaml.safe_load(stream)
        
    aruco_param = ArUcoParameters(
        dictionary=config['aruco_parameters']["dictionary"],
        sidePixels=config['aruco_parameters']["sidePixels"],
        ids=config['aruco_parameters']["ids"]
    )

    white_image = np.ones((aruco_param.sidePixels * 2, aruco_param.sidePixels * 2, 3), dtype=np.uint8) * 255
    white_image = cv2.cvtColor(white_image, cv2.COLOR_BGR2GRAY)

    dictionary = aruco.getPredefinedDictionary(ArUcoDict[aruco_param.dictionary].value)
    
    for id in aruco_param.ids:
        marker = aruco.generateImageMarker(dictionary=dictionary, id=id, sidePixels=aruco_param.sidePixels)

        # Дополнение
        h_w, w_w = white_image.shape[:2]
        h_m, w_m = marker.shape[:2]

        print(white_image.shape, marker.shape)

        white_image[h_w//2 - h_m//2:h_w//2 - h_m//2 + h_m, w_w//2 - w_m//2:w_w//2 - w_m//2 + w_m]=marker

        cv2.imwrite(f"ArUCO_{aruco_param.sidePixels}x{aruco_param.sidePixels}.png", marker)

        cv2.imshow("marker", white_image)
        cv2.waitKey(0)

if __name__ == "__main__":
    main()

