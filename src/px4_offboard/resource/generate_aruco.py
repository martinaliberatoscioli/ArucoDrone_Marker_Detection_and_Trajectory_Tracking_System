import cv2
import numpy as np

# Scelta di un dizionario 
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

marker_id = 42
marker_size = 600  

# Genera il marker
marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# sfondo bianco per migliorare il contrasto
background = np.ones((700, 700), dtype=np.uint8) * 255
x_offset = (700 - marker_size) // 2
y_offset = (700 - marker_size) // 2
background[y_offset:y_offset+marker_size, x_offset:x_offset+marker_size] = marker_img

output_path = "/home/martina/PX4-Autopilot/Tools/simulation/gz/models/arucotag/arucotag.png"


cv2.imwrite(output_path, background, [cv2.IMWRITE_JPEG_QUALITY, 100])


cv2.imshow("Marker Generato", background)
cv2.waitKey(3000)
cv2.destroyAllWindows()

print(f"Nuovo marker ArUco generato: {output_path}")
