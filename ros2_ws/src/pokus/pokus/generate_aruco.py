import cv2
import cv2.aruco as aruco
import os

# Definovanie typu ArUco markeru
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
marker_id = 3  # ID markeru
marker_size = 200  # Rozmer v pixeloch

# Vygenerovanie a uloženie obrázka do zložky resources
output_dir = "resource"
os.makedirs(output_dir, exist_ok=True)
marker_path = os.path.join(output_dir, "marker_3.png")

marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size)
cv2.imwrite(marker_path, marker_image)

print(f"ArUco marker uložený ako {marker_path}")
