# Calibration d'une caméra : les 4 points de référence sont à rentrer, et les coefficents de la matrice d'homographie sont renvoyés. 
# Ensuite, rentrer les coefficients dans le programme Arduino

import cv2
import numpy as np

pixy = "_2"
# Points dans l'image (pixels)
pts_image = np.array([
    [171,39],
    [110,44],
    [161, 195],
    [61, 173]
], dtype=np.float32)
# Points réels sur le plan de travail (en mm)
pts_reels = np.array([
    [300,165],
    [300,455],
    [943,110],
    [938,445]
], dtype=np.float32)

# Calcul de l'homographie
H, _ = cv2.findHomography(pts_image, pts_reels)

# Affichage des coefficients à copier dans Arduino
print("Copie les coefficients suivants dans ton programme Arduino :\n")

print(f"float h11{pixy} = {H[0,0]:.10f};")
print(f"float h12{pixy} = {H[0,1]:.10f};")
print(f"float h13{pixy} = {H[0,2]:.10f};")
print(f"float h21{pixy} = {H[1,0]:.10f};")
print(f"float h22{pixy} = {H[1,1]:.10f};")
print(f"float h23{pixy} = {H[1,2]:.10f};")
print(f"float h31{pixy} = {H[2,0]:.10f};")
print(f"float h32{pixy} = {H[2,1]:.10f};")
print(f"float h33{pixy} = {H[2,2]:.10f};")