import numpy as np
import cv2
import os
from globalVar import *

script_dir =  os.path.abspath(os.path.dirname(__file__))


check_folder(pattern_folder)
roi_x_table=[-2,-1,0,1,2]
roi_y_table=[-2,-1,0,1,2]

def generate_ctfLine(img_shape,line_width,direction='v'):
    img_B=np.zeros(img_shape,dtype=np.uint8)
    img_G=np.zeros(img_shape,dtype=np.uint8)
    img_R=np.zeros(img_shape,dtype=np.uint8)
    if direction=='v':
        for index in range(img_shape[1]):
            if index//line_width%2==1:
                img_B[:,index,:]=255
                img_G[:,index,:]=255
                img_R[:,index,:]=255
    else:
        for index in range(img_shape[0]):
            if index//line_width%2==1:
                img_B[index,:,:]=255
                img_G[index,:,:]=255
                img_R[index,:,:]=255
    return img_B,img_G,img_R        
    
    
for half_lamda in range(1,4):
    for side in sides:
        img_B,img_G,img_R=generate_ctfLine((pattern_height,pattern_width,3),half_lamda,side)
        cv2.imwrite(os.path.join(pattern_folder,"CTF_{}_{}_{}.png".format(half_lamda,side,"B")),img_B)
        cv2.imwrite(os.path.join(pattern_folder,"CTF_{}_{}_{}.png".format(half_lamda,side,"G")),img_G)
        cv2.imwrite(os.path.join(pattern_folder,"CTF_{}_{}_{}.png".format(half_lamda,side,"R")),img_R)
        
roi_width=round(pattern_width*0.75/14)
roi_height=round(pattern_height*0.75/14)
print("ROI size:{}x{}".format(roi_height,roi_width))
roi_shift_x=round(pattern_width/2*0.4)
roi_shift_y=round(pattern_height/2*0.4)
mid_x=pattern_width//2
mid_y=pattern_height//2
print("shift x:{}   y:{}".format(roi_shift_x,roi_shift_y))
roi_img=np.zeros((pattern_height,pattern_width),np.uint8)
for roi_y in roi_y_table:
    for roi_x in roi_x_table:
        roi_img[mid_y+roi_y*roi_shift_y-roi_height//2:mid_y+roi_y*roi_shift_y-roi_height//2+roi_height,mid_x+roi_x*roi_shift_x-roi_width//2:mid_x+roi_x*roi_shift_x-roi_width//2+roi_width]=255
cv2.imwrite(os.path.join(pattern_folder,"roi.png"),roi_img)

    
     

