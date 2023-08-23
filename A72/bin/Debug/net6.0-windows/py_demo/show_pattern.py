import globalVar
import cv2

testchart=r"D:\JBD4020_PC_Client_Usb_V1.8.17\GRR\Image_20230804\Color_Uniformity.png"
#testchart=r"D:\JBD4020_PC_Client_Usb_V1.8.17\GRR\Image_20230804\Color_Uniformity.png"
img=cv2.imread(testchart,1)
#globalVar.show_pattern(img)#show
globalVar.show_pattern_second(img)
#globalVar.close()#close