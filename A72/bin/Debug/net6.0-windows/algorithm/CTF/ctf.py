import numpy as np
import cv2
import os,sys
from globalVar import *
import Flir_camera
from scipy.signal import argrelextrema , find_peaks
from data_collection import *
import argparse
import traceback
peak_distance=8
needCapture=False
script_dir =  os.path.abspath(os.path.dirname(__file__))


def process_command():
    arg_parser = argparse.ArgumentParser()

    #arg_parser.add_argument('--kalibr-calib', dest='kalibrCalib', action='store_true', help='OE Calibration')
    #arg_parser.set_defaults(kalibrCalib=False)
    arg_parser.add_argument('--SN','-S',default='none',required=True, type=str, help='intput Device SN')
    arg_parser.add_argument('--OperateID','-O',default="none",required=True, type=str, help='intput Operate ID')
    arg_parser.add_argument('--Directory','-D',default=script_dir,required=True, type=str, help='Directory')
    
    return arg_parser.parse_args()

#Update_UI(1,"CTF","Capture","on going")
def detect_roi(square,ratio):
    max_x=0
    max_y=0
    min_x=99999
    min_y=99999
    for point in square:
        if point[0]>max_x:
            max_x=point[0]
        if point[0]<min_x:
            min_x=point[0]
        if point[1]>max_y:
            max_y=point[1]
        if point[1]<min_y:
            min_y=point[1]
    mid_x=(min_x+max_x)/2
    mid_y=(min_y+max_y)/2
    length_x=max_x-min_x
    length_y=max_y-min_y
    shift_x=length_x*(1-ratio)/2
    shift_y=length_y*(1-ratio)/2
    x_min,y_min,x_max,y_max=int(min_x+shift_x),int(min_y+shift_y),int(max_x-shift_x),int(max_y-shift_y)
    
    #print("x_min,y_min,x_max,y_max:{},{},{},{}".format(x_min,y_min,x_max,y_max))
    return x_min,y_min,x_max,y_max
def ListSort(elem):
    return elem[1]*10+elem[0]

def CTF_main(SN,OperateID,Dir):
    check_folder(os.path.join(Dir,"capture"))
    check_folder(os.path.join(Dir,"process"))
    init_UI()
    capture_folder=os.path.join(Dir,"capture")
    process_folder=os.path.join(Dir,"process")
    if needCapture==True:
        
        roi=cv2.imread(os.path.join(pattern_folder,"roi.png"),0)
        show_pattern(roi,Dir)
        Flir_camera.getpicture(os.path.join(capture_folder,"capture_roi.png"),333333,0)
        for half_lamda in range(1,4):
            for side in sides:
                for color in colors:
                    img=cv2.imread(os.path.join(pattern_folder,"CTF_{}_{}_{}.png".format(half_lamda,side,color)))
                    img_gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                    show_pattern(img_gray)
                    Flir_camera.getpicture(os.path.join(capture_folder,"capture_{}_{}_{}.png".format(half_lamda,side,color)),333333,0)
        close()
    #Update_UI(1,"CTF","Calculate","ongoing 2")
    print("start")
    roi_img=cv2.imread(os.path.join(capture_folder,"capture_roi.png"),0)
    ret,th1=cv2.threshold(roi_img,50,255,cv2.THRESH_BINARY)
    kernel=np.ones((5,5),np.uint8)
    img = cv2.erode(th1, kernel,iterations=3)   # 侵蝕
    img_out = cv2.dilate(img, kernel,iterations=3)  # 擴張
    show2UI(img_out,"roi.png")
    contours, _hierarchy = cv2.findContours(img_out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    index = 0
    squares=[]
    roi_list=[]
    img_draw=roi_img.copy()
    for cnt in contours:
        cnt_len = cv2.arcLength(cnt, True) #计算轮廓周长
        cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True) #多边形逼近
        if  cv2.contourArea(cnt)>img_out.shape[0]*img_out.shape[1]//300000:#and cv2.isContourConvex(cnt) and len(cnt) == 4 
            #print("Area:"+str(cv2.contourArea(cnt))+"cnt_len:"+str(len(cnt)))
            M = cv2.moments(cnt) #计算轮廓的矩
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])#轮廓重心
            cnt = cnt.reshape(-1, 2)
            index = index + 1
            squares.append(cnt)
            x_min,y_min,x_max,y_max=detect_roi(cnt,1.0)
            roi_list.append([x_min,y_min,x_max,y_max])
            cv2.rectangle(img_draw, (x_min,y_min), (x_max,y_max), (255, 0, 255), 2) 
    roi_list.sort(key=ListSort)
    list_index=0
    img_draw=cv2.cvtColor(img_draw,cv2.COLOR_GRAY2RGB)
    for roi_squares in roi_list:
        cv2.putText(img_draw,str(list_index),(roi_squares[0],roi_squares[3]), cv2.FONT_HERSHEY_SIMPLEX,1, (0,0,255), 1, cv2.LINE_AA)
        list_index+=1
    cv2.imwrite(os.path.join(process_folder,"roi.png"),img_draw)
    result_path =os.path.join(Dir,"result.csv")
    ctf_result_array=np.zeros((3,5,5,2,3),dtype=np.float32)
    with open(result_path,'w') as f:
        for half_lamda in range(1,4):
            for side in sides:
                for color in colors:
                    check_folder(os.path.join(process_folder,"ctf_{}_{}_{}".format(half_lamda,side,color)))
                    img=cv2.imread(os.path.join(capture_folder,"capture_{}_{}_{}.png".format(half_lamda,side,color)),0)
                    show2UI(img,"capture_{}_{}_{}.png".format(half_lamda,side,color))
                    Update_UI(1,"CTF","Calculate","capture_{}_{}_{}.png".format(half_lamda,side,color))
                    sys.stdout.flush()
                    list_index=0
                    ctf_result=[]
                    img_draw=img.copy()
                    img_draw=cv2.cvtColor(img_draw,cv2.COLOR_GRAY2BGR)
                    for roi_squares in roi_list:#[x_min,y_min,x_max,y_max]
                        img_roi=img[roi_squares[1]:roi_squares[3],roi_squares[0]:roi_squares[2]]
                        img_roi_draw=cv2.cvtColor(img_roi.copy(),cv2.COLOR_GRAY2BGR)
                        line_list=[]
                        Lower=[]
                        Uper=[]
                        Lower_everyLine=[]
                        Uper_everyLine=[]
                        squares_height=roi_squares[3]-roi_squares[1]
                        squares_width=roi_squares[2]-roi_squares[0]
                        
                        if side=='v':
                            for i in range(squares_height):
                                line_list.append(img_roi[i,:])
                                peak_index=find_peaks(img_roi[i,:],height=np.mean(img_roi[i,:]),distance=peak_distance)[0]
                                lower_index=find_peaks(255-img_roi[i,:],height=np.mean(255-img_roi[i,:]),distance=peak_distance)[0]
                                for index in peak_index:
                                    Uper.append(img_roi[i,index])
                                    img_roi_draw[i,index,:]=(255,0,255)
                                for index in lower_index:
                                    Lower.append(img_roi[i,index])
                                    img_roi_draw[i,index,:]=(0,255,255)
                                Lower_everyLine.append(np.mean(Lower))
                                Uper_everyLine.append(np.mean(Uper))
                                #Lower.sort()
                                #Uper.sort()
                        else:
                            for i in range(squares_width):
                                line_list.append(img_roi[:,i])
                                peak_index=find_peaks(img_roi[:,i],height=np.mean(img_roi[:,i]),distance=peak_distance)[0]
                                lower_index=find_peaks(255-img_roi[:,i],height=np.mean(255-img_roi[:,i]),distance=peak_distance)[0]
                                for index in peak_index:
                                    Uper.append(img_roi[index,i])
                                    img_roi_draw[index,i,:]=(255,0,255)
                                for index in lower_index:
                                    Lower.append(img_roi[index,i])
                                    img_roi_draw[index,i,:]=(0,255,255)
                                Lower_everyLine.append(np.mean(Lower))
                                Uper_everyLine.append(np.mean(Uper))
                                #Lower.sort()
                                #Uper.sort()
                    
                        #Uper_79index=round(len(Uper)*0.79)
                        #Lower_21index=round(len(Lower)*0.79)
                        #Uper_value=Uper[Uper_79index].astype(float)
                        #Lower_value=Lower[Lower_21index].astype(float)
                        Uper_value=np.mean(Uper_everyLine)
                        Lower_value=np.mean(Lower_everyLine)
                        ctf_value=(Uper_value-Lower_value)/(Uper_value+Lower_value)
                        ctf_value=round(ctf_value,2)
                        ctf_result.append(ctf_value)
                        cv2.imwrite(os.path.join(process_folder,"ctf_{}_{}_{}".format(half_lamda,side,color),"{}.png".format(list_index)),img_roi_draw)
                        img_draw=cv2.putText(img_draw,"ctf:{}".format(round(ctf_value,2)),(roi_squares[0],roi_squares[3]), cv2.FONT_HERSHEY_SIMPLEX,1, (255,0,0), 1, cv2.LINE_AA)
                        cv2.rectangle(img_draw, (roi_squares[0],roi_squares[1]), (roi_squares[2],roi_squares[3]), (0, 0,255), 2)
                        ctf_result_array[colors.index(color)][list_index//5][list_index%5][sides.index(side)][half_lamda-1]=ctf_value*100
                        list_index+=1
                    f.write("{}_{}_{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(half_lamda,side,color,ctf_result[0],ctf_result[1],ctf_result[2],ctf_result[3],ctf_result[4],ctf_result[5],ctf_result[6],ctf_result[7],ctf_result[8],ctf_result[9],ctf_result[10],ctf_result[11],ctf_result[12],ctf_result[13],ctf_result[14],ctf_result[15],ctf_result[16],ctf_result[17],ctf_result[18],ctf_result[19],ctf_result[20],ctf_result[21],ctf_result[22],ctf_result[23],ctf_result[24]))
                    cv2.imwrite(os.path.join(process_folder,"ctf_{}_{}_{}".format(half_lamda,side,color),"result.png"),img_draw)
                    
    a=DUT_data(SN,OperateID)
    a.Set_CTF_result(ctf_result_array)
    a.Save_CSV_file(Dir) 
    a.Save_CSV_file_UI()               
    Update_UI(1,"CTF","done","")
    

if __name__ == '__main__':
    try:
        args = process_command()
        CTF_main(args.SN,args.OperateID,args.Directory)
    except Exception as e:
        error_class = e.__class__.__name__ #取得錯誤類型
        detail = e.args[0] #取得詳細內容
        cl, exc, tb = sys.exc_info() #取得Call Stack
        lastCallStack = traceback.extract_tb(tb)[-1] #取得Call Stack的最後一筆資料
        fileName = lastCallStack[0] #取得發生的檔案名稱
        lineNum = lastCallStack[1] #取得發生的行號
        funcName = lastCallStack[2] #取得發生的函數名稱
        errMsg = "File \"{}\", line {}, in {}: [{}] {}".format(fileName, lineNum, funcName, error_class, detail)
        print("A72 ERROR:{}".format(errMsg))

