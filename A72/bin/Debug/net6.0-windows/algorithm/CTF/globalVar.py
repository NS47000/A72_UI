import cv2
import os,stat,sys,time,traceback
import numpy as np
import math
sides=['v','h']
colors=['B','G','R']
#colors=['B','R']
pattern_width=640
pattern_height=480
script_dir =  os.path.abspath(os.path.dirname(__file__))
pattern_folder=os.path.join(script_dir,"pattern")
capture_folder=os.path.join(script_dir,"capture")
process_folder=os.path.join(script_dir,"process")
UI_file_name="UI_status.txt"
def check_folder(folder_path):
    if os.path.exists(folder_path)==0:
        os.mkdir(folder_path)
        os.chmod(folder_path,stat.S_IWRITE)
        print("Creat {} Folder".format(os.path.basename(folder_path)))

def cmdcommand(*input):
    with open('command.bat', 'w') as f:
        for i in range(0,len(input)):
            f.write(input[i]+"\n")
    
    os.system("command.bat")
def show_pattern(img,Dir):
    cv2.imwrite(os.path.join(Dir,"ready.png"),img)
    
    cmdcommand(
               r"call C:\Users\11011105\Anaconda3\condabin\activate.bat",
               r"call conda activate p29_mfg",
               #r"p29-boardctl -d rb3 echo_all",
               r"calibration_client -d rb3 --power on --display on",
               r'calibration_client -d rb3 --show-image "'+os.path.join(Dir,"ready.png")+'"'
               )

def close():
    cmdcommand(
                r"call C:\Users\11011105\Anaconda3\condabin\activate.bat",
                r"call conda activate p29_mfg",
                r"calibration_client -d rb3 --power off --display off"
               )
def show_image(img):
    cv2.namedWindow('Show Mason Image', cv2.WINDOW_NORMAL)
    cv2.imshow('Show Mason Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def is_file_write_complete():
    result_path = os.path.join(script_dir, "..", "..", UI_file_name)
    # 檢查檔案的修改時間是否超過某個閾值，表示寫入已完成
    threshold_seconds = 1  # 設定閾值，單位為秒
    current_time = time.time()
    modified_time = os.path.getmtime(result_path)
    elapsed_time = current_time - modified_time
    return elapsed_time >= threshold_seconds
def show2UI(img, img_message):
    cv2.imwrite(os.path.join(script_dir, "..", "..", "UI.png"), img)
    result_path = os.path.join(script_dir, "..", "..", UI_file_name)
    txt_context = ""
    with open(result_path, 'r') as f:
        line = f.readline()
        while line != "" and line is not None:
            if "image imfo." in line:
                txt_context += "image imfo.:{}\n".format(img_message)
            else:
                txt_context += line
            line = f.readline()
    while not is_file_write_complete():
        time.sleep(0.1)  # 等待0.1秒
    with open(result_path, 'w') as f:
        f.write(txt_context)
    while not is_file_write_complete():
        time.sleep(0.1)
    print("show UI image")


def init_UI():
    # mode 0:write 1:modify 2:delete 3:clear TXT file

    result_path = os.path.join(script_dir, "..", "..", UI_file_name)
    with open(result_path, 'w') as f:
        f.write("image imfo.:{}\n".format("initial UI"))
        f.write("{},{},{}\n".format("Brightness", "initial", "waiting proccess start"))
        f.write("{},{},{}\n".format("CTF", "initial", "waiting proccess start"))
        f.write("{},{},{}\n".format("Contrast", "initial", "waiting proccess start"))
        f.write("{},{},{}\n".format("Uniformity", "initial", "waiting proccess start"))
    while not is_file_write_complete():
        time.sleep(0.1)  # 等待0.1秒
    print("Status update: initial")
    sys.stdout.flush()


def Update_UI(mode, station, status, errorcode):
    # mode 0:write 1:modify 2:delete 3:clear TXT file

    result_path = os.path.join(script_dir, "..", "..", UI_file_name)
    try:

        if mode == 0:
            with open(result_path, 'a') as f:
                f.write("{},{},{}\n".format(station, status, errorcode))
        elif mode == 1:
            txt_context = ""
            with open(result_path, 'r') as f:
                line = f.readline()
                while line != "" and line is not None:
                    if station in line:
                        txt_context += "{},{},{}\n".format(station, status, errorcode)
                    else:
                        txt_context += line
                    line = f.readline()
            time.sleep(0.1)
            while not is_file_write_complete():
                time.sleep(0.1)  # 等待0.1秒
            with open(result_path, 'w') as f:
                f.write(txt_context)

        elif mode == 2:
            txt_context = ""
            with open(result_path, 'r') as f:
                line = f.readline()
                while line != "" and line is not None:
                    if station in line:
                        pass
                    else:
                        txt_context += line
                    line = f.readline()
            with open(result_path, 'w') as f:
                f.write(txt_context)
        elif mode == 3:
            init_UI()
        else:
            print("please select correct mode: 0:write 1:modify 2:delete 3:clear TXT file")

    except Exception as e:
        error_class = e.__class__.__name__  # 取得錯誤類型
        detail = e.args[0]  # 取得詳細內容
        cl, exc, tb = sys.exc_info()  # 取得Call Stack
        lastCallStack = traceback.extract_tb(tb)[-1]  # 取得Call Stack的最後一筆資料
        fileName = lastCallStack[0]  # 取得發生的檔案名稱
        lineNum = lastCallStack[1]  # 取得發生的行號
        funcName = lastCallStack[2]  # 取得發生的函數名稱
        errMsg = "File \"{}\", line {}, in {}: [{}] {}".format(fileName, lineNum, funcName, error_class, detail)
        print("A72 ERROR:{}".format(errMsg))
    while not is_file_write_complete():
        time.sleep(0.1)  # 等待0.1秒
    print("Status update:{} {} {}".format(station, status, errorcode))
    sys.stdout.flush()
    
    

    