# coding=utf-8
from matplotlib import pylab
import numpy as np
import cv2
import time
import math
import serial
import serial.tools.list_ports
import threading



##############Pyserial######################


SERIAL_IS_OPEN = False      # 默认串口关闭
port_name_list = []         # 端口名称列表
port_com_list = []          # 端口号列表
MySerial = None             # 打开的串口

def 扫描串口():
    port_list = list(serial.tools.list_ports.comports())
    if len(port_list) > 0:
        return port_list
    else:
        return None

def 打开串口(port="COM3", bps=115200, timex=5):
    try:
        # 打开串口
        ser = serial.Serial(port, bps, timeout=timex)

        if ser.is_open:
            global SERIAL_IS_OPEN
            SERIAL_IS_OPEN = True
            print("--- 串口打开 ---")
            return ser

    except Exception as e:
        print("--- 打开异常 ---: ", e)
        return None

def 发送数据(ser, text, code="utf-8"):
    try:
        result = ser.write(text.encode(code))
        if result == len(text):
            print("--- 发送成功 ---：", text)
            return result
        else:
            print("--- 发送错误 ---：", "data len:", len(text), "send len:", result)
            return None
    except Exception as e:
        print("--- 发送异常 ---：", e)

def 读取数据(ser, code="utf-8"):
    if ser.in_waiting:
        str = ser.read(ser.in_waiting).decode(code)
        print("--- 读到数据 ---：", str)
        return str
    else:
        return None


def 关闭串口(ser):
    if ser.is_open:
        try:
            global SERIAL_IS_OPEN
            SERIAL_IS_OPEN = False
            ser.close()
            print("--- 串口关闭 ---")
            return 0
        except Exception as e:
            print("--- 关闭异常 ---：", e)
            return -1
    else:
        print("--- 错误 ---：串口未打开！")
        return -1



#####################camera##################################


def nothing(x):
    pass

# 形态学开操作
def open_binary(binary, x, y):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (x, y))  # 获取图像结构化元素
    dst = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)  # 开操作
    return dst


# 形态学闭操作
def close_binary(binary, x, y):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (x, y))  # 获取图像结构化元素
    dst = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)  # 开操作
    return dst


# 形态学腐蚀操作
def erode_binary(binary, x, y):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (x, y))  # 获取图像结构化元素
    dst = cv2.erode(binary, kernel)  # 腐蚀
    return dst


# 形态学膨胀操作
def dilate_binary(binary, x, y):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (x, y))  # 获取图像结构化元素
    dst = cv2.dilate(binary, kernel)  # 膨胀返回
    return dst


# 找到目标函数
def find_marker(image):
    contours, heriachy1 = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 获取轮廓点集(坐标)
    cv2.drawContours(frame, contours, -1, (0, 0, 255), 2)
    return contours


# 距离计算函数
def distance_to_camera(knownWidth, focalLength, perWidth):
    # compute and return the distance from the maker to the camera
    if knownWidth * focalLength == 0 or perWidth == 0:
        return 0
    else:
        return (knownWidth * focalLength) / perWidth

#球颜色检测函数
def color_detetc(frame):

    hmin1 = cv2.getTrackbarPos('hmin1', 'color_adjust1')
    hmax1 = cv2.getTrackbarPos('hmax1', 'color_adjust1')
    smin1 = cv2.getTrackbarPos('smin1', 'color_adjust1')
    smax1 = cv2.getTrackbarPos('smax1', 'color_adjust1')
    vmin1 = cv2.getTrackbarPos('vmin1', 'color_adjust1')
    vmax1 = cv2.getTrackbarPos('vmax1', 'color_adjust1')
    close = cv2.getTrackbarPos('close', 'mor_adjust')
    erode = cv2.getTrackbarPos('erode', 'mor_adjust')
    dilate = cv2.getTrackbarPos('dilate', 'mor_adjust')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # hsv 色彩空间 分割肤色
    lower_hsv1 = np.array([hmin1, smin1, vmin1])
    upper_hsv1 = np.array([hmax1, smax1, vmax1])
    mask1 = cv2.inRange(hsv, lowerb=lower_hsv1, upperb=upper_hsv1)  # hsv 掩码
    ret, thresh1 = cv2.threshold(mask1, 40, 255, cv2.THRESH_BINARY)  # 二值化处理

    dst_close = close_binary(thresh1, close, close)
    dst_erode = erode_binary(dst_close, erode, erode)
    dst_dilate = dilate_binary(dst_erode, dilate, dilate)
    #cv2.imshow('dst_close:', dst_close)
    #cv2.imshow('dst_erode:', dst_erode)
    #cv2.imshow('dst_dilate:', dst_dilate)
    return dst_dilate

#标记1颜色检测函数
def color_detetc2(frame):

    hmin2 = cv2.getTrackbarPos('hmin2', 'color_adjust2')
    hmax2 = cv2.getTrackbarPos('hmax2', 'color_adjust2')
    smin2 = cv2.getTrackbarPos('smin2', 'color_adjust2')
    smax2 = cv2.getTrackbarPos('smax2', 'color_adjust2')
    vmin2 = cv2.getTrackbarPos('vmin2', 'color_adjust2')
    vmax2 = cv2.getTrackbarPos('vmax2', 'color_adjust2')
    close2 = cv2.getTrackbarPos('close2', 'mor_adjust2')
    erode2 = cv2.getTrackbarPos('erode2', 'mor_adjust2')
    dilate2 = cv2.getTrackbarPos('dilate2', 'mor_adjust2')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # hsv 色彩空间 分割肤色
    lower_hsv2 = np.array([hmin2, smin2, vmin2])
    upper_hsv2 = np.array([hmax2, smax2, vmax2])
    mask2 = cv2.inRange(hsv, lowerb=lower_hsv2, upperb=upper_hsv2)  # hsv 掩码
    ret, thresh2 = cv2.threshold(mask2, 40, 255, cv2.THRESH_BINARY)  # 二值化处理

    dst_close2 = close_binary(thresh2, close2, close2)
    dst_erode2 = erode_binary(dst_close2, erode2, erode2)
    dst_dilate2 = dilate_binary(dst_erode2, dilate2, dilate2)
    #cv2.imshow('dst_close:', dst_close)
    #cv2.imshow('dst_erode:', dst_erode)
    #cv2.imshow('dst_dilate:', dst_dilate)
    return dst_dilate2

#标记2颜色检测函数
def color_detetc3(frame):

    hmin3 = cv2.getTrackbarPos('hmin3', 'color_adjust3')
    hmax3 = cv2.getTrackbarPos('hmax3', 'color_adjust3')
    smin3 = cv2.getTrackbarPos('smin3', 'color_adjust3')
    smax3 = cv2.getTrackbarPos('smax3', 'color_adjust3')
    vmin3 = cv2.getTrackbarPos('vmin3', 'color_adjust3')
    vmax3 = cv2.getTrackbarPos('vmax3', 'color_adjust3')
    close3 = cv2.getTrackbarPos('close3', 'mor_adjust3')
    erode3 = cv2.getTrackbarPos('erode3', 'mor_adjust3')
    dilate3 = cv2.getTrackbarPos('dilate3', 'mor_adjust3')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # hsv 色彩空间 分割肤色
    lower_hsv3 = np.array([hmin3, smin3, vmin3])
    upper_hsv3 = np.array([hmax3, smax3, vmax3])
    mask3 = cv2.inRange(hsv, lowerb=lower_hsv3, upperb=upper_hsv3)  # hsv 掩码
    ret, thresh3 = cv2.threshold(mask3, 40, 255, cv2.THRESH_BINARY)  # 二值化处理

    dst_close3 = close_binary(thresh3, close3, close3)
    dst_erode3 = erode_binary(dst_close3, erode3, erode3)
    dst_dilate3 = dilate_binary(dst_erode3, dilate3, dilate3)
    #cv2.imshow('dst_close:', dst_close)
    #cv2.imshow('dst_erode:', dst_erode)
    #cv2.imshow('dst_dilate:', dst_dilate)
    return dst_dilate3


##################main#####################


if __name__ == "__main__":

    KNOWN_DISTANCE = 24.0
    # A4纸的长和宽(单位:inches)
    KNOWN_WIDTH = 2.69
    KNOWN_HEIGHT = 2.27
    # 通过摄像头标定获取的像素焦距
    focalLength = 511.82
    print('focalLength = ', focalLength)

    # 打开摄像头
    camera = cv2.VideoCapture(1)
    #参数调节窗口
    cv2.namedWindow("color_adjust1")
    cv2.namedWindow("mor_adjust")
    cv2.namedWindow("color_adjust2")
    cv2.namedWindow("mor_adjust2")
    cv2.namedWindow("color_adjust3")
    cv2.namedWindow("mor_adjust3")
    

    #球参数初始值设置
    cv2.createTrackbar("hmin1", "color_adjust1", 25, 255, nothing)
    cv2.createTrackbar("hmax1", "color_adjust1", 77, 255, nothing)
    cv2.createTrackbar("smin1", "color_adjust1", 100, 255, nothing)
    cv2.createTrackbar("smax1", "color_adjust1", 255, 255, nothing)
    cv2.createTrackbar("vmin1", "color_adjust1", 100, 255, nothing)
    cv2.createTrackbar("vmax1", "color_adjust1", 255, 255, nothing)
    cv2.createTrackbar("close", "mor_adjust", 30, 30, nothing)
    cv2.createTrackbar("erode", "mor_adjust", 26, 30, nothing)
    cv2.createTrackbar("dilate", "mor_adjust", 30, 30, nothing)

    #标记1参数初始值设置
    cv2.createTrackbar("hmin2", "color_adjust2", 16, 255, nothing)
    cv2.createTrackbar("hmax2", "color_adjust2", 31, 255, nothing)
    cv2.createTrackbar("smin2", "color_adjust2", 119, 255, nothing)
    cv2.createTrackbar("smax2", "color_adjust2", 255, 255, nothing)
    cv2.createTrackbar("vmin2", "color_adjust2", 0, 255, nothing)
    cv2.createTrackbar("vmax2", "color_adjust2", 255, 255, nothing)
    cv2.createTrackbar("close2", "mor_adjust2", 30, 30, nothing)
    cv2.createTrackbar("erode2", "mor_adjust2", 6, 30, nothing)
    cv2.createTrackbar("dilate2", "mor_adjust2", 19, 30, nothing)

    #标记2参数初始值设置
    cv2.createTrackbar("hmin3", "color_adjust3", 0, 25, nothing)
    cv2.createTrackbar("hmax3", "color_adjust3", 30, 255, nothing)
    cv2.createTrackbar("smin3", "color_adjust3", 100, 255, nothing)
    cv2.createTrackbar("smax3", "color_adjust3", 170, 255, nothing)
    cv2.createTrackbar("vmin3", "color_adjust3", 0, 255, nothing)
    cv2.createTrackbar("vmax3", "color_adjust3", 255, 255, nothing)
    cv2.createTrackbar("close3", "mor_adjust3", 30, 30, nothing)
    cv2.createTrackbar("erode3", "mor_adjust3", 11, 30, nothing)
    cv2.createTrackbar("dilate3", "mor_adjust3", 22, 30, nothing)

    count_start=1  #是否进入计时标志
    L_wait=30        #开始时等30帧，测量绳长均值
    T_wait=30      #等30帧，测量球初始横坐标平均值
    T_count=5      #T和g每十次取一次平均
    sum_T=0
    sum_g=0
    L=0            #绳长
    x0=0           #初始横坐标
    X=29.0           #舵机坐标
    Y=8
    Z=52.0
    delta_x=0      #球和舵机坐标差
    delta_y=0
    delat_z=0
    LR_rad=0       #舵机旋转角度弧度制
    UD_rad=0
    LR_angle=0     #角度制
    UD_angle=0
    ct=0
    arr_x=np.array([])   #小球轨迹储存数组
    arr_y=np.array([])
    arr_t=np.arange(1,13)
    arr_ct=12  #每得到k个数据拟合一次


    #打开串口
    MySerial=打开串口("COM3", 115200, 5)  
    #发送测试
    发送数据(MySerial, "COM is ok", "gbk")
    

    while camera.isOpened():
        # get a frame
        ret, frame = camera.read()         


        #球的初始横坐标
        while T_wait:
            dst_dilate = color_detetc(frame)
            contours = find_marker(dst_dilate)
            if contours == 0:
                continue
            # compute the bounding box of the of the paper region and return it
            # cv2.minAreaRect() c代表点集，返回rect[0]是最小外接矩形中心点坐标，
            # rect[1][0]是width，rect[1][1]是height，rect[2]是角度
            for i, contour in enumerate(contours):
                
                
            
                area1 = cv2.contourArea(contour)
                if area1 > 100:
                    (x1, y1), radius1 = cv2.minEnclosingCircle(contours[i])
                    x1 = round(x1,1)
                    y1 = round(y1,1)
     
                    center1 = (int(x1), int(y1))
                    radius1 = int(radius1)
                    cv2.circle(frame, center1, 3, (0, 0, 255), -1)  # 画出重心

                    x2 = round((x1-30)*35/578,1)
                    y2 = round((431-y1)*24.5/409,1)

                
                    #print("球心坐标:", (x2,y2))
                    #画面显示球心坐标
                    #cv2.putText(frame, "Ball:", (x2,y2), cv2.FONT_HERSHEY_SIMPLEX,
                            #0.5, [255, 255, 255])

                               
            x0=x2+x0         
            T_wait -= 1
            if T_wait == 0 :       
                x0=round(x0/30,1)
                print("初始横坐标:", x0)



        #球的坐标
        dst_dilate = color_detetc(frame)
        contours = find_marker(dst_dilate)
        if contours == 0:
            continue
        # compute the bounding box of the of the paper region and return it
        # cv2.minAreaRect() c代表点集，返回rect[0]是最小外接矩形中心点坐标，
        # rect[1][0]是width，rect[1][1]是height，rect[2]是角度
        for i, contour in enumerate(contours):

            
            area1 = cv2.contourArea(contour)
            if area1 > 100:
                (x1, y1), radius1 = cv2.minEnclosingCircle(contours[i])
                x1 = round(x1,1)
                y1 = round(y1,1)
     
                center1 = (int(x1), int(y1))
                radius1 = int(radius1)
                cv2.circle(frame, center1, 3, (0, 0, 255), -1)  # 画出重心

                x2 = round((x1-80)*60.5/500,1)
                y2 = round((445-y1)*49.2/421,1)
   

                #轨迹拟合与预测
                if arr_ct > 0:
                    arr_x=np.append(arr_x,[x2])
                    arr_y=np.append(arr_y,[y2])
                    arr_ct-=1
                    if arr_ct==0:
                        continue


                if arr_ct==0:
                    #print(arr_x)
                    #arr_t=100

                    i=0
                    while i<=10:
                        arr_x[i]=arr_x[i+1]
                        arr_y[i]=arr_y[i+1]
                        i+=1
                    arr_x[11]=x2
                    arr_y[11]=y2


                    fit_xishu_x=np.polyfit(arr_t,arr_x,3)
                    fit_shizi_x=np.poly1d(fit_xishu_x)
                    x_res=fit_shizi_x(arr_t)

                    fit_xishu_y=np.polyfit(arr_t,arr_y,3)
                    fit_shizi_y=np.poly1d(fit_xishu_y)
                    y_res=fit_shizi_y(arr_t)

                   # plot1 = pylab.plot(arr_t, arr_x, '*', label='original values')
                   ## plot2 = pylab.plot(arr_t, x_res, 'r', label='fit values')
                    # pylab.title('')
                   ## pylab.xlabel('')
                  #  pylab.ylabel('')
                  #  pylab.legend(loc=3, borderaxespad=0., bbox_to_anchor=(0, 0))
                   # pylab.show()

                   # plot3 = pylab.plot(arr_t, arr_y, '*', label='original values')
                   ## plot4 = pylab.plot(arr_t, y_res, 'r', label='fit values')
                   # pylab.title('')
                   # pylab.xlabel('')
                    #pylab.ylabel('')
                    #pylab.legend(loc=3, borderaxespad=0., bbox_to_anchor=(0, 0))
                   # pylab.show()
                   
                    x2=round(fit_shizi_x(15),1)  #t=15时的拟合量代替测量量
                    y2=round(fit_shizi_y(15),1)
                                  


                #画面显示球心坐标
                #print("球心坐标:", (x2,y2))
                #cv2.putText(frame, "Ball:", (x1,y1), cv2.FONT_HERSHEY_SIMPLEX,
                        #0.5, [255, 255, 255])         


                #测距
                marker = cv2.minAreaRect(contours[0])
                inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
                # draw a bounding box around the image and display it
                # inches 转换为 cm
                #画面显示距离
                z2=round(inches * 30.48 / 12,1)
                cv2.putText(frame, "%.2fcm" % (inches * 30.48 / 12),
                            (frame.shape[1] - 300, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                            2.0, (0, 255, 0), 3)

                #print("球心坐标:", (x2,y2,z2))


                #计算舵机旋转角度
                delta_x=round(X-x2,1)
                delta_y=round(y2-Y,1)
                delta_z=round(z2,1)
                LR_rad=math.atan(delta_x/delta_z)
                UD_rad=math.atan(delta_y/(math.sqrt(delta_x**2 + delta_z**2)))
                LR_angle=round(LR_rad*360/(2*3.1415926),2)
                UD_angle=round(UD_rad*360/(2*3.1415926),2)
                if delta_y < 0:
                    UD_angle=0
                
                LR_angle+=90
                
                LR_angle=int(LR_angle*100)
                UD_angle=int(UD_angle*100)
            

                #print("舵机旋转角度:", (LR_angle,UD_angle))

                #发送角度
                Send_LR=str(LR_angle)
                Send_UD=str(UD_angle)
                if len(Send_LR)!=5:
                    t1=5-len(Send_LR)
                    while t1:
                        Send_LR="0"+Send_LR
                        t1-=1
                if len(Send_UD)!=5:
                    t2=5-len(Send_UD)
                    while t2:
                        Send_UD="0"+Send_UD
                        t2-=1
                Send_angle="A"+Send_UD+"B"+Send_LR+"D"

                
                
                发送数据(MySerial, Send_angle, "gbk")
               # print("球心坐标:", (x2,y2))
                #print("x:", delta_x)
                #print("y:", delta_y)
                #print("z:", delta_z)
                    


                #计算周期
                if count_start == 1:
                    if x2 >= x0:
                        time_start = time.time()
                        count_start=0
                if count_start == 0:
                    if x2 < x0:
                        time_end=time.time()
                        T=round(2*(time_end - time_start),3)
                        #print("周期=:", T ,"s")
                        #计算g
                        g=round(4*(3.1415926**2)*0.4/(T**2),3)#默认绳长为0.4                        print("重力加速度g=:", g ,"m/s^2")
                        if T_count:
                            sum_T=sum_T+T
                            sum_g=sum_g+g
                            T_count-=1
                        if T_count==0:
                            sum_T=round(sum_T/5,3)
                            sum_g=round(sum_g/5,3)
                            T_count=5
                            #print("平均周期=:", sum_T ,"s")
                            #print("平均重力加速度g=:", sum_g ,"m/s^2")
                            sum_T=0
                            sum_g=0

                        count_start=1




        # show a frame
        cv2.imshow("capture", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    camera.release()
    cv2.destroyAllWindows()
