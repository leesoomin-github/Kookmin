#!/usr/bin/env python
#-- coding:utf-8 --

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
from xycar_msgs.msg import xycar_motor

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#============================================= 

VEHICLE_LENGTH = 128 ## 차량의 길이를 128 조향각 산출할 때 사용한다
## lfd의 하한과 상한
MIN_LFD = 45
MAX_LFD = 115

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표

##==========================================
# 주차 공간의 거리 및 기울기 등 주행에 사용할 수 있는 값 계산
##==========================================
dy = P_ENTRY[1] -P_END[1]
dx = P_ENTRY[0] -P_END[0]
yaw = math.atan2(dy, dx)
## 주차공간의 기울기
TARGET_YAW = yaw *180/math.pi +90
## 주차 공간 입구와 주차 공간 출구와의 거리 계산
TARGET_DISTANCE = math.sqrt(dy*dy + dx*dx)

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================


def planning(sx, sy, syaw, max_acceleration, dt):
    ## 각각 경로x ,경로y, 곡률의 합, 후진 주행 모드 , 교점을 전역 변수 선언
    global rx, ry ,sum_c, mode, kx
    rx, ry = [], []
    sum_c = 0
    mode = 0
    #=============================================
    ##후진 주행 판단을 위한 거리 구하기 
    #=============================================
    ## 현재 위치와 주차 공간 입구와의 거리 계산
    CURRENT_DISTANCE = math.sqrt((sx-P_ENTRY[0])**2 + (sy-P_ENTRY[1])**2)

    ## 현재 위치가 주차공간의 2배에 해당하는 원의 내부에 있고 차량의 앞 방향이 주차 공간을 바라 보고 있다면
    ## 후진 주행이 필요하다고 판단하여 path를 다른 방식으로 생성
    
    if CURRENT_DISTANCE < 2 * TARGET_DISTANCE and syaw > syaw - 90 and syaw < syaw + 90 :
        ## 주차공간 입구와 출구 사이의 좌표를 통해 두 점을 지나는 직선을 생성하는 행렬
        ## a * P_ENTRY[0] + b = P_ENTRY[1]
        ## a * P_END[0] + b = P_END[1]
        ## a,b 계수를 구하기 위해 행렬을 사용함 >> 계수를 구하여 직선을 특정
        A = np.array([[P_ENTRY[0] ,1],
                      [P_END[0], 1]])
        b= np.array([[P_ENTRY[1]],
                     [P_END[1]]])
        x1= np.linalg.solve(A,b)

        ## height값이 450일 때 제일 후진 주행하기 적합하다고 판단하여 y = 450인 점과 만나는 교점 kx를 구함
        kx = int((450-x1[1])/x1[0])  ## 교점의 x좌표

        ## 교점과 시작 위치를 잇는 2차함수를 생성하는 행렬
        C = np.array([[kx**2, kx**1, 1],
                      [sx**2, sx, 1],
                      [2*sx ,1, 0]])
        d= np.array([[450],
                     [sy],
                     [-get_yaw(syaw)]])
        
        x2= np.linalg.solve(C,d)
        
        ## 위에서 구한 값들을 차례 대로 넣어서 rx, ry에 저장
        ## 2차함수를 먼저 생성하고 그 다음 직선을 생성
        for k in range(kx, int(sx) + 1):
            rx.append(k)
            ry.append(x2[0]*k**2 +x2[1]*k + x2[2])

        for k in range(kx , P_END[0] + 1):
            rx.append(k)
            ry.append(x1[0]*k + x1[1])
        ## 후진 주행을 위한 mode 설정    
        mode = 1

    ## 만약에 원의 내부에 들어오지 않으면 5차함수를 통해 path를 생성                    
    else:
        ##시작 위치와 주차 공간 입구 와 출구를 지나는 5차함수를 생성하는 행렬
        E = np.array([[sx ** 5, sx ** 4,sx ** 3, sx ** 2,sx,1],
                [P_ENTRY[0] ** 5, P_ENTRY[0] **4, P_ENTRY[0]**3,P_ENTRY[0] **2 ,P_ENTRY[0], 1],
                [P_END[0] ** 5, P_END[0] **4, P_END[0]**3,P_END[0] **2 ,P_END[0], 1],
                [5*sx ** 4, 4*sx ** 3,3*sx ** 2, 2*sx,1,0],
                [5 * P_ENTRY[0] ** 4, 4 * P_ENTRY[0]**3, 3*P_ENTRY[0]**2, 2*P_ENTRY[0],1, 0],
                [5 * P_END[0] ** 4, 4 * P_END[0]**3, 3*P_END[0]**2, 2*P_END[0],1, 0]])
        f = np.array([[sy],
                [P_ENTRY[1]],
                [P_END[1]],
                [-get_yaw(syaw)],
                [-1],
                [-1]])

        x3 = np.linalg.solve(E, f)

        ## 주차공간 입구와 출구를 지나는 직선을 생성
        G = np.array([[P_ENTRY[0] ,1],
                      [P_END[0], 1]])
        h= np.array([[P_ENTRY[1]],
                     [P_END[1]]])
        x4= np.linalg.solve(G,h)

        ## 차례대로 넣어준다
        ## 5차함수를 먼저 생성 후 직선을 생성
        ## sx가 주차공간의 입구보다 작다면 range(sx, P_ENTRY[0] + 1)
        if sx < P_ENTRY[0]:
            for k in range(int(sx),P_ENTRY[0] + 1 ):
                rx.append(k)
                ry.append(int(x3[0] * k **5 + x3[1] * k **4 + x3[2] * k**3 + x3[3]*k**2 + x3[4]*k + x3[5]))
            for k in range(P_ENTRY[0] + 1, P_END[0] + 1):
                rx.append(k)
                ry.append(x4[0]*k + x4[1])
        ## 반대라면 range(P_ENTRY[0] , int(sx) + 1)         
        else:
            for k in range(P_ENTRY[0], int(sx) + 1 ):
                rx.append(k)
                ry.append(int(x3[0] * k **5 + x3[1] * k **4 + x3[2] * k**3 + x3[3]*k**2 + x3[4]*k + x3[5]))
            for k in range(P_ENTRY[0] + 1, P_END[0] + 1):
                rx.append(k)
                ry.append(x4[0]*k + x4[1])


    #=============================================
    # tracking 에서의 전방 주시 거리를 구하기 위해 곡률을 구하는 부분
    # 전방주시거리  = k * 곡선의 곡률의 합이라고 생각하여
    # 계수 k를 구하고 곡선의 곡률의 합을 구하여 해당 경로에 맞는 전방 주시 거리를 구함
    #=============================================
    yaw = []
    ds = []
    c = []
    
    # waypoint 각각의 yaw 값과 거리를 구하여 곡률을 구한 뒤 다 더하는 부분
    for i in range(len(rx) -1):
        dx = rx[i+1] - rx[i]
        dy = ry[i+1] - ry[i]
        yaw.append(math.atan2(dy ,dx))
        ds.append(math.hypot(dy, dx))
        
    for i in range(len(yaw) - 1):
        if ds[i] == 0:
            ds[i] = 1
        c.append(abs((yaw[i+1] - yaw[i])/ds[i]))

    sum_c  = sum(c)
    #=============================================
    
    return rx, ry ## 경로 반환       
#=============================================
## 시뮬레이터에서 제공해주는 syaw의 값이 기존의 xy 좌표계에서 사용하는 각도의 값에 차이가 있어
## 따로 기울기를 구해주는 함수 
## syaw 값을 받으면 라디안으로 변환하여 시작 좌표에서의 기울기 값을 구하여 경로 생성에 사용

def get_yaw(syaw):

    yaw = math.tan(math.pi*(syaw/180))
    if syaw == 90 or syaw == 270:
        yaw = 0  

    return yaw
#=============================================

    #=============================================
    # 생성된 경로를 따라가는 함수
    # 파이게임 screen, 현재위치 x,y 현재각도, yaw
    # 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
    # 각도와 속도를 결정하여 주행한다.
    #=============================================
 
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    ## 곡률의 합인 sum_c를 전역변수로 선언하여 사용
    global rx, ry , sum_c, mode, kx
    
    ## 모든 점과의 거리를 계산하여 현재 waypoint가 어디인지 구함
    min_dis=float('inf')
    for i in range(len(rx)) :
        dx=x - rx[i-1]
        dy=y - ry[i-1]
        dis=math.sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis 
            current_wp=i

    ## 직접 구한 계수(K)로 tracking 하는 것이 path의 곡률과 관계가 있다고 생각하여 
    ## 전방 주시 거리와 비슷한 개념으로 사용
    ## 1번 path의 제일 적합한 lfd 는 115이고 3번은 50 4번은 15 2번은 50이어서 
    ## lfd = k * C(경로의 곡률) 이라고 생각하여 lfd/C = k 이런식으로 계산한 후
    ## 1번 path의 k ,2번 path의 k, 3번 path의 k, 4번 path의 k를 다 더하여 평균낸 값이 0.575이다.
    k = 0.575
    ## 계수 k와 곡률의 합인 sum_c를 곱하여 전방 주시 거리로 사용
    lfd = int(k*sum_c)

    ##LFD의 상한과 하한을 정해줌
    if lfd < MIN_LFD :
        lfd = MIN_LFD
    if lfd > MAX_LFD :
        lfd = MAX_LFD

    ## 후진 주행 후 주차할 때의 lfd를 정해줌
    if mode  == 1 :
        lfd = 15

    if mode == 2:
        lfd = 50

    ## 현재 waypoint와 lfd의 합이 전체 경로의 길이보다 크면
    ## 주차공간에 다 왔다고 판단하여 마지막 waypoint와 비교함    
    if current_wp + lfd > len(rx) - 1:
        dy = -(ry[-1]-y)
        dx = rx[-1]-x

    ## 다 오지 않았다면 정상대로 전방주시거리를 참고하여 주행
    else:
        dy = -(ry[current_wp +lfd]-y)
        dx = rx[current_wp + lfd]-x 

    ## lfd를 통해 정한 좌표와 현재 좌표와의 각도차이를 구하는 부분 
    target_yaw= math.atan2(dy,dx)*180/math.pi

    ## 시뮬레이터에서 주는 yaw값과 바로 위에서 구한 target_yaw와의 차이를 차량이 움직여야 할 최종 목표 yaw 값으로 설정
    input_yaw = math.pi*(yaw - target_yaw)/180
    
    ## 속도가 0일때 50으로 입력하여 조향각을 구하는데 차질이 없게 하기 위한 if문
    if velocity == 0:
        velocity = 50 
    
    ## 차량의 길이 VEHICLE_LENGTH, 바로 위에서 구한 목표 yaw값인 input_yaw, 현재 속도 vel, 단위 시간 dt를 통해
    ## 차량의 조향각을 산출함
    input_angle = math.atan2((input_yaw*VEHICLE_LENGTH),(velocity*dt))

    ## 산출된 조향각의 degree화 
    input_angle = input_angle *180/math.pi

    ## planning 함수에서 넘어온 mode 값이 1이면 후진 한다고 판단하여 if 아래의 코드를 수행함
    if mode == 1:

        ## speed와 angle 값은 모두 다 반대로
        speed = -50
        angle = -int(input_angle)

        ## 교점과의 거리를 계산 교점 = (kx ,450)
        dx = x - kx
        dy = y - 450

        ## 교점과의 거리 계산
        dis = math.hypot(dx, dy)

        ## 거리가 100이하 이면 mode에 1을 더하여 후진을 멈추고 전진 주행 하도록 함
        ## 그러면 다음 루프에서 위의 코드 
        ## if mode == 2:가 실행됨 >> lfd 50으로 하고 주행
        if dis < 100:
            mode +=1

    ## mode가 1이 아니면 전진 주행         
    else:
        speed = 50
        angle = int(input_angle)

    ### 주차공간에서 정지하기 위해 AR태그의 좌표를 이용하여 AR태그 좌표와의 거리가 80이하가 되었을때 멈추게 함   
    parking_spot_x = x-AR[0]
    parking_spot_y = y-AR[1]

    ## 주차 공간과의 거리 계산
    parking_distance = math.hypot(parking_spot_x,parking_spot_y)

    ## 거리가 80이하이면 정지
    if parking_distance < 80 :
        speed = 0 

    ## 최종 angle, speed publish
    drive(angle, speed)


   

    
    
