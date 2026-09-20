import cv2                                                                     # OpenCV 라이브러리 임포트
import numpy as np                                                             # Numpy 라이브러리 임포트 
import serial
import time
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk

serial_port = serial.Serial('COM2', 57600, timeout=1)                          # 사용할 포트를 지정
time.sleep(2)                                                                  # 포트 안정화에 필요한 시간 지정  

def send_data_FK():                                                            # 송신을 위한 함수 생성
    try:
        BA_UI = float(boom_angle_entry.get())
        AA_UI = float(arm_angle_entry.get())
        data_to_send = f"SF,{AA_UI},{BA_UI},E\n"
        serial_port.write(data_to_send.encode())
        messagebox.showinfo("Data Sent", f"Data sent to Arduino: {data_to_send}")
    except ValueError:
        messagebox.showwarning("Invalid Input", "Please enter valid numeric values for the angles.")

def send_data_IK():
    try:
        X_UI = float(X_entry.get())
        Y_UI = float(Y_entry.get())
        data_to_send = f"SI,{X_UI},{Y_UI},E\n"
        serial_port.write(data_to_send.encode())
        messagebox.showinfo("Data Sent", f"Data sent to Arduino: {data_to_send}")
    except ValueError:
        messagebox.showwarning("Invalid Input", "Please enter valid numeric values for the angles.") 

def send_data_Cal():
    if 'Calibration_Angle_Boom' in globals() and 'Calibration_Angle_Arm' in globals():
        data_to_send = f"SF,{Calibration_Angle_Arm},{Calibration_Angle_Boom},E\n"
        serial_port.write(data_to_send.encode())
        messagebox.showinfo("Data Sent", f"Data sent to Arduino: {data_to_send}")
    else:
        messagebox.showwarning("Data Missing", "Angles have not been calculated yet.")

def close_app():                                                               # GUI및 함수 사용을 위한 Close 관련 함수 생성
    cap.release()
    serial_port.close()
    root.destroy()
    cv2.destroyAllWindows()

root = tk.Tk()                                                                 # GUI사용을 위한 기본 틀 생성
root.title("Arduino Control Interface")
root.geometry("800x1000")
 
canvas = tk.Canvas(root, width=640, height=480)                                # OpenCV 창 import를 위한 위치 지정
canvas.pack()

boom_angle_label = tk.Label(root, text="Boom Angle (BA_UI):")                  # Boom Angle 송신을 위한 창 생성
boom_angle_label.pack(pady=5)
boom_angle_entry = tk.Entry(root)
boom_angle_entry.pack(pady=5)
 
arm_angle_label = tk.Label(root, text="Arm Angle (AA_UI):")                    # Arm Angle 송신을 위한 창 생성
arm_angle_label.pack(pady=5)
arm_angle_entry = tk.Entry(root)
arm_angle_entry.pack(pady=5)

send_angle_button = tk.Button(root, text="SEND_Angle", command=send_data_FK)   # Forward Kinematics 송신을 위한 버튼 생성
send_angle_button.pack(pady=10)

X_label = tk.Label(root, text="X (BA_UI):")
X_label.pack(pady=5)
X_entry = tk.Entry(root)
X_entry.pack(pady=5)

Y_label = tk.Label(root, text="Y(AA_UI):")
Y_label.pack(pady=5)
Y_entry = tk.Entry(root)
Y_entry.pack(pady=5)

send_button = tk.Button(root, text="SEND_Coordinate", command=send_data_IK) 
send_button.pack(pady=15)

send_button = tk.Button(root, text="SEND_Calibration", command=send_data_Cal)  # Calibartion을 위한 버튼 생성 
send_button.pack(pady=10)

close_button = tk.Button(root, text="CLOSE", command=close_app)                # 종료를 위한 버튼 생성
close_button.pack(pady=5)

# 색영역 지정                                                                   # [] 안에 [H,S,V] 의 순서로 영역을 지정
lower_red1 = np.array([0, 130, 100])                                           # 빨간색 색영역 지정
upper_red1 = np.array([30, 255, 255])                                          # 빨간색의 경우 0주변과 179주변이 그 영역임   
lower_red2 = np.array([150, 130, 100])                                         # 그렇기 때문에 2가지 색영역을 불러옴
upper_red2 = np.array([179, 255, 255])

cap = cv2.VideoCapture(0)                                                      # 카메라를 활성화하여 스트림을 가져옴(0번은 기본카메라, 기본카메라가 아닌 추가 카메라들은 1~로 가져올수 있음) 
       
previous_reds = {'red1': None, 'red2': None, 'red3': None, 'red4': None}       # 이전 프레임의 red1, red2, red3, red4 좌표를 저장
initial_detection_complete = False                                             # 초기 탐지 여부를 확인하기 위한 플래그
received_data = ""                                                             # 아두이노로 데이터 수신을 하기 위한 변수 지정
                                                 
def update_frame():                                                                                # OpenCV를 함수화
    global red1, red2, red3, red4, received_data, previous_reds  # 사용할 변수들을 전역변수로 설정해 유지
    global BA_UI, AA_UI, initial_detection_complete,Calibration_Angle_Arm,Calibration_Angle_Boom
    global X_UI, Y_UI                                                                  
    ret, frame = cap.read()                                                    # 영상이 불러와지는지 여부를 확인하고 프레임을 가져옴.
    if not ret:                                                                # 영상이 불러와지지 않은 경우 
        print("카메라에서 영상을 가져올 수 없습니다.")                           # 에러메세지 출력 
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)                               # OpenCV는 이미지 처리에 BGR을 사용하기 때문에 BGR을 HSV로 바꿔야함
    
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)                       # OpenCV내에서 사용할 빨간색 색영역 지정
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = mask_red1 | mask_red2                                           # 2개로 나눠진 빨간색 색영역을 하나로 통합
       
    red_only = cv2.bitwise_and(frame, frame, mask=mask_red)                    # 프레임에서 빨간색 색영역만 추출
    

    gray_red = cv2.cvtColor(red_only, cv2.COLOR_BGR2GRAY)                      # Hough Circle Trasform은 그레이스케일의 데이터로 변환해야되기 
                                                                               # 때문에 OpenCV의 BGR 값들을 GrayScale로 변환  
    gray_red = cv2.GaussianBlur(gray_red, (11, 11), 2)                         # 경계를 명확하게 하기 위해 가우시안 블러를 사용  

    
    circles_red = cv2.HoughCircles(gray_red, cv2.HOUGH_GRADIENT, dp=1.0,       # Hough Circle Transform을 기반으로 원형 객체 탐지
                                   minDist=10, param1=10, param2=30,           # dp : 누적매개변수의 해상도를 원본 이미지의 1.2배로 설정함(높을수록 정확도는 높아지나 탐지 속도가 느려짐)
                                   minRadius=5, maxRadius=100)                 # minDis : 원들 사이의 최소 거리, 픽셀 단위
    
    detected_circles = []                                                      # 탐지된 원들의 정보를 저장하기 위한 빈 공간 생성

    if circles_red is not None:                                                # 빨간색 원이 탐지되었을 때만 실행
        circles_red = np.round(circles_red[0, :]).astype("int")                # 탐지된 원의 좌표, 반지름을 정수로 변환
        for (x, y, r) in circles_red:                                          # 정수로 변환된 좌표, 반지름에 대해서 반복문 시작
            detected_circles.append((x, y, r, 'Red'))                          # 탐지된 정보를 리스트에 저장
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)                       # 원의 외곽선을 초록색으로 표시
            cv2.circle(frame, (x, y), 2, (255, 255, 0), 3)                     # 원의 중심점을 노란색으로 표시
        
        
        if len(detected_circles) >= 4:                                         # 최소 4개의 원이 탐지된 경우에만 실행
            # 원들 간의 거리를 계산하여 가장 가까운 두 개의 원을 찾음
            distances = [(i, j, np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2]))) 
                         for i, (x1, y1, r1, _) in enumerate(detected_circles) 
                         for j, (x2, y2, r2, _) in enumerate(detected_circles) if i < j]                        # i와 j는 서로 다른 원의 인덱스를 의미하며, 두 원 사이의 유클리드 거리를 계산함
            distances.sort(key=lambda x: x[2])                                                                  # 계산된 거리값을 기준으로 오름차순으로 정렬함
            
            # 가장 가까운 두 원의 인덱스 가져오기
            idx1, idx2, _ = distances[0]                                                                        # 가장 짧은 거리의 두 원의 인덱스를 가져옴
            red1, red2 = sorted([detected_circles[idx1], detected_circles[idx2]], key=lambda x: x[0])           # x 좌표를 기준으로 왼쪽과 오른쪽 원을 정렬함
            
            # 남은 원들 중에서 우하단과의 거리로 red4를 선택하고 나머지는 red3로 할당
            remaining_circles = [circle for i, circle in enumerate(detected_circles) if i not in [idx1, idx2]]  # 남아있는 원의 목록을 생성함
            bottom_right = (frame.shape[1], frame.shape[0])                                                     # 우하단 좌표
            distances_to_bottom_right = [(circle, np.linalg.norm(np.array([circle[0], circle[1]]) - np.array(bottom_right))) for circle in remaining_circles]
            distances_to_bottom_right.sort(key=lambda x: x[1])
            red4, red3 = distances_to_bottom_right[0][0], distances_to_bottom_right[1][0]                       # 가장 가까운 원을 red4로, 나머지를 red3으로 할당함
            
            initial_detection_complete = True  # 초기 탐지 완료
            previous_reds = {'red1': red1, 'red2': red2, 'red3': red3, 'red4': red4}

    elif initial_detection_complete:
        red1, red2, red3, red4 = previous_reds.values()

    if initial_detection_complete:
        distance_red2_red3 = np.linalg.norm(np.array([red2[0], red2[1]]) - np.array([red3[0], red3[1]]))    # Arm의 픽셀 좌표계 유킬리드 거리 구하기
        distance_red3_red4 = np.linalg.norm(np.array([red3[0], red3[1]]) - np.array([red4[0], red4[1]]))    # Boom의 픽셀 좌표계 유킬리드 거리 구하기
        alpha = (12/distance_red2_red3 + 12/distance_red3_red4)/2                                           # cm 단위계 변환을 위한 계수 Alpa 구하기
        
        converted_red1 = (red1[0] * alpha, red1[1] * alpha, red1[2] * alpha, red1[3])                       # x,y,z에 Alpa를 곱해서 단위계 변환
        converted_red2 = (red2[0] * alpha, red2[1] * alpha, red2[2] * alpha, red2[3])
        converted_red3 = (red3[0] * alpha, red3[1] * alpha, red3[2] * alpha, red3[3])
        converted_red4 = (red4[0] * alpha, red4[1] * alpha, red4[2] * alpha, red4[3])
        
        final_red1 = (converted_red1[0] - converted_red4[0], -(converted_red1[1] - converted_red4[1])+12)      # 좌표계 변환 및 오프셋 적용                         
        final_red2 = (converted_red2[0] - converted_red4[0], -(converted_red2[1] - converted_red4[1])+12)                             
        final_red3 = (converted_red3[0] - converted_red4[0], -(converted_red3[1] - converted_red4[1])+12)                            
        final_red4 = (converted_red4[0] - converted_red4[0], -(converted_red4[1] - converted_red4[1])+12)                       

        final_red1 = (-final_red1[0],final_red1[1])                                                            # 로봇팔이 왼쪽을 보는 경우 해당 코드 적용 필요                      
        final_red2 = (-final_red2[0],final_red2[1])                             
        final_red3 = (-final_red3[0],final_red3[1])                            
        final_red4 = (-final_red4[0],final_red4[1])
        
        # 원들에 레이블 추가
        for (label, (x, y, r, _)) in zip(['red1', 'red2', 'red3', 'red4'], [red1, red2, red3, red4]):
            cv2.putText(frame, label, (x - 50, y + 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)                                      # 각 원에 레이블을 추가하여 프레임에 표시함
        for (label, (x, y, r, _), final) in zip(['red1', 'red2', 'red3', 'red4'], [red1, red2, red3, red4], [final_red1, final_red2, final_red3, final_red4]):
            cv2.putText(frame, f"({int(final[0])}, {int(final[1])})", (x - 50, y - 10),                     # 변환된 좌표로 표시
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2) 
        
        # Boom 각도 계산    
        vector_red4_to_red3 = (final_red3[0] - final_red4[0], final_red3[1] - final_red4[1])                # Boom Vector
        y_axis_vector = (0, 1)                                                                              # Y축 벡터
        dot_product = vector_red4_to_red3[0] * y_axis_vector[0] + vector_red4_to_red3[1] * y_axis_vector[1] # 벡터 내적 
        norm_v1 = np.linalg.norm(vector_red4_to_red3)                                                       # Boom Vector 크기
        norm_v2 = np.linalg.norm(y_axis_vector)                                                             # Y축 벡터 크기 
        angle_rad = np.arccos(dot_product / (norm_v1 * norm_v2))                                            # Angle 계산
        boom_angle = np.degrees(angle_rad)                                                                  # 단위 변환(라디안->도) 
        Direction_vector = vector_red4_to_red3[0]                                                           # +- 부호지정을 위해 조건 부여
        if  Direction_vector < 0:
              boom_angle = -boom_angle
              
        # Arm 각도 계산    
        vector_red3_to_red2 = (final_red2[0] - final_red3[0], final_red2[1] - final_red3[1])                    # Arm Vector
        x_axis_vector = (1, 0)                                                                                  # Y축 벡터
        dot_product_arm = vector_red3_to_red2[0] * x_axis_vector[0] + vector_red3_to_red2[1] * x_axis_vector[1] # 벡터 내적 
        norm_v3 = np.linalg.norm(vector_red3_to_red2)                                                           # Arm Vector 크기
        norm_v4 = np.linalg.norm(x_axis_vector)                                                                 # X축 벡터 크기 
        angle_rad_arm = np.arccos(dot_product_arm / (norm_v3 * norm_v4))                                        # Angle 계산
        arm_angle = np.degrees(angle_rad_arm)                                                                   # 단위 변환(라디안->도) 
        Direction_vector = vector_red3_to_red2[1]                                                               # +- 부호지정을 위해 조건 부여
        if  Direction_vector > 0:
               arm_angle = -arm_angle      
              
              
        cv2.putText(frame, f"Boom Angle: {boom_angle:.2f} degrees", (30, 50), 
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2) 
        cv2.putText(frame, f"Arm Angle: {arm_angle:.2f} degrees", (30, 70), 
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, f'End Effector Coordinate: {final_red1[0]:.2f},{final_red1[1]:.2f}', (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    else:
        cv2.putText(frame, "Detecting...", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
 
    if serial_port.in_waiting > 0:                                                                      # 아두이노에서 데이터를 보냈을 경우 수신
        received_data = serial_port.readline().decode().strip()
        
        
    if received_data:                                                                                   # 아두이노에서 받은 데이터를 프로토콜에 따라 처리     
        if received_data.startswith("SD") and received_data.endswith("E"):                              # SD로 시작하고 E로 끝나는 데이터만 수신
            parts = received_data.split(',')
            if len(parts) == 4:
                boom_angle_motor = int(parts[1])
                boom_angle_motor = float(boom_angle_motor)
                arm_angle_motor = int(parts[2])
                arm_angle_motor = float(arm_angle_motor)
                Dif_Angle_Boom =  boom_angle_motor -  boom_angle
                Dif_Angle_Arm =  arm_angle_motor - arm_angle
                Calibration_Angle_Boom =  boom_angle_motor + Dif_Angle_Boom
                Calibration_Angle_Arm =  arm_angle_motor + Dif_Angle_Arm

                cv2.putText(frame, f'Arm Angle Received from Arduino: {arm_angle_motor}', (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f'Boom Angle Received from Arduino: {boom_angle_motor}', (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f'Arm Angle Difference: {Dif_Angle_Arm}', (30, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f'Boom Angle Difference: {Dif_Angle_Boom}', (30, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    else:
            cv2.putText(frame, "Data not received", (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)    
            
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)                         # GUI와 OpenCV 창 통합
    img = Image.fromarray(frame_rgb)
    imgtk = ImageTk.PhotoImage(image=img)
    canvas.create_image(0, 0, anchor=tk.NW, image=imgtk)
    canvas.imgtk = imgtk
    root.after(10, update_frame)  
    
update_frame()
root.mainloop()
