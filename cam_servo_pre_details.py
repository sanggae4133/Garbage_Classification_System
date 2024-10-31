import sys
import time
import threading
import cv2
import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont
import random
from tkinter import Tk, Button, Label
import RPi.GPIO as GPIO

from CameraCalibration.CalibrationConfig import *

# GPIO 모드 설정 및 서보 핀 번호 정의
GPIO.setmode(GPIO.BCM)

# 각 쓰레기 카테고리에 해당하는 서보 핀 번호를 설정
SERVO_PIN_MAP = {
    "garbage": 2,   # 일반쓰레기 서보 핀
    "plastic": 4,   # 플라스틱 서보 핀
    "glass": 6,     # 유리 서보 핀
    "can": 27,      # 캔 서보 핀
    "paper": 25     # 종이 서보 핀
}

# 모델에서 분류한 쓰레기 종류를 서보 분류와 연결
CATEGORY_MAP = {
    "Cup": "garbage",              # 컵은 일반 쓰레기로 분류
    "Carton": "paper",             # 종이박스는 종이로 분류
    "Styrofoam piece": "plastic",  # 스티로폼 조각은 플라스틱으로 분류
    "Plastic gloves": "plastic",   # 비닐 장갑은 플라스틱으로 분류
    "Broken glass": "glass",       # 깨진 유리는 유리로 분류
    "Battery": "garbage",          # 배터리는 일반 쓰레기로 분류
    "Straw": "plastic",            # 빨대는 플라스틱으로 분류
    "Blister pack": "garbage",     # 약 껍질은 일반 쓰레기로 분류
    "Plastic bag & wrapper": "plastic", # 플라스틱 포장지는 플라스틱으로 분류
    "Paper": "paper",              # 종이는 종이로 분류
    "Can": "can",                  # 캔은 캔으로 분류
    "Lid": "plastic",              # 뚜껑은 플라스틱으로 분류
    "Plastic container": "plastic",# 플라스틱 용기는 플라스틱으로 분류
    "Plastic utensils": "plastic", # 플라스틱 식기는 플라스틱으로 분류
    "Bottle cap": "plastic",       # 병 뚜껑은 플라스틱으로 분류
    "Paper bag": "paper",          # 종이 가방은 종이로 분류
    "Rope & strings": "plastic",   # 로프 및 끈은 플라스틱으로 분류
    "Food waste": "garbage",       # 음식물 쓰레기는 일반 쓰레기로 분류
    "Scrap metal": "can",          # 고철은 캔으로 분류
    "Bottle": "plastic",           # 플라스틱 병은 플라스틱으로 분류
    "Glass jar": "glass",          # 유리 단지는 유리로 분류
    "Squeezable tube": "garbage",  # 짜는 튜브는 일반 쓰레기로 분류
    "Aluminium foil": "can",       # 알루미늄 호일은 캔으로 분류
    "Clear plastic bottle": "plastic", # 투명 플라스틱 병은 플라스틱으로 분류
    "Other plastic bottle": "plastic", # 기타 플라스틱 병은 플라스틱으로 분류
    "Glass bottle": "glass"        # 유리병은 유리로 분류
}

# 분류 라벨을 한글로 표시하기 위한 매핑
label_mapping = {
    "Cup": "컵(도자기)",               # Cup을 한글 '컵(도자기)'으로 표시
    "Carton": "종이박스",              # Carton을 한글 '종이박스'로 표시
    "Styrofoam piece": "스티로폼",      # Styrofoam piece를 한글 '스티로폼'으로 표시
    "Plastic gloves": "비닐 장갑",      # Plastic gloves를 한글 '비닐 장갑'으로 표시
    "Broken glass": "깨진 유리",        # Broken glass를 한글 '깨진 유리'로 표시
    "Battery": "배터리",               # Battery를 한글 '배터리'로 표시
    "Straw": "빨대",                   # Straw를 한글 '빨대'로 표시
    "Blister pack": "약 껍질",         # Blister pack을 한글 '약 껍질'로 표시
    "Plastic bag & wrapper": "플라스틱 포장지", # Plastic bag & wrapper를 한글 '플라스틱 포장지'로 표시
    "Paper": "종이",                   # Paper를 한글 '종이'로 표시
    "Can": "캔",                      # Can을 한글 '캔'으로 표시
    "Lid": "플라스틱 뚜껑",            # Lid를 한글 '플라스틱 뚜껑'으로 표시
    "Plastic container": "플라스틱 용기", # Plastic container를 한글 '플라스틱 용기'로 표시
    "Plastic utensils": "플라스틱 일회용 식기", # Plastic utensils를 한글 '플라스틱 일회용 식기'로 표시
    "Bottle cap": "병 뚜껑",           # Bottle cap을 한글 '병 뚜껑'으로 표시
    "Paper bag": "종이 가방",          # Paper bag을 한글 '종이 가방'으로 표시
    "Rope & strings": "로프 및 끈",     # Rope & strings를 한글 '로프 및 끈'으로 표시
    "Food waste": "음식물 쓰레기",      # Food waste를 한글 '음식물 쓰레기'로 표시
    "Scrap metal": "고철",             # Scrap metal을 한글 '고철'로 표시
    "Bottle": "플라스틱 병",            # Bottle을 한글 '플라스틱 병'으로 표시
    "Glass jar": "유리 단지",           # Glass jar를 한글 '유리 단지'로 표시
    "Squeezable tube": "짜는 튜브",      # Squeezable tube를 한글 '짜는 튜브'로 표시
    "Aluminium foil": "알루미늄 호일",   # Aluminium foil을 한글 '알루미늄 호일'로 표시
    "Clear plastic bottle": "투명 플라스틱 병", # Clear plastic bottle을 한글 '투명 플라스틱 병'으로 표시
    "Other plastic bottle": "기타 플라스틱 병", # Other plastic bottle을 한글 '기타 플라스틱 병'으로 표시
    "Glass bottle": "유리병"            # Glass bottle을 한글 '유리병'으로 표시
}

# 각 쓰레기 분류에 따라 분리수거 안내 메시지 설정
advice_mapping = {
    "Cup": "도자기 컵은 깨졌을 경우 신문지로 감싸 버려주세요",      # 도자기 컵에 대한 분리수거 안내
    "Carton": "종이박스는 접어서 부피를 줄여 버려주세요",           # 종이박스에 대한 분리수거 안내
    "Styrofoam piece": "스티로폼은 깨끗히 닦아 조각내 버려주세요",  # 스티로폼에 대한 분리수거 안내
    "Plastic gloves": "비닐 장갑은 플라스틱에 버려주세요",          # 비닐 장갑에 대한 분리수거 안내
    "Broken glass": "깨진 유리는 신문지로 감싸 버려주세요",         # 깨진 유리에 대한 분리수거 안내
    "Battery": "폐건전지는 가능한 전용 수거함에 버려주세요\n'스마트 서울 앱'에서 근처 전용 수거함을 확인할 수 있습니다", # 배터리 분리수거 안내
    "Straw": "빨대는 깨끗히 닦아 버려주세요",                    # 빨대 분리수거 안내
    "Blister pack": "약 껍질은 플라스틱과 호일 분리가 어렵다면 일반쓰레기로 배출해주세요", # 약 껍질 분리수거 안내
    "Plastic bag & wrapper": "플라스틱 포장지는 플라스틱에 버려주세요", # 플라스틱 포장지 분리수거 안내
    "Paper": "종이는 부피를 줄여 접어 버려주세요",               # 종이 분리수거 안내
    "Can": "캔은 구겨서 버려주세요",                            # 캔 분리수거 안내
    "Lid": "플라스틱 뚜껑은 플라스틱에 버려주세요",               # 플라스틱 뚜껑 분리수거 안내
    "Plastic container": "플라스틱 용기는 깨끗히 씻어 버려주세요", # 플라스틱 용기 분리수거 안내
    "Plastic utensils": "플라스틱 식기는 깨끗히 씻어 버려주세요", # 플라스틱 식기 분리수거 안내
    "Bottle cap": "병 뚜껑은 깨끗히 씻어 버려주세요",            # 병 뚜껑 분리수거 안내
    "Paper bag": "종이 가방은 접어 부피를 줄여 버려주세요",       # 종이 가방 분리수거 안내
    "Rope & strings": "로프 및 끈은 재질별로 분리해 잘게 잘라 버려주세요", # 로프 및 끈 분리수거 안내
    "Food waste": "음식물 쓰레기는 음식물 쓰레기 전용에 버려주세요", # 음식물 쓰레기 분리수거 안내
    "Scrap metal": "고철은 캔에 버려주세요",                   # 고철 분리수거 안내
    "Bottle": "플라스틱 병은 플라스틱에 버려주세요",             # 플라스틱 병 분리수거 안내
    "Glass jar": "유리 단지가 깨졌다면 신문지로 감싸 버려주세요", # 유리 단지 분리수거 안내
    "Squeezable tube": "짜는 튜브는 내용물을 모두 비우고 일반쓰레기에 버려주세요", # 짜는 튜브 분리수거 안내
    "Aluminium foil": "알루미늄 호일은 깨끗히 씻어 캔에 버려주세요", # 알루미늄 호일 분리수거 안내
    "Clear plastic bottle": "투명 플라스틱 병은 라벨을 분리하고 깨끗히 씻어 구겨서 버려주세요", # 투명 플라스틱 병 분리수거 안내
    "Other plastic bottle": "기타 플라스틱 병은 라벨을 분리하고 깨끗히 씻어 구겨서 버려주세요", # 기타 플라스틱 병 분리수거 안내
    "Glass bottle": "유리병이 깨졌다면 신문지로 감싸 버려주세요"    # 유리병 분리수거 안내
}

# 서보모터의 최대 및 최소 듀티 설정 (각 서보 위치 조정을 위한 값 설정)
SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3

# 서보모터 객체 저장 딕셔너리
servo_motors = {}

# YOLOv5 모델 로드 (사전 학습된 YOLOv5 모델을 로드하여 사용할 준비)
model = torch.hub.load('./yolov5', 'custom', path='./best.pt', source='local')

# 서보모터 초기화 함수 (각 서보모터 핀을 출력 모드로 설정하고 PWM 신호로 제어)
def initialize_servos():
    for servo_type, pin in SERVO_PIN_MAP.items():
        GPIO.setup(pin, GPIO.OUT)         # 서보 핀을 출력 모드로 설정
        motor = GPIO.PWM(pin, 50)         # 50Hz의 PWM 신호 생성
        motor.start(0)                    # PWM 신호 시작
        servo_motors[pin] = motor         # 서보모터 객체를 딕셔너리에 저장

# 서보모터 위치 설정 함수 (각 서보모터의 각도를 설정)
def setServoPos(servo, degree):
    duty = SERVO_MIN_DUTY + (degree * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0) # 각도에 따라 듀티 값 계산
    servo.ChangeDutyCycle(duty)  # 서보모터 듀티 설정하여 각도 조정

# 서보모터 작동 함수 (카테고리에 따라 서보모터를 작동시킴)
def operate_servo(category):
    servo_type = CATEGORY_MAP.get(category)     # 카테고리에 해당하는 서보 타입 가져오기
    pin = SERVO_PIN_MAP.get(servo_type)         # 서보 타입에 해당하는 핀 번호 가져오기
    if pin in servo_motors:
        print(f"Operating servo for {category} ({servo_type}) on pin {pin}")
        
        # 서보모터를 90도로 이동
        setServoPos(servo_motors[pin], 90)

        # 10초 후 서보모터를 0도로 복귀
        root.after(10000, lambda: setServoPos(servo_motors[pin], 0))

# 서보모터 열기 및 닫기 함수 (특정 핀의 서보모터를 열고 닫음)
def open_servo(pin):
    if pin in servo_motors:
        setServoPos(servo_motors[pin], 90)   # 지정된 핀의 서보모터를 90도로 이동

def close_servo(pin):
    if pin in servo_motors:
        setServoPos(servo_motors[pin], 0)    # 지정된 핀의 서보모터를 0도로 복귀

# 카메라 클래스 정의 (카메라 초기화 및 프레임 캡처 기능 포함)
class Camera:
    def __init__(self, resolution=(640, 480)):
        self.cap = None                       # 비디오 캡처 객체 초기화
        self.width = resolution[0]            # 설정된 너비 값 저장
        self.height = resolution[1]           # 설정된 높이 값 저장
        self.frame = None                     # 현재 프레임 초기화
        self.opened = False                   # 카메라 열림 상태를 나타내는 변수

        # 교정 파라미터 로드 및 교정 맵 생성
        self.param_data = np.load(calibration_param_path + '.npz') # 교정 파일 로드
        dim = tuple(self.param_data['dim_array'])                 # 교정 이미지 크기
        k = np.array(self.param_data['k_array'].tolist())         # 교정 행렬
        d = np.array(self.param_data['d_array'].tolist())         # 왜곡 계수
        p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim ,None).copy() # 새로운 교정 행렬 계산
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), p, dim, cv2.CV_16SC2) # 교정 맵 생성
        
        # 카메라 프레임을 계속해서 읽어오는 쓰레드 시작
        self.th = threading.Thread(target=self.camera_task, args=(), daemon=True)
        self.th.start()

    # 카메라 열기 함수 (비디오 캡처 초기화 및 교정 설정)
    def camera_open(self, correction=False):
        try:
            self.cap = cv2.VideoCapture(-1)   # 기본 카메라 장치 열기
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V')) # 비디오 형식 설정
            self.cap.set(cv2.CAP_PROP_FPS, 30)       # FPS 설정
            self.cap.set(cv2.CAP_PROP_SATURATION, 40) # 색감 설정
            self.correction = correction             # 교정 사용 여부 설정
            self.opened = True                       # 카메라 열림 상태로 설정
        except Exception as e:
            print('카메라 오픈 에러 발생:', e)        # 카메라 오픈 오류 메시지 출력

    # 카메라 닫기 함수 (비디오 캡처 해제)
    def camera_close(self):
        try:
            self.opened = False                   # 카메라 열림 상태 해제
            time.sleep(0.2)                       # 카메라 닫힘 전 대기 시간
            if self.cap is not None:
                self.cap.release()                # 비디오 캡처 해제
                time.sleep(0.05)
            self.cap = None                       # 캡처 객체 초기화
        except Exception as e:
            print('카메라 close 에러 발생:', e)     # 카메라 닫기 오류 메시지 출력

    # 프레임을 계속해서 읽어오는 카메라 작업 함수 (쓰레드 내에서 실행)
    def camera_task(self):
        while True:
            try:
                if self.opened and self.cap.isOpened():          # 카메라가 열려있는 경우
                    ret, frame_tmp = self.cap.read()             # 프레임 읽기
                    if ret:
                        frame_resize = cv2.resize(frame_tmp, (self.width, self.height), interpolation=cv2.INTER_NEAREST) # 프레임 리사이즈
                        
                        if self.correction:                      # 교정 적용 여부
                            self.frame = cv2.remap(frame_resize, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                        else:
                            self.frame = frame_resize             # 교정하지 않은 경우 리사이즈 프레임 사용
                        ret = False
                    else:
                        self.frame = None                        # 프레임을 얻지 못하면 None 할당
                        self.cap.release()
                        cap = cv2.VideoCapture(-1)               # 새 비디오 캡처 객체 생성
                        ret, _ = cap.read()
                        if ret:
                            self.cap = cap                       # 새 캡처 객체로 대체
                elif self.opened:
                    self.cap.release()
                    cap = cv2.VideoCapture(-1)
                    ret, _ = cap.read()
                    if ret:
                        self.cap = cap
                else:
                    time.sleep(0.01)
            except Exception as e:
                print('카메라 작동 오류:', e)   # 오류 발생 시 메시지 출력 후 잠시 대기
                time.sleep(0.01)

# 프레임 내 쓰레기 예측 및 분류 함수
def predict_frame(frame, isServo, label_list):
    img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)) # 프레임을 PIL 이미지로 변환
    results = model(img)                                          # YOLO 모델로 이미지 예측
    boxes = results.xyxy[0][:, :4].cpu().numpy()                  # 예측된 객체의 좌표 값
    labels = results.xyxy[0][:, -1].cpu().numpy()                 # 예측된 객체의 라벨 값

    # 객체별 고유 색상 생성 (랜덤 컬러 지정)
    class_colors = {label: (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for label in set(labels)}

    # 이미지 위에 예측 결과 그리기
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype(font="./font/SpoqaHanSansNeo-Regular.otf", size=70) # 폰트 파일이 있을 경우 로드
    except IOError:
        font = ImageFont.load_default()  # 폰트 파일이 없을 경우 기본 폰트 사용

    for box, label in zip(boxes, labels):
        xmin, ymin, xmax, ymax = box                     # 바운딩 박스 좌표
        color = class_colors[label]                      # 객체별 색상
        draw.rectangle([xmin, ymin, xmax, ymax], outline=color, width=10) # 객체 바운딩 박스 그리기

        label_text = label_mapping[results.names[int(label)]] # 객체 라벨의 한글 번역 가져오기
        text_bbox = draw.textbbox((xmin, ymin), label_text, font=font) # 텍스트 바운딩 박스 계산
        text_background = [xmin, ymin - (text_bbox[3] - text_bbox[1]), xmin + (text_bbox[2] - text_bbox[0]), ymin]
        draw.rectangle(text_background, fill=color)      # 텍스트 배경 색상 적용
        draw.text((xmin, ymin - (text_bbox[3] - text_bbox[1])), label_text, fill="black", font=font) # 객체 라벨 그리기

        if isServo == 1:                                 # 서보모터를 작동시키려는 경우
            label_list.append(results.names[int(label)]) # 라벨 리스트에 추가

    return np.array(img)                                 # 최종 이미지를 배열로 반환

# 버튼 클릭 이벤트 함수 (쓰레기 분류 후 서보모터 작동)
def on_button_click():
    global camera
    global isFreeze
    global currentServoPinList

    frame = camera.frame                                # 현재 프레임 가져오기

    if isFreeze == 0:                                   # 프레임이 동결되지 않았을 경우
        isFreeze = 1                                    # 프레임 동결 상태로 변경

        if frame is not None:
            labelList = []                              # 라벨 리스트 초기화
            adviceMsgList = []                          # 분리수거 안내 메시지 리스트 초기화

            predict_frame(frame, 1, labelList)          # 프레임 예측 실행 및 라벨 수집

            for label in labelList:
                adviceMsgList.append(advice_mapping[label]) # 라벨에 따른 분리수거 메시지 수집

                servo_type = CATEGORY_MAP.get(label)    # 쓰레기 라벨에 해당하는 서보 타입
                pin = SERVO_PIN_MAP.get(servo_type)     # 서보 타입에 해당하는 핀 번호
                currentServoPinList.append(pin)         # 현재 서보 핀 리스트에 추가
            
            adviceMsgList = set(adviceMsgList)          # 중복 제거하여 고유 메시지만 유지
            adviceMsgList = list(adviceMsgList)
            currentServoPinList = set(currentServoPinList) # 중복 제거하여 고유 핀 번호만 유지
            currentServoPinList = list(currentServoPinList)
        
            # 감지된 쓰레기의 분리수거 방법을 메시지로 출력
            full_message = "[감지된 쓰레기의 분리수거 방법]\n" + "\n".join(adviceMsgList)
            update_message(full_message)                # 메시지 UI 업데이트

            # 각 핀의 서보모터를 열기
            for pin in currentServoPinList:
                open_servo(pin)

    else:
        for pin in currentServoPinList:                 # 이미 열린 상태일 경우 모든 서보모터를 닫기
            close_servo(pin)
        currentServoPinList = []                        # 현재 서보 핀 리스트 초기화
        update_message("버튼을 누르면 감지된 쓰레기통 뚜껑이 열립니다.") # 기본 메시지 출력
        isFreeze = 0                                    # 프레임 동결 상태 해제

# 메시지 텍스트 업데이트 함수 (라벨을 사용하여 메시지 업데이트)
def update_message(text):
    global message_label
    message_label.config(text=text)                      # 라벨 텍스트 설정


# 메인 함수
if __name__ == '__main__':
    camera = Camera()                                   # 카메라 객체 생성

    try:
        initialize_servos()                             # 서보모터 초기화
        
        camera.camera_open()                            # 카메라 열기

        root = Tk()                                     # Tkinter 창 생성
        root.title("쓰레기통 컨트롤러")                 # 창 제목 설정
        root.geometry("600x400")                        # 창 크기 설정

        test_button = Button(root, text="뚜껑 작동", command=on_button_click) # 버튼 생성 및 클릭 시 이벤트 연결
        test_button.pack(pady=20)                       # 버튼을 창에 배치

        message_label = Label(root, text="버튼을 누르면 감지된 쓰레기통 뚜껑이 열립니다.", font=("Arial", 12)) # 초기 메시지 라벨
        message_label.pack()                            # 라벨을 창에 배치

        isFreeze = 0                                    # 프레임 동결 변수 초기화
        currentServoPinList = []                        # 서보모터 핀 리스트 초기화

        # 카메라 디스플레이 함수
        def display_camera():
            while True:
                frame = camera.frame                    # 현재 프레임 가져오기
                if frame is not None and isFreeze == 0: # 동결 상태가 아닌 경우
                    result_frame = predict_frame(frame, 0, None) # 프레임 예측
                    cv2.imshow('YOLOv5 Predictions', cv2.cvtColor(result_frame, cv2.COLOR_RGB2BGR)) # 예측 결과 창에 표시
                    if cv2.waitKey(1) == 27:            # 'Esc'키를 누르면 종료
                        break
            camera.camera_close()                       # 카메라 닫기
            cv2.destroyAllWindows()                     # 모든 창 닫기

        threading.Thread(target=display_camera, daemon=True).start() # 카메라 디스플레이 쓰레드 시작

        root.mainloop()                                 # Tkinter 메인 루프 실행

    except Exception as e:
        print(f"Error: {e}")                            # 예외 발생 시 오류 메시지 출력

    finally:
        for motor in servo_motors.values():             # 서보모터 종료
            motor.stop()
        GPIO.cleanup()                                  # GPIO 클린업
        camera.camera_close()                           # 카메라 종료
