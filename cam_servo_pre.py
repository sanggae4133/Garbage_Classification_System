import sys
import time
import threading
import cv2
import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont
import random
from tkinter import Tk, Button, Label, font
import RPi.GPIO as GPIO

from CameraCalibration.CalibrationConfig import *


# Set GPIO mode and servo settings
GPIO.setmode(GPIO.BCM)

SERVO_PIN_MAP = {
    "garbage": 2,
    "plastic": 4,
    "glass": 6,
    "can": 27,
    "paper": 25
}

CATEGORY_MAP = {
    "Cup": "garbage",
    "Carton": "paper",
    "Styrofoam piece": "plastic",
    "Plastic gloves": "plastic",
    "Broken glass": "glass",
    "Battery": "garbage",
    "Straw": "plastic",
    "Blister pack": "garbage",
    "Plastic bag & wrapper": "plastic",
    "Paper": "paper",
    "Can": "can",
    "Lid": "plastic",
    "Plastic container": "plastic",
    "Plastic utensils": "plastic",
    "Bottle cap": "plastic",
    "Paper bag": "paper",
    "Rope & strings": "plastic",
    "Food waste": "garbage",
    "Scrap metal": "can",
    "Bottle": "plastic",
    "Glass jar": "glass",
    "Squeezable tube": "garbage",
    "Aluminium foil": "can",
    "Clear plastic bottle": "plastic",
    "Other plastic bottle": "plastic",
    "Glass bottle": "glass"
}

label_mapping = {
    "Cup": "컵(도자기)",
    "Carton": "종이박스",
    "Styrofoam piece": "스티로폼",
    "Plastic gloves": "비닐 장갑",
    "Broken glass": "깨진 유리",
    "Battery": "배터리",
    "Straw": "빨대",
    "Blister pack": "약 껍질",
    "Plastic bag & wrapper": "플라스틱 포장지",
    "Paper": "종이",
    "Can": "캔",
    "Lid": "플라스틱 뚜껑",
    "Plastic container": "플라스틱 용기",
    "Plastic utensils": "플라스틱 일회용 식기",
    "Bottle cap": "병 뚜껑",
    "Paper bag": "종이 가방",
    "Rope & strings": "로프 및 끈",
    "Food waste": "음식물 쓰레기",
    "Scrap metal": "고철",
    "Bottle": "플라스틱 병",
    "Glass jar": "유리 단지",
    "Squeezable tube": "짜는 튜브",
    "Aluminium foil": "알루미늄 호일",
    "Clear plastic bottle": "투명 플라스틱 병",
    "Other plastic bottle": "기타 플라스틱 병",
    "Glass bottle": "유리병"
}

advice_mapping = {
    "Cup": "도자기 컵은 깨졌을 경우 신문지로 감싸 버려주세요",
    "Carton": "종이박스는 접어서 부피를 줄여 버려주세요",
    "Styrofoam piece": "스티로폼은 깨끗히 닦아 조각내 버려주세요",
    "Plastic gloves": "비닐 장갑은 플라스틱에 버려주세요",
    "Broken glass": "깨진 유리는 신문지로 감싸 버려주세요",
    "Battery": "폐건전지는 가능한 전용 수거함에 버려주세요\n\'스마트 서울 앱\'에서 근처 전용 수거함을 확인할 수 있습니다",
    "Straw": "빨대는 깨끗히 닦아 버려주세요",
    "Blister pack": "약 껍질은 플라스틱과 호일 분리가 어렵다면 일반쓰레기로 배출해주세요",
    "Plastic bag & wrapper": "플라스틱 포장지는 플라스틱에 버려주세요",
    "Paper": "종이는 부피를 줄여 접어 버려주세요",
    "Can": "캔은 구겨서 버려주세요",
    "Lid": "플라스틱 뚜껑은 플라스틱에 버려주세요",
    "Plastic container": "플라스틱 용기는 깨끗히 씻어 버려주세요",
    "Plastic utensils": "플라스틱 식기는 깨끗히 씻어 버려주세요",
    "Bottle cap": "병 뚜껑은 깨끗히 씻어 버려주세요",
    "Paper bag": "종이 가방은 접어 부피를 줄여 버려주세요",
    "Rope & strings": "로프 및 끈은 재질별로 분리해 잘게 잘라 버려주세요",
    "Food waste": "음식물 쓰레기는 음식물 쓰레기 전용에 버려주세요",
    "Scrap metal": "고철은 캔에 버려주세요",
    "Bottle": "플라스틱 병은 플라스틱에 버려주세요",
    "Glass jar": "유리 단지가 깨졌다면 신문지로 감싸 버려주세요",
    "Squeezable tube": "짜는 튜브는 내용물을 모두 비우고 일반쓰레기에 버려주세요",
    "Aluminium foil": "알루미늄 호일은 깨끗히 씻어 캔에 버려주세요",
    "Clear plastic bottle": "투명 플라스틱 병은 라벨을 분리하고 깨끗히 씻어 구겨서 버려주세요",
    "Other plastic bottle": "기타 플라스틱 병은 라벨을 분리하고 깨끗히 씻어 구겨서 버려주세요",
    "Glass bottle": "유리병이 깨졌다면 신문지로 감싸 버려주세요"
}

SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3
servo_motors = {}

# YOLOv5 모델 로딩
model = torch.hub.load('./yolov5', 'custom', path='./best.pt', source='local')

# 서보모터 초기화
def initialize_servos():
    for servo_type, pin in SERVO_PIN_MAP.items():
        GPIO.setup(pin, GPIO.OUT)
        motor = GPIO.PWM(pin, 50)
        motor.start(0)
        servo_motors[pin] = motor

# 서보모터 위치 설정
def setServoPos(servo, degree):
    duty = SERVO_MIN_DUTY + (degree * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)
    servo.ChangeDutyCycle(duty)

# 서보모터 작동
def operate_servo(category):
    servo_type = CATEGORY_MAP.get(category)
    pin = SERVO_PIN_MAP.get(servo_type)
    if pin in servo_motors:
        print(f"Operating servo for {category} ({servo_type}) on pin {pin}")
        
        # 서보모터를 90도로 이동
        setServoPos(servo_motors[pin], 90)

        # 10초 후 서보모터를 0도로 복귀
        root.after(10000, lambda: setServoPos(servo_motors[pin], 0))

def open_servo(pin):
    if pin in servo_motors:
        setServoPos(servo_motors[pin], 90)

def close_servo(pin):
    if pin in servo_motors:
        setServoPos(servo_motors[pin], 0)

# 카메라 클래스
class Camera:
    def __init__(self, resolution=(640, 480)):
        self.cap = None
        self.width = resolution[0]
        self.height = resolution[1]
        self.frame = None
        self.opened = False
        
        #加载参数(load parameter)
        self.param_data = np.load(calibration_param_path + '.npz')
        #获取参数(get parameter)
        dim = tuple(self.param_data['dim_array'])
        k = np.array(self.param_data['k_array'].tolist())
        d = np.array(self.param_data['d_array'].tolist())
        p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim ,None).copy()
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), p, dim, cv2.CV_16SC2)
        
        self.th = threading.Thread(target=self.camera_task, args=(), daemon=True)
        self.th.start()

    def camera_open(self, correction=False):
        try:
            self.cap = cv2.VideoCapture(-1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_SATURATION, 40)
            self.correction = correction
            self.opened = True
        except Exception as e:
            print('카메라 오픈 에러 발생:', e)

    def camera_close(self):
        try:
            self.opened = False
            time.sleep(0.2)
            if self.cap is not None:
                self.cap.release()
                time.sleep(0.05)
            self.cap = None
        except Exception as e:
            print('카메라 close 에러 발생:', e)

    def camera_task(self):
        while True:
            try:
                if self.opened and self.cap.isOpened():
                    ret, frame_tmp = self.cap.read()
                    if ret:
                        frame_resize = cv2.resize(frame_tmp, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
                        
                        if self.correction:
                            self.frame = cv2.remap(frame_resize, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                        else:
                            self.frame = frame_resize
                            
                        ret = False
                    else:
                        self.frame = None
                        self.cap.release()
                        cap = cv2.VideoCapture(-1)
                        ret, _ = cap.read()
                        if ret:
                            self.cap = cap
                elif self.opened:
                    self.cap.release()
                    cap = cv2.VideoCapture(-1)
                    ret, _ = cap.read()
                    if ret:
                        self.cap = cap              
                else:
                    time.sleep(0.01)
            except Exception as e:
                print('카메라 작동 오류:', e)
                time.sleep(0.01)

def predict_frame(frame, isServo, label_list):
    img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    results = model(img)
    boxes = results.xyxy[0][:, :4].cpu().numpy()
    labels = results.xyxy[0][:, -1].cpu().numpy()

    class_colors = {label: (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for label in set(labels)}

    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype(font="./font/SpoqaHanSansNeo-Regular.otf", size=70)
    except IOError:
        font = ImageFont.load_default()

    for box, label in zip(boxes, labels):
        xmin, ymin, xmax, ymax = box
        color = class_colors[label]
        draw.rectangle([xmin, ymin, xmax, ymax], outline=color, width=10)

        label_text = label_mapping[results.names[int(label)]]
        text_bbox = draw.textbbox((xmin, ymin), label_text, font=font)
        text_background = [xmin, ymin - (text_bbox[3] - text_bbox[1]), xmin + (text_bbox[2] - text_bbox[0]), ymin]
        draw.rectangle(text_background, fill=color)
        draw.text((xmin, ymin - (text_bbox[3] - text_bbox[1])), label_text, fill="black", font=font)

        if isServo == 1:
            label_list.append(results.names[int(label)])

    return np.array(img)


# 버튼 클릭 이벤트
def on_button_click():
    global camera
    global isFreeze
    global currentServoPinList

    frame = camera.frame

    if isFreeze == 0:
        isFreeze = 1

        if frame is not None:
            labelList = []
            adviceMsgList = []

            predict_frame(frame, 1, labelList)

            for label in labelList:
                adviceMsgList.append(advice_mapping[label])

                servo_type = CATEGORY_MAP.get(label)
                pin = SERVO_PIN_MAP.get(servo_type)
                currentServoPinList.append(pin)
            
            adviceMsgList = set(adviceMsgList)
            adviceMsgList = list(adviceMsgList)
            currentServoPinList = set(currentServoPinList)
            currentServoPinList = list(currentServoPinList)
        
            # 메시지를 줄바꿈으로 구분하여 출력
            full_message = "[감지된 쓰레기의 분리수거 방법]\n" + "\n".join(adviceMsgList)
            update_message(full_message)

            for pin in currentServoPinList:
                open_servo(pin)

    else:
        for pin in currentServoPinList:
            close_servo(pin)
        currentServoPinList = []
        update_message("버튼을 누르면 감지된 쓰레기통 뚜껑이 열립니다.")
        isFreeze = 0

# 메시지 텍스트 업데이트 함수
def update_message(text):
    global message_label
    message_label.config(text=text)


# 메인 함수
if __name__ == '__main__':
    camera = Camera()

    try:
        initialize_servos()
        
        camera.camera_open()

        root = Tk()
        root.title("쓰레기통 컨트롤러")
        root.geometry("600x400")

        # 버튼 폰트 크기와 패딩 설정
        button_font = font.Font(size=16, weight="bold")
        test_button = Button(root, text="뚜껑 작동", command=on_button_click, font=button_font)
        test_button.pack(pady=40, padx=20)  # 더 넓은 패딩을 추가하여 버튼 크기 확대

        # 메시지 레이블 폰트 크기 설정
        message_label = Label(root, text="버튼을 누르면 감지된 쓰레기통 뚜껑이 열립니다.", font=("Arial", 16))
        message_label.pack(pady=20)  # 레이블의 위아래에 패딩 추가

        isFreeze = 0

        currentServoPinList = []

        def display_camera():
            while True:
                frame = camera.frame
                if frame is not None and isFreeze == 0:
                    result_frame = predict_frame(frame, 0, None)
                    cv2.imshow('YOLOv5 Predictions', cv2.cvtColor(result_frame, cv2.COLOR_RGB2BGR))
                    if cv2.waitKey(1) == 27:
                        break
            camera.camera_close()
            cv2.destroyAllWindows()

        threading.Thread(target=display_camera, daemon=True).start()

        root.mainloop()

    except Exception as e:
        print(f"Error: {e}")

    finally:
        for motor in servo_motors.values():
            motor.stop()
        GPIO.cleanup()
        camera.camera_close()






