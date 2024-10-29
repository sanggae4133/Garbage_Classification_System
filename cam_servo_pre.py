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
    "Cup": "plastic",
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
    "Cup": "컵",
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
    "Lid": "뚜껑",
    "Plastic container": "플라스틱 용기",
    "Plastic utensils": "플라스틱 식기",
    "Bottle cap": "병 뚜껑",
    "Paper bag": "종이 가방",
    "Rope & strings": "로프 및 끈",
    "Food waste": "음식물 쓰레기",
    "Scrap metal": "고철",
    "Bottle": "병",
    "Glass jar": "유리병",
    "Squeezable tube": "짜는 튜브",
    "Aluminium foil": "알루미늄 호일",
    "Clear plastic bottle": "투명 플라스틱 병",
    "Other plastic bottle": "기타 플라스틱 병",
    "Glass bottle": "유리병"
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
        setServoPos(servo_motors[pin], 90)
        time.sleep(1)
        setServoPos(servo_motors[pin], 0)

# 카메라 클래스
class Camera:
    def __init__(self, resolution=(640, 480)):
        self.cap = cv2.VideoCapture(0)
        self.width, self.height = resolution
        self.frame = None
        self.opened = True
        self.thread = threading.Thread(target=self.camera_task, daemon=True)
        self.thread.start()

    '''
    def camera_task(self):
        while self.opened:
            ret, frame = self.cap.read()
            if ret:
                self.frame = self.draw_predictions(frame)
            time.sleep(0.01)

    def draw_predictions(self, frame):
        img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        results = model(img)
        boxes = results.xyxy[0][:, :4].cpu().numpy()
        labels = results.xyxy[0][:, -1].cpu().numpy()

        draw = ImageDraw.Draw(img)
        for box, label in zip(boxes, labels):
            xmin, ymin, xmax, ymax = box
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            label_text = label_mapping.get(results.names[int(label)], "Unknown")
            draw.rectangle([xmin, ymin, xmax, ymax], outline=color, width=3)
            draw.text((xmin, ymin), label_text, fill="black")

        return np.array(img)

    def release(self):
        self.opened = False
        self.cap.release()
    '''

    def camera_open(self, correction=False):
        try:
            self.cap = cv2.VideoCapture(-1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_SATURATION, 40)
            self.correction = correction
            self.opened = True
        except Exception as e:
            print('Error opening camera:', e)

    def camera_close(self):
        try:
            self.opened = False
            time.sleep(0.2)
            if self.cap is not None:
                self.cap.release()
                time.sleep(0.05)
            self.cap = None
        except Exception as e:
            print('Error closing camera:', e)

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
                print('Error capturing camera frame:', e)
                time.sleep(0.01)

def predict_frame(frame, isServo):
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
            operate_servo(label)

    return np.array(img)


# 버튼 클릭 이벤트
def on_button_click():
    global camera
    global isFreeze

    frame = camera.frame
    isFreeze = 1

    if frame is not None:
        predict_frame(frame, 1)
        
        # 메시지 텍스트를 즉시 업데이트하고 메인 이벤트 루프를 유지합니다.
        update_message("뚜껑이 열립니다! 10초 후에 닫힙니다.")

        # 10초 후에 메시지를 복구하는 타이머 설정
        root.after(10000, lambda: update_message("버튼을 누르면 10초 동안 해당하는 쓰레기통 뚜껑이 열립니다."))

    isFreeze = 0

# 메시지 텍스트 업데이트 함수
def update_message(text):
    global message_label
    message_label.config(text=text)

# 메인 함수
if __name__ == '__main__':
    try:
        initialize_servos()

        camera = Camera()
        camera.camera_open()

        root = Tk()
        root.title("쓰레기통 컨트롤러")
        root.geometry("600x400")

        test_button = Button(root, text="뚜껑 작동", command=on_button_click)
        test_button.pack(pady=20)

        message_label = Label(root, text="버튼을 누르면 10초 동안 해당하는 쓰레기통 뚜껑이 열립니다.", font=("Arial", 12))
        message_label.pack()

        isFreeze = 0

        def display_camera():
            while True:
                frame = camera.frame
                if frame is not None and isFreeze == 0:
                    result_frame = predict_frame(frame, 0)
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






