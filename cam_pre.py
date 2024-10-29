import sys
import cv2
import time
import threading
import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont
import random
from io import BytesIO

from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class Camera:
    def __init__(self, resolution=(640, 480)):
        self.cap = None
        self.width = resolution[0]
        self.height = resolution[1]
        self.frame = None
        self.opened = False

        # Load calibration parameters
        self.param_data = np.load(calibration_param_path + '.npz')
        dim = tuple(self.param_data['dim_array'])
        k = np.array(self.param_data['k_array'].tolist())
        d = np.array(self.param_data['d_array'].tolist())
        p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim, None).copy()
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

# YOLOv5 모델 로드 및 레이블 매핑
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

model = torch.hub.load('./yolov5', 'custom', path='./best.pt', source='local')

def predict_frame(frame):
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

    return np.array(img)

# 메인 루프
if __name__ == '__main__':
    camera = Camera()
    camera.camera_open()

    while True:
        frame = camera.frame
        if frame is not None:
            result_frame = predict_frame(frame)
            cv2.imshow('YOLOv5 Predictions', cv2.cvtColor(result_frame, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) == 27:  # ESC 키로 종료
                break

    camera.camera_close()
    cv2.destroyAllWindows()