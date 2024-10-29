import sys
import cv2
import time
import threading
import numpy as np
from CameraCalibration.CalibrationConfig import *

# tensorflow
import tensorflow.keras as keras
import tensorflow as tf
from keras.preprocessing import image
import keras.applications.mobilenet_v2 as mobilenetv2

import os

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

# ML model setup
IMAGE_WIDTH = 224
IMAGE_HEIGHT = 224
IMAGE_SIZE = (IMAGE_WIDTH, IMAGE_HEIGHT)
IMAGE_CHANNELS = 3

model_weights_path = "saved_models/model12.weights.h5"
categories = {0: 'paper', 1: 'cardboard', 2: 'plastic', 3: 'metal', 4: 'trash', 5: 'glass'}

mobilenetv2_layer = mobilenetv2.MobileNetV2(
  include_top=False,
  input_shape=(IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_CHANNELS),
  weights='imagenet'
)

mobilenetv2_layer.trainable = False

model = keras.Sequential()
model.add(keras.Input(shape=(IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_CHANNELS)))

def mobilenetv2_preprocessing(img):
    return mobilenetv2.preprocess_input(img)

model.add(keras.layers.Lambda(mobilenetv2_preprocessing))
model.add(mobilenetv2_layer)
model.add(tf.keras.layers.GlobalAveragePooling2D())
model.add(keras.layers.Dense(len(categories), activation='softmax'))

model.load_weights(model_weights_path)
print('Model loaded successfully!')

def prepare_frame(frame):
    img = cv2.resize(frame, IMAGE_SIZE)
    img_array = np.expand_dims(img, axis=0)  # Add batch dimension
    img_array = mobilenetv2.preprocess_input(img_array)  # Preprocess for MobileNetV2
    return img_array

def predict_frame(frame):
    img_array = prepare_frame(frame)
    prediction = model.predict(img_array)
    predicted_class = np.argmax(prediction, axis=1)[0]
    predicted_category = categories[predicted_class]
    return predicted_category, prediction[0]

# Main loop
if __name__ == '__main__':
    camera = Camera()
    camera.camera_open()
    while True:
        frame = camera.frame
        if frame is not None:
            # Predict frame
            category, confidence = predict_frame(frame)
            print(f'Predicted category: {category}')
            print(f'Confidence scores: {confidence}')

            # Display frame
            cv2.imshow('Camera Frame', frame)
            key = cv2.waitKey(1)
            if key == 27:  # ESC key to exit
                break
    camera.camera_close()
    cv2.destroyAllWindows()