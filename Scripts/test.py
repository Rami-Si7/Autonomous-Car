import os
import socketio
import eventlet
import numpy as np
from flask import Flask, request, jsonify
from tensorflow.keras.models import load_model
from io import BytesIO
from PIL import Image
import cv2
from keras.losses import MeanSquaredError
from ultralytics import YOLO
from concurrent.futures import ThreadPoolExecutor
import threading
from tensorflow.keras import backend as K

app = Flask(__name__)  
sio = socketio.Server()

executor = ThreadPoolExecutor(max_workers=4) # we use threadpool to handle multiple request at the same time, to  make responding time faster

yoloModel = YOLO('models/best.pt') # we use the fine-tuned yolo model on traffic signs data

parkigLotYolo = YOLO('models/parking-slot.pt') 

model = load_model('models/FINAL.h5', custom_objects={'mse': MeanSquaredError()}) # load our trained model

def print_active_threads():
    print(f"Active Threads: {threading.active_count()}")
    for thread in threading.enumerate():
        print(thread.name)

# process the image:
#    1. cropping it to get only the relevant part
#    2. converting it to YUV like in the paper
#    3. normilze the image
def preProcess(img):

    img = img[65:200,:,:]
    img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    img = cv2.GaussianBlur(img,  (3, 3), 0)
    img = cv2.resize(img, (200, 66))
    img = img/255

    return img


def merge_close_lines(lines, min_distance=60):
    """Merge lines that are close to each other."""
    merged_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        merged = False
        for merged_line in merged_lines:
            mx1, my1, mx2, my2 = merged_line
            # Calculate distance between line midpoints
            dist = np.sqrt((x1 + x2 - mx1 - mx2)**2 + (y1 + y2 - my1 - my2)**2)
            if dist < min_distance:  # If lines are close, merge them
                # Average the line points
                merged_line[0] = (x1 + mx1) // 2
                merged_line[1] = (y1 + my1) // 2
                merged_line[2] = (x2 + mx2) // 2
                merged_line[3] = (y2 + my2) // 2
                merged = True
                break
        if not merged:
            merged_lines.append([x1, y1, x2, y2])
    return merged_lines

# this function may will have a use in a future work
def find_curves(img):

    gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    print(gray_img.shape)
    gray_img = gray_img[40:170,:]
    blurred = cv2.GaussianBlur(gray_img, (7,7), 10)
    canny = cv2.Canny(blurred, 50, 150) 
    filterd = cv2.bilateralFilter(canny, 10, 75, 75)

    lines = cv2.HoughLinesP(filterd, rho=2, theta=np.pi/180, threshold=50, 
                        minLineLength=30, maxLineGap=30)
    lines = merge_close_lines(lines=lines)

    print(len(lines))
    lines_image = img[40:170,:].copy()

    max_slop = float('-inf')
    min_slop = float('inf')
    x1_max = 0
    x2_max = 0
    y1_max = 0
    y2_max = 0
    x1_min = 0
    x2_min = 0
    y1_min = 0
    y2_min = 0
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2  = line
            slope = (y2 - y1) / (x2 - x1 + 1e-6)
            if max_slop < slope:
                max_slop = slope
                x1_max = x1
                x2_max = x2
                y1_max = y1
                y2_max = y2
            if min_slop > slope:
                min_slop = slope
                x1_min = x1
                x2_min = x2
                y1_min = y1
                y2_min = y2


    color = (255, 0, 255)
    cv2.line(lines_image, (x1_max, y1_max), (x2_max, y2_max), color , 5)
    color = (255, 0, 0)
    cv2.line(lines_image, (x1_min, y1_min), (x2_min, y2_min), color , 5)
    cv2.imwrite("/Users/ramisimaan/Desktop/canny.png", lines_image)
    print("max_slop:",max_slop)
    print("min_slop:",min_slop)
    # return jsonify({'slope': max_slop if max_slop > abs(min_slop) else min_slop})



# this function handle the requests from the client regarding the steering prediction    
@app.route('/predict_steering', methods=['POST'])
def predict_steering():
    try:
        print_active_threads()
        file = request.files['image']
        image_bytes = file.read()
        file.close()
        image_np = np.array(Image.open(BytesIO(image_bytes)))
        image = preProcess(image_np)
        image = np.expand_dims(image, axis=0)
        
        # Predict steering angle in a separate thread
        future = executor.submit(model.predict, image)
        future.add_done_callback(lambda x: x.cancel() if not x.running() else None)
        steering = float(future.result())

        return jsonify({'steering': steering})
    
    except Exception as e:
        print(f"Error in Steering Prediction: {e}")
        return jsonify({'error': str(e)}), 500

def detect_midpoint(image):
    gray = cv2.cvtColor(image[350:], cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength= 50, maxLineGap=50)
    lines = merge_close_lines(lines, 200)
    if len(lines) >= 2:
        color = (255, 0, 0)
        lines_image = image[350:].copy()
        x1, y1, x2, y2 = lines[0]
        cv2.line(lines_image, (x1, y1), (x2, y2), color , 10)
        _x1, _y1, _x2, _y2 = lines[1]
        cv2.line(lines_image, (_x1, _y1), (_x2, _y2), color , 10)
        mid_x1 = int((x1 + _x1)/2)
        mid_y1 = int((y1 + _y1)/2)
        mid_x2 = int((x2 + _x2)/2)
        mid_y2 = int((y2 + _y2)/2)
        mid_point_x = int((mid_x1 + mid_x2) / 2)
        mid_point_y = int((mid_y1 + mid_y2) / 2)
        cv2.circle(lines_image, (mid_point_x, mid_point_y), 5,color , 10)
        # cv2.imwrite("/Users/ramisimaan/Desktop/canny.png", lines_image)
        return {"x": mid_point_x, "y": mid_point_y}

    else:
        print(len(lines))
        return {"x": -1, "y":-1}
    # Midpoints of the start and end points of the lines

    ### add return value
    # return {"x": mid_point_x, "y": mid_point_y}
    



# this function handle the requests from the client regarding the yolo detections    

@app.route('/predict_yolo_1', methods=['POST'])
def predict_yolo_1():
    try:
        file = request.files['image']
        image_bytes = file.read()
        file.close()
        image_np = np.array(Image.open(BytesIO(image_bytes)))
        
        # Run YOLO in a separate thread
        future = executor.submit(run_yolo_detection, yoloModel, image_np, True)
        detections = future.result()
        
        return jsonify({'detections': detections})
    
    except Exception as e:
        print(f"Error in YOLO Detection: {e}")
        return jsonify({'error': str(e)}), 500
    
@app.route('/predict_yolo_2', methods=['POST'])
def predict_yolo_2():
    try:
        file = request.files['image']
        image_bytes = file.read()
        file.close()
        image_np = np.array(Image.open(BytesIO(image_bytes)))
        
        # Run YOLO in a separate thread
        future = executor.submit(run_yolo_detection, yoloModel, image_np, False)
        detections = future.result()
        
        return jsonify({'detections': detections})
    
    except Exception as e:
        print(f"Error in YOLO Detection: {e}")
        return jsonify({'error': str(e)}), 500
    
@app.route('/predict_parkinglot_yolo', methods=['POST'])
def predict_parkinglot_yolo():
    try:
        file = request.files['image']
        image_bytes = file.read()
        file.close()
        image_np = np.array(Image.open(BytesIO(image_bytes)))
        
        # Run YOLO in a separate thread
        future = executor.submit(run_yolo_detection, parkigLotYolo, image_np, True)
        detections = future.result()
        # call detect_midpoint
        # return result
        # midpoint = detect_midpoint(image_np)
        # print(midpoint)

        return jsonify({'detections': detections})
    
    except Exception as e:
        print(f"Error in YOLO Detection: {e}")
        return jsonify({'error': str(e)}), 500

def run_yolo_detection(model, image, flag):
    # cv2.imwrite("/Users/ramisimaan/Desktop/image_yolo.png", image)
    if flag:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    detections = []
    results = model.predict(image, save=False)
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            confidence = box.conf[0]
            class_id = box.cls[0]
            class_name = model.names[int(class_id)]
            print(class_name, "  ", confidence)
            detections.append({"class": class_name, "confidence": float(confidence),
                               "bbox": [float(x1), float(y1), float(x2), float(y2)]})
    return detections


@sio.on('connect')
def connect(sid, environ):
    print(f"Client {sid} connected!")
    send_control(0, 0)

def send_control(steering, throttle):
    sio.emit('steer', data={'steering_angle': steering, 'throttle': throttle})
import math

def slope_to_degrees(slope):
    angle_radians = math.atan(slope)  # Calculate the angle in radians
    angle_degrees = math.degrees(angle_radians)  # Convert radians to degrees
    return angle_degrees


if __name__ == "__main__":

    app = socketio.WSGIApp(sio, app)  # Corrected setup
    eventlet.wsgi.server(eventlet.listen(('0.0.0.0', 4567)), app, log_output=False)




