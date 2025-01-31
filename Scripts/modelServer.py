import os
import socketio
import eventlet
import numpy as np
from flask import Flask, request, jsonify
from tensorflow.keras.models import load_model
from keras.losses import MeanSquaredError
import cv2
from io import BytesIO
import base64
import time 

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

# Flask app and SocketIO server
sio = socketio.Server()
app = Flask(__name__)
maxSpeed = 10

# Load Steering Prediction Model
model = load_model('/Users/ramisimaan/project-in-advabced-robotics/Scripts/model-20.h5', custom_objects={'mse': MeanSquaredError()})

def preProcess(img):

    img = img[80:300,:,:]
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    img = cv2.GaussianBlur(img,  (3, 3), 0)
    img = cv2.resize(img, (200, 66))
    img = img/255
    # print("min:" ,np.min(img))
    # plt.imshow(img)
    return img

@app.route('/predict', methods=['POST'])
def predict():
    try:
        if 'image' not in request.files:
            return jsonify({'error': 'No image provided'}), 400

        file = request.files['image']
        print(f"Received file: {file.filename}")

        # Generate a unique file name using a timestamp
        temp_image_path = f"temp_image_{int(time.time() * 1000)}.png"
        
        # Save the uploaded file temporarily
        file.save(temp_image_path)



        # Use mpimg.imread to read the image
        image = cv2.imread(temp_image_path)
        image = np.asarray(image)
        image = preProcess(image)
        image = np.array([image])

        # Predict using the model
        steering = float(model.predict(image))
        throttle = 25.0  # Example throttle logic
        os.remove(temp_image_path)

        return jsonify({'steering': steering, 'throttle': throttle})
    except Exception as e:
        print(f"Error: {e}")
        return jsonify({'error': str(e)}), 500
    
if __name__ == '__main__':
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
