# Networking
from flask import Flask, request, jsonify
# ML
import io
import torch
import torchvision.transforms as transforms
from PIL import Image
import pickle
import time

app = Flask(__name__)

device = torch.device('mps' if torch.has_mps else 'cpu')
model = torch.hub.load('ultralytics/yolov5', 'custom', path='model/model.pt', device=device)
gpu_transform = transforms.ToTensor()

IP = '192.168.0.176'
PORT = 8000


@app.route('/detect_objects', methods=['POST'])
def detect_objects():
    # Extract the incoming image
    try:
        # Get the image from the request and unpickle it
        img = pickle.loads(request.data)
        # Convert img to ndarray
        img = Image.open(io.BytesIO(img))
        # Convert the image to a pytorch tensor
        img = gpu_transform(img).unsqueeze(0).to(device)
        
        starttime = time.time()
        # Run the model on the image
        pred = model(img)
        pred.print() # profiling

        response = None        
        if len(pred):
            sizes = (pred[:, 2] - pred[:, 0]) * (pred[:, 3] - pred[:, 1])
            idx = sizes.argmax().item()
            
            x1, y1, x2, y2, _, label = pred[idx]
            # Float32MultiArray
            response = (x1, y1, x2, y2, label)

        endtime = time.time()
        print(endtime - starttime)

        pred.show() # display
        print(pred.xywh) # print
        return jsonify(response)
        
    except Exception as e:
        print(e)
        return "Could not process the image", 400


if __name__ == '__main__':
    app.run(port=8000, debug=True, host='0.0.0.0')