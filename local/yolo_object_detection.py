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
IP = '192.168.0.43'
PORT = 8080
# def object_position(model_res):
#     k1, b1, k2, b2 = 161434.365, 589.2512, 3.4910, -1392.257
#     res  = model_res.xyxy[0][0]
#     if k1 * res[0] + b1 > res[1] and k1 * res[2] + b1 > res[3]:
#         #It is on the oncomming traffic
#         return 1
#     else:
#         if k2 * res[0] + b2 < res[1] and k2 * res[2] + b2 < res[3]:
#             #It is on our way
#             return 2
#         else:
#             # It is on the roadside
#             return 3
def object_position(res):
    #if it is closer than 20cm
    # if (res[3] > 290):
    #     return 5
    if (res[3] > 300):
        if res[0] < 640/3 and res[2] < 640/3:
            return 1 # It is on the oncomming traffic
        else:
            if res[0] < 640*2/3 and res[2] < 640*2/3:
                return 5 # It is on our way
            else:
                return 3 # It is on the roadside
    else:
        return 4 # too far (more tham 20 cm)
        print('it is too small')
"""
{
    1: oncomming traffic,
    2: on our way,
    3: roadside,
    4: too far,
    5: alarm (way too close)
}
"""
@app.route('/detect_objects', methods=['POST'])
def detect_objects():
    # Extract the incoming image
    try:
        # Get the image from the request and unpickle it
        img = pickle.loads(request.data)
        # Convert img to ndarray
        img = Image.open(io.BytesIO(img))
        # Convert the image to a pytorch tensor
        # img = gpu_transform(img).unsqueeze(0).to(device)
        starttime = time.time()
        # Run the model on the image
        pred = model(img)
        # pred.show()
        # pred.save()
        # Boxes object for bbox outputs with probability and class
        response = (0, 0, 0, 0, 0)
        if len(pred.xyxy[0]):
            boxes = pred.xyxy[0]
            # print(boxes)
            #code for the emergency stop
            # closest = boxes[boxes[:, -1].argmax().item()]
            boxes = boxes[boxes[:, 5] == 1] #leaving only duckies
            sizes = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
            try:
                idx = sizes.argmax().item()
            except:
                return jsonify((0,0,0,0,0))
            x1, y1, x2, y2, prob, label = boxes[idx]
            # if  closest[3].item() > 420:
            # response = (closest[0].item(), closest[1].item(), closest[2].item(), closest[3].item(), 5) #emergency stop case
            if  y2.item() > 350:
                response = (x1.item(), y1.item(), x2.item(), y2.item(), 5)
                pred.show()
            else:
                # boxes = boxes[boxes[:, 5] == 1] #leaving only duckies
                # sizes = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
                # idx = sizes.argmax().item()
                # x1, y1, x2, y2, prob, label = boxes[idx]
                response = (x1.item(), y1.item(), x2.item(), y2.item(), object_position(boxes[idx]))
        print(response)
        endtime = time.time()
        # pred.show() # display
        # print(pred.xywh) # print
        return jsonify(response)
    except Exception as e:
        print(e)
        return "Could not process the image", 400
if __name__ == '__main__':
    app.run(port=8080, debug=True, host='0.0.0.0')