import os
import cv2
from roboflow import Roboflow
rf = Roboflow(api_key="Y4UDL12Hp9eN5einnk11")
project = rf.workspace().project("tool_new")
model = project.version(4).model

w = 1280
h = 720
# infer on a local image
path = '/home/cam/Tool_detection/test'
img_list = os.listdir(path)
for i in img_list:
    result = model.predict(os.path.join(path, i), confidence=40, overlap=30).json()
    img = cv2.imread(os.path.join(path, i))
    print(result)
    for obj in result['predictions']:
        x_min = int(obj['x'] - obj['width']/2)
        x_max = int(obj['x'] + obj['width']/2)
        y_min = int(obj['y'] - obj['height']/2)
        y_max = int(obj['y'] + obj['height']/2)
        cv2.circle(img, (int(obj['x']), int(obj['y'])), 5, (255, 0, 0), 5)
        cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        cv2.putText(img, obj['class'], (x_min - 5, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imwrite(os.path.join('/home/cam/Tool_detection/test_result', i), cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
