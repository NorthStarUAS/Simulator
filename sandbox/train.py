#!/usr/bin/python3

import cv2
from fnmatch import fnmatch
import numpy as np
import os
import random

from classifier import Classifier
from lbp import LBP

root = "/home/clolson/.nsWorld/cache/google/17"
pattern = "*.jpg"

file_list = []
for path, subdirs, files in os.walk(root):
    for name in files:
        if fnmatch(name, pattern):
            # print(os.path.join(path, name))
            file_list.append(os.path.join(path, name))
print("Found files matching %s:" % pattern, len(file_list))
# print(file_list)

random.shuffle(file_list)

model = Classifier()
model.init_model("objects")
lbp = None

label = 0

def onmouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hist = lbp.gen_classifier(y, x)
        model.add_data(label, hist)
    elif event == cv2.EVENT_RBUTTONDOWN:
        pass

for file in file_list:
    print(file)
    image = cv2.imread(file)
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    cv2.imshow("source", image)
    cv2.setMouseCallback("source", onmouse)

    lbp = LBP(gray)
    max = np.max(lbp.lbp_map)
    cv2.imshow("LBP Map", (lbp.lbp_map * 255.0 / max).astype("uint8"))

    rows, cols = gray.shape
    vectors = []
    for i in range(cols):
        for j in range(rows):
            hist = lbp.gen_classifier(j, i)
            vectors.append(hist)
    vectors = np.array(vectors)
    print(vectors.shape, vectors)

    predictions = model.model.predict(vectors)
    scores = model.model.decision_function(vectors)
    print("predictions:", predictions)
    print("scores:", scores)

    while True:
        keyb = cv2.waitKey()
        if keyb >= ord('0') and keyb <= ord('9'):
            label = keyb - ord('0')
            print("set label:", label)
        elif keyb == ord(' '):
            # go to next image
            break
        elif keyb == ord('g'):
            if show_grid == "user":
                show_grid = "prediction"
            elif show_grid == "prediction":
                show_grid = "none"
            elif show_grid == "none":
                show_grid = "user"
        elif keyb == ord('f'):
            model.update_model()
            # model.update_prediction()
        elif keyb == ord('q'):
            quit()

