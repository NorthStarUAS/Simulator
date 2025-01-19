#!/usr/bin/python3

# Machine learning (classification) module built with Linear Support Vectors

import matplotlib.pyplot as plt
import numpy as np
import os
import pickle
import sklearn.svm              # pip3 install scikit-learn
#import sklearn.linear_model              # pip3 install scikit-learn

class Classifier():

    def __init__(self):
        self.basename = None
        self.model = None
        self.cells = {}
        self.saved_labels = []
        self.saved_data = []

    def init_model(self, basename):
        if basename:
            fitname = basename + ".fit"
            dataname = basename + ".data"
        if basename and os.path.isfile(fitname):
            print("Loading SVC model from:", fitname)
            self.model = pickle.load(open(fitname, "rb"))
        else:
            print("Initializing a new SVC model")
            self.model = sklearn.svm.LinearSVC(max_iter=5000000)
        if basename and os.path.isfile(dataname):
            print("Loading saved model data from:", dataname)
            (self.saved_labels, self.saved_data) = pickle.load( open(dataname, "rb"))
        self.basename = basename

    def add_data(self, label, vector):
        self.saved_labels.append(label)
        self.saved_data.append(vector)

    # compute the grid layout and classifier
    def compute_grid(self, grid_size=160):
        (h, w) = self.lbp_map.shape[:2]
        hcells = int(h / grid_size)
        wcells = int(w / grid_size)
        self.rows = np.linspace(0, h, hcells).astype('int')
        self.cols = np.linspace(0, w, wcells).astype('int')
        self.cells = {}
        for j in range(len(self.rows)-1):
            for i in range(len(self.cols)-1):
                (r1, r2, c1, c2) = (int(self.rows[j]), int(self.rows[j+1]),
                                    int(self.cols[i]), int(self.cols[i+1]))
                key = "%d,%d,%d,%d" % (r1, r2, c1, c2)
                self.cells[key] = { "region": (r1, r2, c1, c2),
                                    "classifier": None,
                                    "user": None,
                                    "prediction": 0,
                                    "score": 0 }

        for key in self.cells:
            (r1, r2, c1, c2) = self.cells[key]["region"]
            self.cells[key]["classifier"] = self.gen_classifier(r1, r2, c1, c2)
            print(key, self.cells[key]["classifier"])

    # do the model fit
    def update_model(self):
        print("model update request")
        labels = self.saved_labels
        data = self.saved_data
        if len(set(labels)) >= 2:
            print("Updating model fit, training points:", len(data))
            self.model.fit(data, labels)
            if self.basename:
                dataname = self.basename + ".data"
                fitname = self.basename + ".fit"
                print("Saving data:", dataname)
                pickle.dump( (labels, data), open(dataname, "wb"))
                print("Saving model:", fitname)
                pickle.dump(self.model, open(fitname, "wb"))
            print("Done.")
        else:
            print("need at least two labels to create a model.")

    def update_prediction(self):
        if self.model == None:
            print("No model defined in update_prediction()")
            return
        for key in self.cells:
            cell = self.cells[key]
            cell["prediction"] = self.model.predict(cell["classifier"].reshape(1, -1))[0]
            cell["score"] = self.model.decision_function(cell["classifier"].reshape(1, -1))[0]
