import rospkg
import numpy as np
from sklearn import svm
#from sklearn import joblib
import joblib
import sys

ModelFile = 'trained_model/imu_svm.pkl'

class SVM:
    def __init__(self, predictions=False, training=False, f=None):
        if training:
            self.train(f)

        if predictions:
            self.clf = joblib.load(ModelFile)

    def train(self, data):
        t = [line.strip() for line in open(data,"r")]
        points = list(list())
        for x in t:
            points.append(x.split())
        X, y = self.split_data(points)
        classifier = svm.SVC(kernel='linear', C = 1.0)
        classifier.fit(X,y)
        joblib.dump(classifier, ModelFile)

    def split_data(self, data):
        #x=[acceleration, ang velocity]
        #y=class (ie 0=didn't collide 1=did collide)
        X, y = [], []
        for line in data:
            # acceleration = data[0:3], ang_velocity = data[3:6] OR vice versa
            X.append(line[0:6])
            try:
                y.append(line[7])
            except IndexError:
                y.append(0)
        return [np.array(X), y]

    def predict(imu_data):
        if not self.clf:
            return None
        return self.clf.predict(imu_data)

def main():
    if len(sys.argv) < 2:
        print 'ERROR: Need to pass a file to train with'
        exit()
    SVM(training=True, f=sys.argv[1])

if __name__ == "__main__":
    main()
