# python object_detection_webcam-Copy1.py --prototxt MobileNetSSD_deploy.prototxt.txt --model MobileNetSSD_deploy.caffemodel
# Credits: https://www.pyimagesearch.com/2017/09/11/object-detection-with-deep-learning-and-opencv/


#Alunos: Fernando Fincatti, Ellen Shen e Gabriela Boriero
#Deteccao de Garrafas

import numpy as np
import argparse
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-p", "--prototxt", required=True,
	help="path to Caffe 'deploy' prototxt file")
ap.add_argument("-m", "--model", required=True,
	help="path to Caffe pre-trained model")
ap.add_argument("-c", "--confidence", type=float, default=0.2,
	help="minimum probability to filter weak detections")
args = vars(ap.parse_args())

CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
	"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
	"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
	"sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])



def detect(frame):
    image = frame.copy()
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)


    print("[INFO] computing object detections...")
    net.setInput(blob)
    detections = net.forward()

    results = []

    for i in np.arange(0, detections.shape[2]):

        confidence = detections[0, 0, i, 2]

        if confidence > args["confidence"]:

            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")


            label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
            print("[INFO] {}".format(label))
            cv2.rectangle(image, (startX, startY), (endX, endY),
                COLORS[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(image, label, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

            results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY) ))

    return image, results






import cv2

cap = cv2.VideoCapture(0)

print("Known classes")
print(CLASSES)


contador = 0
while(True):
    ret, frame = cap.read()
    
    result_frame, result_tuples = detect(frame)
    print(result_tuples)
    i=0
    achou = False
    while i in range(len(result_tuples)):
        if result_tuples[i][0] == "bottle":
            achou = True
            break
        else:
            i += 1
            achou = False
    
    if achou == True:
        contador += 1
    elif achou == False:
        contador = 0
    
    if achou == True:
        for i in result_tuples:
            if i[0] == "bottle" and contador >= 5:
                cv2.rectangle(frame, i[2], i[3],
                    255, 30)
            
    print(contador)


    cv2.imshow('frame',frame)

    # Prints the structures results:
    # Format:
    # ("CLASS", confidence, (x1, y1, x2, y3))
#    for t in result_tuples:
#        print(t)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
