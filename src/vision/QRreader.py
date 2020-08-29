import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar

def QRdraw_write(frame,points,data):
        QRpoint0 = points[0]
        QRpoint1 = points[1]
        QRpoint2 = points[2]
        QRpoint3 = points[3]

        cv2.line(frame, QRpoint0, QRpoint1, [0, 255, 0], 2)
        cv2.line(frame, QRpoint1, QRpoint2, [0, 255, 0], 2)
        cv2.line(frame, QRpoint2, QRpoint3, [0, 255, 0], 2)
        cv2.line(frame, QRpoint3, QRpoint0, [0, 255, 0], 2)

        cv2.putText(frame, data, (100, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 255), 3)

cap = cv2.VideoCapture(1)
while True:
        ret, frame = cap.read()
        #frame = cv2.flip(frame,1)

        decodedObjects = pyzbar.decode(frame)
        for obj in decodedObjects:

                QRdata = str(obj.data.decode('ascii'))
                QRpoints = obj.polygon
                print(QRpoints)
                QRdraw_write(frame,QRpoints,QRdata)

        cv2.imshow("Live", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break
