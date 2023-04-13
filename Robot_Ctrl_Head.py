import cv2
import numpy as np
import math
from face_detector import get_face_detector, find_faces
from face_landmarks import get_landmark_model, detect_marks
import select
import time
import keyboard
import socket

pos_rob = [0,0,0]
Robot_Position_Flag = False

class EGM(object):

    def __init__(self, port=6511):

        self.socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('',port))
        self.send_sequence_number=0
        self.egm_addr=None
        self.count=0

    def receive_from_robot(self, timeout=0):

        s=self.socket
        s_list=[s]
        try:
            res=select.select(s_list, [], s_list, timeout)
        except select.error as err:
            if err.args[0] == errno.EINTR:
                return False, None
            else:
                raise

        if len(res[0]) == 0 and len(res[2])==0:
            return False, None
        try:
            (buf, addr)=s.recvfrom(65536)
        except:
            self.egm_addr=None
            return False, None

        self.egm_addr=addr

        robot_message=egm_pb2.EgmRobot()
        robot_message.ParseFromString(buf)

        rapid_running=False
        motors_on=False

        global Robot_Position_Flag,Robot_pos
        if robot_message.HasField('feedBack'):
            if Robot_Position_Flag == False:
                Robot_pos = robot_message.feedBack.cartesian.pos
                Robot_Position_Flag = True
        if robot_message.HasField('rapidExecState'):
            rapid_running = robot_message.rapidExecState.state == robot_message.rapidExecState.RAPID_RUNNING
        if robot_message.HasField('motorState'):
            motors_on = robot_message.motorState.state == robot_message.motorState.MOTORS_ON

        return True, Robot_pos

    def send_to_robot(self,pos,Rob_pos):

        if not self.egm_addr:
            return False

        self.send_sequence_number+=1

        sensorMessage=egm_pb2.EgmSensor()

        header=sensorMessage.header
        header.mtype=egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_CORRECTION')
        header.seqno=self.send_sequence_number
        self.send_sequence_number+=1

        planned=sensorMessage.planned

        if Rob_pos is not None:
            planned.cartesian.pos.x = Rob_pos.x + pos[2]
            planned.cartesian.pos.y = Rob_pos.y + pos[0]
            planned.cartesian.pos.z = Rob_pos.z - pos[1]
            planned.cartesian.euler.x=-Orient-40
            planned.cartesian.euler.y=180
            planned.cartesian.euler.z=0
            #print(planned.cartesian.pos)
        buf2=sensorMessage.SerializeToString()

        try:
            self.socket.sendto(buf2, self.egm_addr)
        except:
            return False

        return True

EGM1 = EGM()


def angle_calculator(x,y):
    xcm = (x * .02171875) - 6.95   #since we take value from half of the sceen. width = 13.9 cm  = .02171875 cm per pixel
    ycm = (y * .02171875) - 5.4    #since we take value from half of the sceen. height = 10.8 cm  = .02171875 cm per pixel
    ######## x values
    anglex = int(math.degrees(math.atan2(xcm,30))*10)

    angley = int(math.degrees(math.atan2(ycm,30))*10)

    return anglex,angley


face_model = get_face_detector()
landmark_model = get_landmark_model()
cap = cv2.VideoCapture(0)

font = cv2.FONT_HERSHEY_SIMPLEX
def z_calibration():
    z_cal_list = []
    for x in range(10):
        ret, img = cap.read()
        img = cv2.flip(img,1)
        if ret == True:
            faces = find_faces(img, face_model)
            for face in faces:
                marks = detect_marks(img, landmark_model, face)
                image_points = np.array([
                                        marks[30],     # Nose tip
                                        marks[8],      # Chin
                                        marks[36],     # Left eye left corner
                                        marks[45],     # Right eye right corner
                                        marks[48],     # Left Mouth corner
                                        marks[54]      # Right mouth corner
                                    ], dtype="double")
                left_eye_x = image_points[2][0]
                left_eye_y = image_points[2][1]
                right_mouth_x = image_points[5][1]
                right_mouth_y = image_points[5][1]
                dist_eye = (((left_eye_x - right_mouth_x)**2)+((left_eye_y - right_mouth_y)**2))**0.5
                z_cal_list.append(dist_eye)

    z_cal = sum(z_cal_list)/len(z_cal_list)
    z_cal = z_cal / 2
    return z_cal

is_calibration_done = False

while True:
    ret, img = cap.read()
    img = cv2.flip(img,1)
    if ret == True:
        faces = find_faces(img, face_model)
        for face in faces:
            marks = detect_marks(img, landmark_model, face)
            image_points = np.array([
                                    marks[30],     # Nose tip
                                    marks[8],      # Chin
                                    marks[36],     # Left eye left corner
                                    marks[45],     # Right eye right corner
                                    marks[48],     # Left Mouth corner
                                    marks[54]      # Right mouth corner
                                ], dtype="double")

            for p in image_points:
                cv2.circle(img, (int(p[0]), int(p[1])), 3, (0,0,255), -1)

            ang = angle_calculator(image_points[0][0],image_points[0][1])


            try:
                if keyboard.is_pressed('h'):
                    z_cal = z_calibration()
                    print("Z_CAL = ", z_cal)
                    is_calibration_done = True
            except:
                print('Wrong Key')

            if is_calibration_done == True:

                if ang[1] >= 8:
                    cv2.putText(img, 'Head Down', (30, 50), font, 1, (0,0,0), 3)
                    pos_rob[1] += -1

                elif ang[1] <= - 15:
                    cv2.putText(img, 'Head Up', (30, 50), font, 1, (0,0,0), 3)
                    pos_rob[1] += 1

                if ang[0] >= 10:
                    cv2.putText(img, 'Head Right', (30, 100), font, 1, (0,0,0), 3)
                    pos_rob[0] += 1

                elif ang[0] <= -10:
                    cv2.putText(img, 'Head Left', (30, 100), font, 1, (0,0,0), 3)
                    pos_rob[0] += -1

                left_eye_x = image_points[2][0]
                left_eye_y = image_points[2][1]
                right_mouth_x = image_points[5][1]
                right_mouth_y = image_points[5][1]
                dist_eye = (((left_eye_x - right_mouth_x)**2)+((left_eye_y - right_mouth_y)**2))**0.5
                dist_eye = dist_eye / 2

                diff_Z = - dist_eye + z_cal
                print("Z_cal : ",z_cal , " Dist : ", diff_Z)

                if diff_Z > 0.2*z_cal:
                    cv2.putText(img, 'Head Forward', (30, 150), font, 1, (0,0,0), 3)
                    pos_rob[0] += -1
                elif diff_Z <-0.2*z_cal:
                    cv2.putText(img, 'Head Back', (30, 150), font, 1, (0,0,0), 3)
                    pos_rob[0] += 1


                print('X:',pos_rob[0],'; Y:',pos_rob[1])



        cv2.line(img, (320,150),(320,330) , (0, 255, 0), 1)
        cv2.line(img, (220,240),(420,240) , (0, 255, 0), 1)

        cv2.imshow('img', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cv2.destroyAllWindows()
cap.release()
