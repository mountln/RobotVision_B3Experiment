import numpy as np
import cv2
import serial
import time
from pid import PID
import path


# Print the HSV value of the pixel by clicking
def mouse_event(event, x, y, _, __):
    if event == cv2.EVENT_MOUSEMOVE:
        global target_x, target_y
        target_x, target_y = x, y


# Init camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)  # Set cap buffer size
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Allow 640x480, 320x240, 160x120
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cv2.namedWindow('webcam', cv2.WINDOW_NORMAL)
cv2.setMouseCallback('webcam', mouse_event)
cv2.moveWindow('webcam', 40, 0)

CENTER_X, CENTER_Y = 160, 70  # The central position of board
SERVO_A_INIT, SERVO_B_INIT = 68, 64  # Servo parameters

target_x, target_y = CENTER_X, CENTER_Y
ball_x, ball_y = CENTER_X, CENTER_Y
pid_x_v = PID(0.12, 0, 0.03)
pid_x_d = PID(2.9, 0, 0)
pid_x_d.output_limits = (-18, 18)
pid_y_v = PID(0.23, 0, 0.07)
pid_y_d = PID(2.7, 0, 0)
pid_y_d.output_limits = (-20, 20)

ser = serial.Serial("/dev/cu.usbmodem14101", 115200, timeout=1)
time.sleep(2)
ser.write(bytes([255, 23, 88, 89, 0]))
ser.write(bytes([255, 24, SERVO_A_INIT, 0]))
ser.write(bytes([255, 25, SERVO_B_INIT, 0]))

path_generator = path.get_path_generator(
    "eight", 9, (CENTER_X - 15, CENTER_Y + 10), 80)
while True:
    target_x, target_y = path_generator()
    _, frame = cap.read()
    # Perspective transform
    map_matrix = cv2.getPerspectiveTransform(
        np.float32([[50, 55], [38, 204], [281, 205], [269, 57]]),
        np.float32([[0, 0], [0, 150], [300, 150], [300, 0]]))
    frame = cv2.warpPerspective(frame, map_matrix, (300, 150))
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(frameHSV, (0, 180, 0), (180, 255, 255))

    # Find contours from the binary image
    # Result is an array of contour info
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    # Compute sizes (areas) of the contours found
    areas = [cv2.contourArea(c) for c in contours]
    cv2.circle(frame, (target_x, target_y), 5, (255, 0, 0),
               thickness=-1)  # draw target position

    # Draw the largest contour (if any) in green and its center in yellow
    if len(areas) > 0:
        max_index = np.argmax(areas)
        c = contours[max_index]
        cv2.drawContours(frame, [c], 0, (127, 255, 127), 2)

        # Compute the center of the contour
        M = cv2.moments(c)
        if M["m00"] != 0:
            pre_ball_x, pre_ball_y = ball_x, ball_y
            ball_x = int(M["m10"] / M["m00"])
            ball_y = int(M["m01"] / M["m00"])
            cv2.circle(frame, (ball_x, ball_y), 4, (127, 255, 255), -1)

            # calculate the velocity of the ball
            velocity_x = ball_x - pre_ball_x
            velocity_y = ball_y - pre_ball_y

            # if ball_x > CENTER_X and ball_y < CENTER_Y + 5:
            #     target_x, target_y = CENTER_X - 95, CENTER_Y - 20
            # elif ball_x < CENTER_X and ball_y < CENTER_Y - 5:
            #     target_x, target_y = CENTER_X - 95, CENTER_Y + 20
            # elif ball_x < CENTER_X and ball_y > CENTER_Y - 5:
            #     target_x, target_y = CENTER_X + 60, CENTER_Y + 20
            # elif ball_x > CENTER_X and ball_y > CENTER_Y + 5:
            #     target_x, target_y = CENTER_X + 60, CENTER_Y - 20

            pid_x_v.setpoint, pid_y_v.setpoint = target_x, target_y
            pid_x_d.setpoint = pid_x_v(ball_x)
            x_degree = pid_x_d(velocity_x)
            pid_y_d.setpoint = pid_y_v(ball_y)
            y_degree = pid_y_d(velocity_y)
            servo_a = int(SERVO_A_INIT - x_degree - y_degree)
            servo_b = int(SERVO_B_INIT + x_degree - y_degree)
            ser.write(bytes([255, 24, servo_a, 0]))
            ser.write(bytes([255, 25, servo_b, 0]))

    # Display the resulting frames
    cv2.imshow('webcam', frame)

    # Press 'q' in the frame window to exit loop
    key_press = cv2.waitKey(1) & 0xFF
    if key_press == ord('q'):
        print(pid_y_v.Kd)
        print(pid_y_d.Kd)
        break
    elif key_press == ord('w'):
        pid_y_v.Kd += 0.01
        pid_y_v.reset()
        print(pid_y_v.Kd)
    elif key_press == ord('s'):
        pid_y_v.Kd -= 0.01
        pid_y_v.reset()
        print(pid_y_v.Kd)
    elif key_press == ord('i'):
        pid_y_d.Kd *= 10
        pid_y_d.reset()
        print(pid_y_d.Kd)
    elif key_press == ord('k'):
        pid_y_d.Kd /= 10
        pid_y_d.reset()
        print(pid_y_d.Kd)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
