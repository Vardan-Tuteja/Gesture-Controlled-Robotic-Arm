import cv2
import mediapipe as mp
import time
import serial

# Configurations
write_video = True
debug = True
cam_source = 0  # 0, 1 for USB cam, "http://192.168.1.165:4747/video" for webcam
x_min, x_mid, x_max = 0, 75, 150
palm_angle_min, palm_angle_mid = -50, 20
y_min, y_mid, y_max = 0, 90, 180
wrist_y_min, wrist_y_max = 0.3, 0.9
z_min, z_mid, z_max = 10, 90, 180
plam_size_min, plam_size_max = 0.1, 0.3
claw_open_angle, claw_close_angle = 60, 0
servo_angle = [x_mid, y_mid, z_mid, claw_open_angle]
prev_servo_angle = servo_angle.copy()  # Use copy to avoid reference issues
fist_threshold = 7

# Serial communication setup
def setup_serial_connection():
    try:
        ser = serial.Serial('COM3', 9600, timeout=1)
        print("Successfully connected to COM3")
        return ser
    except (OSError, serial.SerialException) as e:
        print(f"Failed to connect to COM8: {e}")
        return None

ser = setup_serial_connection()

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

# Initialize video capture
cap = cv2.VideoCapture(cam_source)

if write_video:
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 60.0, (640, 480))

clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
map_range = lambda x, in_min, in_max, out_min, out_max: int(abs((x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min))

def is_fist(hand_landmarks, palm_size):
    distance_sum = 0
    WRIST = hand_landmarks.landmark[0]
    for i in [7, 8, 11, 12, 15, 16, 19, 20]:
        distance_sum += ((WRIST.x - hand_landmarks.landmark[i].x) ** 2 +
                         (WRIST.y - hand_landmarks.landmark[i].y) ** 2 +
                         (WRIST.z - hand_landmarks.landmark[i].z) ** 2) ** 0.5
    return distance_sum / palm_size < fist_threshold

def landmark_to_servo_angle(hand_landmarks):
    servo_angle = [x_mid, y_mid, z_mid, claw_open_angle]
    WRIST = hand_landmarks.landmark[0]
    INDEX_FINGER_MCP = hand_landmarks.landmark[5]
    palm_size = ((WRIST.x - INDEX_FINGER_MCP.x) ** 2 + (WRIST.y - INDEX_FINGER_MCP.y) ** 2 +
                 (WRIST.z - INDEX_FINGER_MCP.z) ** 2) ** 0.5
    
    servo_angle[3] = claw_close_angle if is_fist(hand_landmarks, palm_size) else claw_open_angle
    
    distance = palm_size
    angle = (WRIST.x - INDEX_FINGER_MCP.x) / distance
    angle = int(angle * 180 / 3.1415926)
    angle = clamp(angle, palm_angle_min, palm_angle_mid)
    servo_angle[0] = map_range(angle, palm_angle_min, palm_angle_mid, x_max, x_min)
    
    wrist_y = clamp(WRIST.y, wrist_y_min, wrist_y_max)
    servo_angle[1] = map_range(wrist_y, wrist_y_min, wrist_y_max, y_max, y_min)
    
    palm_size = clamp(palm_size, plam_size_min, plam_size_max)
    servo_angle[2] = map_range(palm_size, plam_size_min, plam_size_max, z_max, z_min)
    
    return servo_angle

def send_to_serial(servo_angle):
    if ser is not None:
        try:
            data = f"{servo_angle[0]},{servo_angle[1]},{servo_angle[2]},{servo_angle[3]}\n"
            ser.write(data.encode())
            print("Data sent to serial:", data)
        except Exception as e:
            print("Error occurred while sending data to serial:", e)
    else:
        print("Serial connection not available. Data not sent.")

def process_frame(frame, hands):
    global prev_servo_angle
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            servo_angle = landmark_to_servo_angle(hand_landmarks)
            if servo_angle != prev_servo_angle:
                print("Servo angle: ", servo_angle)
                prev_servo_angle = servo_angle.copy()
                send_to_serial(servo_angle)
            
            mp_drawing.draw_landmarks(
                image,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style())
    
    image = cv2.flip(image, 1)
    cv2.putText(image, str(prev_servo_angle), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    return image

def main():
    hands = mp.solutions.hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5)
    
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue
        
        processed_frame = process_frame(frame, hands)
        cv2.imshow('MediaPipe Hands', processed_frame)
        
        if write_video:
            out.write(processed_frame)
        
        if cv2.waitKey(5) & 0xFF == 27:
            break
        
        time.sleep(0.15)  # Send data every 150ms (adjust as needed)
    
    cap.release()
    if write_video:
        out.release()
    if ser is not None:
        ser.close()  # Close the serial connection when done
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()