#Importing required libraries
import cv2
import speech_recognition as sr
import time
import torch
from ultralytics import YOLO
import urllib.request
import numpy as np

#Initialize voice recognizer
r = sr.Recognizer()


#Converting speech to text
def listen_for_command():
    with sr.Microphone() as source:
        print('Command Please - ')
        audio = r.listen(source)

        try:
            text = r.recognize_google(audio)
            print(text)
            return text

        except:
            print('sorry could not recognize your voice')
            text = 'hello'
            return text


#Initializing PWM of motors
def move_forward():
    import requests

    #ESP32 IP address
    esp32_ip = "http://192.168.1.202"

    #Function to set the brightness of LEDs
    def set_led_brightness(led1_brightness, led2_brightness):
        #Make an HTTP GET request to set the brightness of both LEDs
        try:
            response = requests.get(f"{esp32_ip}/set", params={
                "value1": led1_brightness,
                "value2": led2_brightness
            })
            if response.status_code == 200:
                print(f"LED 1 Brightness: {led1_brightness}, LED 2 Brightness: {led2_brightness}")
            else:
                print("Failed to update brightness. Status code:", response.status_code)
        except Exception as e:
            print("Error:", e)

    #Main function
    def main():
        print("Setting LED brightness on the ESP32")

        #Set the brightness for both LEDs (values from 0 to 255)
        led1_brightness = 150  
        led2_brightness = 150  

        #Call the function to set the brightness
        set_led_brightness(led1_brightness, led2_brightness)

    if __name__ == "__main__":
        main()


#Code to turn left
def turn_left():
    import requests

    #ESP32 IP address
    esp32_ip = "http://192.168.1.202"

    #Function to set the brightness of LEDs
    def set_led_brightness(led1_brightness, led2_brightness):
        #Make an HTTP GET request to set the brightness of both LEDs
        try:
            response = requests.get(f"{esp32_ip}/set", params={
                "value1": led1_brightness,
                "value2": led2_brightness
            })
            if response.status_code == 200:
                print(f"LED 1 Brightness: {led1_brightness}, LED 2 Brightness: {led2_brightness}")
            else:
                print("Failed to update brightness. Status code:", response.status_code)
        except Exception as e:
            print("Error:", e)

    #Main function
    def main():
        print("Setting LED brightness on the ESP32")

        #Set the brightness for both LEDs (values from 0 to 255)
        led1_brightness = 0  
        led2_brightness = 150 

        #Call the function to set the brightness
        set_led_brightness(led1_brightness, led2_brightness)

    if __name__ == "__main__":
        main()


#Code to turn right
def turn_right():
    import requests

    #ESP32 IP address
    esp32_ip = "http://192.168.1.202"

    #Function to set the brightness of LEDs
    def set_led_brightness(led1_brightness, led2_brightness):
        #Make an HTTP GET request to set the brightness of both LEDs
        try:
            response = requests.get(f"{esp32_ip}/set", params={
                "value1": led1_brightness,
                "value2": led2_brightness
            })
            if response.status_code == 200:
                print(f"LED 1 Brightness: {led1_brightness}, LED 2 Brightness: {led2_brightness}")
            else:
                print("Failed to update brightness. Status code:", response.status_code)
        except Exception as e:
            print("Error:", e)

    #Main function
    def main():
        print("Setting LED brightness on the ESP32")

        #Set the brightness for both LEDs (values from 0 to 255)
        led1_brightness = 150  
        led2_brightness = 0  

        #Call the function to set the brightness
        set_led_brightness(led1_brightness, led2_brightness)

    if __name__ == "__main__":
        main()


#Code to stop the trolly
def stop_trolley():
    import requests

    #ESP32 IP address
    esp32_ip = "http://192.168.1.202"

    #Function to set the brightness of LEDs
    def set_led_brightness(led1_brightness, led2_brightness):
        #Make an HTTP GET request to set the brightness of both LEDs
        try:
            response = requests.get(f"{esp32_ip}/set", params={
                "value1": led1_brightness,
                "value2": led2_brightness
            })
            if response.status_code == 200:
                print(f"LED 1 Brightness: {led1_brightness}, LED 2 Brightness: {led2_brightness}")
            else:
                print("Failed to update brightness. Status code:", response.status_code)
        except Exception as e:
            print("Error:", e)

    #Main function
    def main():
        print("Setting LED brightness on the ESP32")

        #Set the brightness for both LEDs (values from 0 to 255)
        led1_brightness = 0 
        led2_brightness = 0  

        #Call the function to set the brightness
        set_led_brightness(led1_brightness, led2_brightness)

    if __name__ == "__main__":
        main()


#Detecting person and following by Loading the YOLOv8s model
url = 'http://192.168.1.241/cam-hi.jpg'
def capture_and_follow():
    #URL of the ESP32 camera stream
    url = 'http://192.168.1.241/cam-hi.jpg'

    #Load the YOLOv8s model
    model = YOLO('yolov8s.pt')

    cv2.namedWindow("ESP32 Camera Feed with YOLO Detection", cv2.WINDOW_AUTOSIZE)

    #Variables to track person movement
    person_center_previous = None

    while True:
        #Capture frame-by-frame from ESP32 camera
        try:
            #Fetch the image from ESP32 camera stream
            img_resp = urllib.request.urlopen(url)
            imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
            frame = cv2.imdecode(imgnp, -1)
        except Exception as e:
            print(f"Error accessing ESP32 camera: {e}")
            break

        #Check if the frame is valid
        if frame is None:
            print("Failed to capture image from ESP32 camera.")
            break

        #Perform object detection with YOLO
        results = model(frame)

        #Extract detections from results
        person_detected = False
        person_center_current = None

        for result in results:
            for det in result.boxes:
                x1, y1, x2, y2 = map(int, det.xyxy[0])  
                conf = det.conf[0].item() 
                cls = int(det.cls[0].item())  

                #Assuming 'person' class is 0
                if cls == 0 and conf > 0.5:  
                    #Draw bounding box for the detected person
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)

                    #Basic logic for following: center-based movement control
                    center_x = (x1 + x2) // 2
                    frame_center = frame.shape[1] // 2

                    #Track current person center
                    person_center_current = center_x

                    #Determine movement based on person's position
                    if center_x < frame_center - 50:  #Person is to the left
                        turn_left()
                    elif center_x > frame_center + 50:  #Person is to the right
                        turn_right()
                    else:  #Person is in the center
                        if person_center_previous is not None and abs(
                                person_center_current - person_center_previous) > 10:
                            #If the person is moving, move forward
                            move_forward()
                        else:
                            #If the person is not moving, stop
                            stop_trolley()

                    person_detected = True
                    break

        #If no person is detected, stop the trolley
        if not person_detected:
            stop_trolley()

        #Update previous center position
        person_center_previous = person_center_current

        #Display the frame with YOLO detections
        cv2.imshow('ESP32 Camera Feed with YOLO Detection', frame)

        #Exit loop if 'q' is pressed
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()


#Main-loop
def main():
    ia = False  #Flag to check if the trolley is active

    try:
        while True:
            command = listen_for_command()

            if "ready" in command and not ia:
                print("Activating System...")
                ia = True
                capture_and_follow()  #Start human detection and following

            elif "finish" in command and ia:
                print("Resetting System...")
                ia = False
                stop_trolley()

            elif "forward" in command:
                print("Moving Forward")
                move_forward()
                time.sleep(2)
                stop_trolley()

            elif "right" in command:
                print("Turning Right")
                turn_right()
                time.sleep(2)
                stop_trolley()

            elif "left" in command:
                print("Turning Left")
                turn_left()
                time.sleep(2)
                stop_trolley()

            elif "stop" in command:
                print("Stopping")
                stop_trolley()

    except KeyboardInterrupt:
        print("Program terminated.")
        stop_trolley()


if __name__ == "__main__":
    main()

#In this code you can notice that we have used the terms 'brightness, LED' in some lines of code.
#But in our final prototype we haven't used LEDs.
#Initially while we are doing the trial & error process of our code, we used two LEDs instead of two motors for easier checking.
#So we written our code using those terms.
#But this is the exact same code we used for our final prototype.
#And guess what? We successfully implemented all the feature which we thoght to showcase in our final prototype:)
