import cv2
import requests
import numpy as np

def capture_camera_and_send():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break
		
        cv2.imshow('Camera Feed', frame)  # Display the frame
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            break
            
        # Encode the frame as JPEG
        _, jpeg = cv2.imencode('.jpg', frame)
        frame_bytes = jpeg.tobytes()

        # Send the frame to the Flask server
        try:
            response = requests.post('http://127.0.0.1:5000/update_frame', data=frame_bytes)
            if response.status_code != 200:
                print("Failed to send frame to server")
        except requests.exceptions.RequestException as e:
            print("Error sending frame to server:", e)
            
    
        try:
            response = requests.get('http://127.0.0.1:5000/get_triggered_value')
            data = response.json()

            print(data)
            triggered_value = data.get('triggered_value')
            print("Triggered Value:", triggered_value)



            if(triggered_value == 'STOP'):
        
                try:
                    response = requests.post('http://127.0.0.1:5000/update_value', data={'webapp_var_objdetected': 5})
                    if response.status_code == 200:
                        print("Value updated successfully")
                    else:
                        print("Failed to update value python")
                except requests.exceptions.RequestException as e:
                    print("Error sending request:", e)



        except requests.ConnectionError:
            print("Connection error: Failed to connect to the server:",requests.ConnectionError)	
    	

        
    cap.release()

if __name__ == '__main__':
    capture_camera_and_send()

