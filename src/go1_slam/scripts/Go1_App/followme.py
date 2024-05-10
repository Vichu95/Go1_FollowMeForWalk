import cv2
import requests
import numpy as np

def capture_camera_and_send():
    cap = cv2.VideoCapture(0)

    triggered_value = ''
    while triggered_value != 'STOP':
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
            response = requests.post('http://10.201.0.237:5000/update_frame', data=frame_bytes)
            if response.status_code != 200:
                print("Failed to send frame to server")
        except requests.exceptions.RequestException as e:
            print("Error sending frame to server:", e)
            
    
        try:
            response = requests.get('http://10.201.0.237:5000/get_stopcmd_value')
            data = response.json()

            print(data)
            triggered_value = data.get('value')
            print("Triggered Value:", triggered_value)



            if(triggered_value == 'STOP'):
                print("STOPPINGGG!!")
                response = requests.post('http://10.201.0.237:5000/update_value', data={'stop_request_cmd': 'Init'})



        except requests.ConnectionError:
            print("Connection error: Failed to connect to the server:",requests.ConnectionError)	
    	

        
    cap.release()

if __name__ == '__main__':
    capture_camera_and_send()

