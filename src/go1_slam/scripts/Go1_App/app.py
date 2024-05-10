from flask import Flask, render_template, Response, jsonify,request
import numpy as np
import cv2
import os

app = Flask(__name__)

####
## WebApp Variables
####
frame_data = None
webapp_var_objdetected = 0
triggered_value = None


####
## HTML FILE
####
@app.route('/')
def index():
    return render_template('index.html',
                           webapp_var_objdetected=webapp_var_objdetected
                           
                           
                           )


##########################
##     V A R   R E A D 
##########################
@app.route('/update_value', methods=['POST'])
def update_value():
    global webapp_var_objdetected
    if request.form.get('webapp_var_objdetected'):
        webapp_var_objdetected = request.form.get('webapp_var_objdetected')
        return jsonify({'status': 'Value updated successfully', 'new_value': webapp_var_objdetected})
    else:
        return jsonify({'status': 'Failed to update value'}), 400



###############################
##    B U T T O N   R E A D 
###############################
@app.route('/trigger_value', methods=['POST'])
def trigger_value():
    global triggered_value
    data = request.json  # Extract JSON data from the request
    if data:
        value = data.get('value')
        print("Received value:", value)  # Log the received value
        triggered_value = value
        return 'Value triggered successfully' , 200
    else:
        return 'No data received', 400  # Return an error response if no data is received

@app.route('/get_triggered_value', methods=['GET'])
def get_triggered_value():
    global triggered_value
    return jsonify({'triggered_value': triggered_value})


##########################
##   V I D E O   R E A D 
##########################

@app.route('/update_frame', methods=['POST'])
def update_frame():
    global frame_data
    frame_data = request.data
    return 'Frame received successfully', 200

def generate_frames():
    global frame_data
    while True:
        if frame_data is None:
            continue

        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        ret, jpeg = cv2.imencode('.jpg', frame)
        frame_bytes = jpeg.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')





@app.route('/start_session')
def start_session():
    # Start a new tmux session with your command
    os.system("tmux new-session -d -s my_session_name 'stopgo1'")
    return 'Session started successfully'

@app.route('/stop_session')
def stop_session():
    # Kill the tmux session
    os.system("tmux kill-session -t my_session_name")
    return 'Session stopped successfully'




##################
##   M A I N
##################
if __name__ == '__main__':
   app.run(host='0.0.0.0', port=5000)

