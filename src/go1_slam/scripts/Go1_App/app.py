from flask import Flask, render_template, Response, jsonify,request
import numpy as np
import cv2
import os
import stop


app = Flask(__name__)

####
## WebApp Variables
####
frame_data = None
webapp_var_STOP_followme = None


####
## HTML FILE
####
@app.route('/')
def index():
    return render_template('index.html')


##########################
##     V A R   R E A D 
##########################
@app.route('/update_stopcmd_value', methods=['POST'])
def update_stopcmd_value():
    global webapp_var_STOP_followme
    new_value = request.form.get('stop_request_cmd')
    if new_value:
        webapp_var_STOP_followme = new_value
        return jsonify({'status': 'Value updated successfully', 'stop_request_cmd': new_value})
    else:
        return jsonify({'status': 'Failed to update value'}), 400

@app.route('/get_stopcmd_value', methods=['GET'])
def get_stopcmd_value():
    global webapp_var_STOP_followme
    return jsonify({'stop_request_cmd': webapp_var_STOP_followme})



####################################
##    T R I G G E R     S T O P
####################################

@app.route('/execute_script', methods=['POST'])
def execute_script():
    # Code to run a script on your laptop
    # os.system("python3 stop.py")
    result = stop.stop_func()


    # return 'Script executed successfully'
    return jsonify({'status': result, 'output': 'success'})
 


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





##################
##   M A I N
##################
if __name__ == '__main__':
   app.run(host='0.0.0.0', port=5000)

