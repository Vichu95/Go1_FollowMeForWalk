import requests
import subprocess
import time
import os

def stop_func():

    print("Stopping...")

    log = ''

    try:
        response = requests.post('http://192.168.12.65:5000/update_stopcmd_value', data={'stop_request_cmd': 'STOP'})
        if response.status_code == 200:
            log += "\nStop trigger to followme : success"
        else:
            log += "\nStop trigger to followme : failed"
    except requests.exceptions.RequestException as e:
        log += "\nError sending Stop trigger to followme:", e


    time.sleep(0.3)

    cmd = "rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
    
    
    # os.system("tmux new-session -d -s my_session_name '" + cmd + "'")
    # os.system("tmux new-session -d -s my_session_name 'rostopic pub /cmd_vel geometry_msgs/Twist \'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\''")
    #subprocess.run(cmd, shell=True)

    log +=  "\nPublishing zero cmd_vel for safety..."
    
    log += "\nStopped!"

    print (log)

    return log

