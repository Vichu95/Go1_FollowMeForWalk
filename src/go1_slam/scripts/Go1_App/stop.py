
import requests





if __name__ == '__main__':

    print("STOP .py started executing....")


    try:
        response = requests.post('http://10.201.0.237:5000/update_value', data={'new_value': 'STOP'})
        if response.status_code == 200:
            print("Value updated successfully")
        else:
            print("Failed to update value python")
    except requests.exceptions.RequestException as e:
        print("Error sending request:", e)



    
    print("STOP .py stopped executing....")