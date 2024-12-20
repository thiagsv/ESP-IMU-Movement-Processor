import os
import datetime
import shutil
from scripts.applyKalmanFilter import applyKalmanFilter

processedFile = 'data/quaternions.sto'

def processIMUData():
    """
    Processes data from the ESP32 device.

    This function processes the raw data from the ESP32 device by applying a Kalman filter 
    and then converting the processed data into a .sto file structure. The resulting data 
    is saved in two locations: a timestamped file in the 'opensimData' directory in the user's 
    home directory and a fixed location 'data/quaternions.sto'.


    Returns:
    None
    """

    createSTOStructure(processedFile)

    try:
        applyFilter()
        saveFile2User()
    except Exception as e:
        print(f'Error processing file: {e}')

def saveFile2User():
    """
    Saves data to a file in a specified location.

    Args:
    None
    
    Returns:
    None
    """

    path = os.path.expanduser('~') + '/opensimData/'
    if not os.path.exists(path):
        os.makedirs(path)
    
    dateTime = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
    path += '/data_' + dateTime + '.sto'

    try:
        shutil.copy(processedFile, path)
    except Exception as e:
        print(f'Error at saving data: {e}')

def createSTOStructure(processedFile):
    """
    Create .sto structure required by OpenSim

    Args:
    None

    Returns:
    None
    """
    with open(processedFile, 'w') as out_f:
        out_f.write('DataRate=100.000000\n')
        out_f.write('DataType=Quaternion\n')
        out_f.write('version=3\n')
        out_f.write('OpenSimVersion=4.1\n')
        out_f.write('endheader\n')
        out_f.write('time\tpelvis_imu\tfemur_r_imu\tfemur_l_imu\ttibia_r_imu\ttibia_l_imu\n')

def applyFilter():
    applyKalmanFilter()
