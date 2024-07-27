import os
import datetime

def processEspData(espData):
    """
    Processes data from the ESP32 device.

    This function processes the raw data from the ESP32 device by applying a Kalman filter 
    and then converting the processed data into a .sto file structure. The resulting data 
    is saved in two locations: a timestamped file in the 'opensimData' directory in the user's 
    home directory and a fixed location 'data/imuData.sto'.

    Args:
    espData (str): The raw data from the ESP32 device.

    Returns:
    None
    """
    processedData = applyKalmanFilter(espData)
    stoStructure = createSTOStructure(processedData)

    saveFile(stoStructure)
    saveFile(stoStructure, True)


def saveFile(data, local = False):
    """
    Saves data to a file in a specified location.

    This function saves the provided data to a file. If the `local` parameter is False, 
    the data is saved in a directory named 'opensimData' in the user's home directory 
    with a timestamped filename. If `local` is True, the data is saved in a fixed location 
    'data/imuData.sto'.

    Args:
    data (str): The data to be saved.
    local (bool): If True, saves the data to 'data/imuData.sto'. If False, saves the data 
                    to a timestamped file in the 'opensimData' directory in the user's home directory.
    
    Returns:
    None
    """
    if not local:
        path = os.path.expanduser('~') + '/opensimData/'
        if not os.path.exists(path):
            os.makedirs(path)
        
        dateTime = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
        path += '/data_' + dateTime + '.sto'
    else:
        path = 'data/imuData.sto'

    try:
        with open(path, 'w') as arquivo:
            arquivo.write(data)
        print(f'Saved data at {path}')
    except Exception as e:
        print(f'Error at saving data: {e}')

def createSTOStructure(espData):
    return espData

def applyKalmanFilter(espData):
    return espData