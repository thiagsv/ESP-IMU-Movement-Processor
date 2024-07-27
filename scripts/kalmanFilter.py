import os
import datetime

def processEspData(espData, dateTime):
    processedData = applyKalmanFilter(espData)
    stoStructure = createSTOStructure(processedData)

    saveFile(stoStructure)
    saveFile(stoStructure, True)

def saveFile(data, local = False):
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