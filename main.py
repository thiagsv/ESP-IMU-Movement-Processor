import sys
import datetime
from scripts.opensim import getCalibrationDataPath, generateCalibratedModel, getTrackingDataPath, trackingMovement

def main():
    realStartAt = None
    realEndAt = None
    mock = False

    if len(sys.argv) >= 2:
        if len(sys.argv) > 2:
            mock = sys.argv[3] == 'True' or sys.argv[3] == '1' or sys.argv[3] == 'true'

        if not mock:
            realStartAt = sys.argv[1]
            realEndAt = sys.argv[2]
    
    dateTime = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

    getCalibrationDataPath(mock, dateTime)
    generateCalibratedModel(dateTime)
    getTrackingDataPath(mock, dateTime)
    trackingMovement(mock, dateTime, realStartAt, realEndAt)

if __name__ == '__main__':
    main()
