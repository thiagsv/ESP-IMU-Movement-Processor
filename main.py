import sys
import datetime
from scripts.opensim import getCalibrationDataPath, generateCalibratedModel, getTrackingDataPath, trackingMovement

def main():
    if len(sys.argv) < 3:
        print("Usage: python main.py <mock> [<startAt> <endAt>]")
        sys.exit(1)
    
    mock = sys.argv[1] == 'True'
    realStartAt = None
    realEndAt = None

    dataHora = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

    if not mock and len(sys.argv) >= 5:
        realStartAt = sys.argv[3]
        realEndAt = sys.argv[4]

    getCalibrationDataPath(mock, dataHora)
    generateCalibratedModel(dataHora)
    getTrackingDataPath(mock, dataHora)
    trackingMovement(realStartAt, realEndAt)

if __name__ == '__main__':
    main()
