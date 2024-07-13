import sys
import datetime
from scripts.opensim import getCalibrationDataPath, generateCalibratedModel, getTrackingDataPath, trackingMovement

def main():
    if len(sys.argv) < 2:
        print("Usage: python main.py <mock> [<startAt> <endAt>]")
        sys.exit(1)
    
    mock = sys.argv[1] == 'True'
    realStartAt = None
    realEndAt = None

    dataHora = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

    if not mock and len(sys.argv) >= 4:
        realStartAt = sys.argv[2]
        realEndAt = sys.argv[3]

    getCalibrationDataPath(mock, dataHora)
    generateCalibratedModel(dataHora)
    getTrackingDataPath(mock, dataHora)
    trackingMovement(realStartAt, realEndAt)

if __name__ == '__main__':
    main()
