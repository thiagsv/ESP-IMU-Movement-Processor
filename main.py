import sys
from scripts.opensim import getCalibrationDataPath, generateCalibratedModel, getTrackingDataPath, trackingMovement

def main():
    if len(sys.argv) < 3:
        print("Usage: python main.py <mock> <unixtimestamp> [<startAt> <endAt>]")
        sys.exit(1)
    
    mock = sys.argv[1] == 'True'
    unixtimestamp = sys.argv[2]
    realStartAt = None
    realEndAt = None

    if not mock and len(sys.argv) >= 5:
        realStartAt = sys.argv[3]
        realEndAt = sys.argv[4]

    getCalibrationDataPath(mock, unixtimestamp)
    generateCalibratedModel(unixtimestamp)
    getTrackingDataPath(mock, unixtimestamp)
    trackingMovement(realStartAt, realEndAt)

if __name__ == '__main__':
    main()
