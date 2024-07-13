import opensim as osim
from math import pi

modelPath = 'Rajagopal_2015.osim'
orientationsFileName = 'data/'
baseIMUName = 'pelvis_imu'
baseIMUHeading = 'z'
visulizeCalibration = False
trackingFileName = 'data/mock/orientations.sto'
modelFileName = 'calibrated_model_'
resultsDirectory = 'IKResults'

def getCalibrationDataPath(mock, unixtimestamp):
    """
    Retrieves the calibration data path.

    Args:
    mock (bool): Whether to use mock data.
    unixtimestamp (str): If not mock, specify the file with data.

    Returns:
    None
    """

    global orientationsFileName

    if mock:
        orientationsFileName = orientationsFileName + 'mock/orientations.sto'
        return
    
    orientationsFileName += unixtimestamp

def generateCalibratedModel(unixtimestamp):
    """
    Generate a calibrated model

    Args:
    unixtimestamp (int)

    Returns:
    None
    """
    global modelFileName

    sensorToOpensimRotations = osim.Vec3(-pi/2, 0, 0)
    imuPlacer = osim.IMUPlacer()
    imuPlacer.set_model_file(modelPath)
    imuPlacer.set_orientation_file_for_calibration(orientationsFileName)
    imuPlacer.set_sensor_to_opensim_rotations(sensorToOpensimRotations)
    imuPlacer.set_base_imu_label(baseIMUName)
    imuPlacer.set_base_heading_axis(baseIMUHeading)
    imuPlacer.run(visulizeCalibration)
    model = imuPlacer.getCalibratedModel()
    model.printToXML(f'calibrated_model_{unixtimestamp}')

def getTrackingDataPath(mock, unixtimestamp):
    """
    Retrieves the tracking data path.

    Args:
    mock (bool): Whether to use mock data.
    unixtimestamp (str): timestamp of reading from ESP

    Returns:
    None
    """
    global modelFileName, orientationsFileName

    modelFileName += unixtimestamp

    if not mock:
        orientationsFileName = 'data/' + unixtimestamp
        startAt = float(startAt)  # Convert to float if necessary
        endAt = float(endAt)      # Convert to float if necessary

def trackingMovement(startAt = None, endAt = None):
    """
    Executes inverse kinematics tracking.

    Args:

    startAt (str|float): time that movement starts
    endAt (str|float): time that movement ends

    Returns:
    None
    """
    sensor_to_opensim_rotation = osim.Vec3(-pi/2, 0, 0)
    visualizeTracking = True

    if startAt is None or endAt is None:
        startAt = 7.25
        endAt = 15


    imuIK = osim.IMUInverseKinematicsTool()
    imuIK.set_model_file(modelFileName)
    imuIK.set_orientations_file(trackingFileName)
    imuIK.set_sensor_to_opensim_rotations(sensor_to_opensim_rotation)
    imuIK.set_results_directory(resultsDirectory)
    imuIK.set_time_range(0, startAt)
    imuIK.set_time_range(1, endAt)
    imuIK.run(visualizeTracking)
