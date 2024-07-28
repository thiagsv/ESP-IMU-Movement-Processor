import opensim as osim
from math import pi

orientationsFileName = ''

def getCalibrationDataPath(mock):
    """
    Retrieves the calibration data path.

    Args:
    mock (bool): Whether to use mock data.

    Returns:
    None
    """

    global orientationsFileName

    if mock:
        orientationsFileName = 'data/mock/orientations.sto'
        return
    
    orientationsFileName = 'data/quaternions.sto'

def generateCalibratedModel():
    """
    Generate a calibrated model

    Args:
    None

    Returns:
    None
    """

    visulizeCalibration = False
    baseIMUHeading = 'z'
    baseIMUName = 'pelvis_imu'
    modelPath = 'Rajagopal_2015.osim'

    sensorToOpensimRotations = osim.Vec3(-pi/2, 0, 0)
    imuPlacer = osim.IMUPlacer()
    imuPlacer.set_model_file(modelPath)
    imuPlacer.set_orientation_file_for_calibration(orientationsFileName)
    imuPlacer.set_sensor_to_opensim_rotations(sensorToOpensimRotations)
    imuPlacer.set_base_imu_label(baseIMUName)
    imuPlacer.set_base_heading_axis(baseIMUHeading)
    imuPlacer.run(visulizeCalibration)
    model = imuPlacer.getCalibratedModel()
    model.printToXML(f'calibrated_model')

def trackingMovement(mock, startAt = None, endAt = None):
    """
    Executes inverse kinematics tracking.

    Args:
    mock (bool): Whether to use mock data.
    dateTime (str): If not mock, specify the file with data.
    startAt (str|float): time that movement starts
    endAt (str|float): time that movement ends

    Returns:
    None
    """
    sensor_to_opensim_rotation = osim.Vec3(-pi/2, 0, 0)
    visualizeTracking = True
    resultsDirectory = 'IKResults'
    modelFileName = 'calibrated_model'

    if mock:
        startAt = 7.25
        endAt = 15
    else:
        startAt = float(startAt)
        endAt = float(endAt)

    imuIK = osim.IMUInverseKinematicsTool()
    imuIK.set_model_file(modelFileName)
    imuIK.set_orientations_file(orientationsFileName)
    imuIK.set_sensor_to_opensim_rotations(sensor_to_opensim_rotation)
    imuIK.set_results_directory(resultsDirectory)
    imuIK.set_time_range(0, startAt)
    imuIK.set_time_range(1, endAt)
    imuIK.run(visualizeTracking)
