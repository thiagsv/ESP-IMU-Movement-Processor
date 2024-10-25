import numpy as np
from numpy import dot
import math
from filterpy.kalman import KalmanFilter

float_formatter = "{:.6f}".format
np.set_printoptions(formatter={'float_kind': float_formatter})

def getQuaternionFromEuler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    """
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)

    return [qw, qx, qy, qz]

def applyKalmanFilter():
    # Constants
    n_sensor = 5
    filePath = 'data/imu/espDataFiltered.txt'
    outputFilePath = 'data/quaternions.sto'

    # Calibration vectors
    rateCalibrationRoll = [-0.09, -3.68, -4.72, -4.2, 7.55]
    rateCalibrationPitch = [0.63, -2.89, 1.93, 2.45, 1.07]
    rateCalibrationYaw = [0.44, -0.42, -0.91, -1.15, 1.87]

    acelCalibrationRoll = [1.030966, 1.027678, 1.041764, 1.074233, 1.042983]
    acelCalibrationPitch = [1.003606, 0.983924, 0.987624, 0.98639, 0.98764]
    acelCalibrationYaw = [0.96429, 1.072670, 0.972605, 0.749889, 1.014723]

    with open(filePath, 'r') as f, open(outputFilePath, 'w') as outputFile:
        lines = f.readlines()

        # Escreve o cabeçalho do arquivo de saída
        outputFile.write("Time\tqx\tqy\tqz\tqw\n")

        for i in range(0, len(lines) - n_sensor, n_sensor):
            linhaReferencia = lines[i:i+n_sensor]
            linhaCalcular = lines[i+n_sensor:i+(2*n_sensor)]

            for num in range(n_sensor):
                linhaCalcularNum = linhaCalcular[num].strip().split(',')
                linhaReferenciaNum = linhaReferencia[num].strip().split(',')

                # Parse the time values
                try:
                    tempo_referencia = float(linhaReferenciaNum[7])
                    tempo_calculo = float(linhaCalcularNum[7])
                except (IndexError, ValueError):
                    print(f"Erro ao acessar tempo para o sensor {num}.")
                    continue

                Ti = tempo_calculo - tempo_referencia
                if Ti <= 0:
                    print(f"Delta T inválido para o sensor {num}.")
                    continue

                # Calibrate accelerometer data
                linhaCalcularNum[1] = float(linhaCalcularNum[1]) - acelCalibrationRoll[num]
                linhaCalcularNum[2] = float(linhaCalcularNum[2]) - acelCalibrationPitch[num]
                linhaCalcularNum[3] = float(linhaCalcularNum[3]) - acelCalibrationYaw[num]

                # Calibrate gyroscope data
                rateRoll = float(linhaCalcularNum[4]) - rateCalibrationRoll[num]
                ratePitch = float(linhaCalcularNum[5]) - rateCalibrationPitch[num]
                rateYaw = float(linhaCalcularNum[6]) - rateCalibrationYaw[num]

                # Calculate Euler angles
                angleRoll = math.atan(float(linhaCalcularNum[1]) / math.sqrt(
                    float(linhaCalcularNum[0])**2 + float(linhaCalcularNum[2])**2)) * (180 / math.pi)
                anglePitch = -math.atan(float(linhaCalcularNum[0]) / math.sqrt(
                    float(linhaCalcularNum[1])**2 + float(linhaCalcularNum[2])**2)) * (180 / math.pi)
                angleYaw = math.atan(float(linhaCalcularNum[2]) / math.sqrt(
                    float(linhaCalcularNum[0])**2 + float(linhaCalcularNum[1])**2)) * (180 / math.pi)

                # Inicializando o filtro de Kalman
                kf = KalmanFilter(dim_x=6, dim_z=3)

                # Definindo as matrizes do filtro de Kalman
                kf.F = np.array([
                    [1, 0, 0, Ti, 0, 0],
                    [0, 1, 0, 0, Ti, 0],
                    [0, 0, 1, 0, 0, Ti],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1]
                ])

                kf.H = np.array([
                    [1, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 0, 1, 0, 0, 0]
                ])

                kf.R = np.eye(3) * 0.1  # Matriz de ruído de medição
                kf.Q = np.eye(6) * 0.01  # Matriz de ruído do processo
                kf.B = np.array([
                    [Ti, 0, 0],
                    [0, Ti, 0],
                    [0, 0, Ti],
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0]
                ])
                kf.u = np.array([rateRoll, ratePitch, rateYaw]).reshape((3, 1))

                # Estimação inicial
                kf.x = np.array([0, 0, 0, 0, 0, 0]).reshape((6, 1))
                kf.P = np.eye(6)  # Matriz de covariância inicial

                # Entrada de medição
                z = np.array([[angleRoll], [anglePitch], [angleYaw]])

                # Predição e atualização
                kf.predict()
                kf.update(z)

                # Convertendo os ângulos para quaterniões
                kalman_quaternion = getQuaternionFromEuler(
                    kf.x[0, 0] * (math.pi / 180),
                    kf.x[1, 0] * (math.pi / 180),
                    kf.x[2, 0] * (math.pi / 180)
                )

                # Escrevendo os dados no arquivo de saída
                outputFile.write(f"{tempo_calculo:.6f}\t{kalman_quaternion[0]:.6f}\t{kalman_quaternion[1]:.6f}\t"
                                 f"{kalman_quaternion[2]:.6f}\t{kalman_quaternion[3]:.6f}\n")
