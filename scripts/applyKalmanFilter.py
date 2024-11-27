import numpy as np
import math
from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt

# Configuração para a formatação de floats
float_formatter = "{:.6f}".format
np.set_printoptions(formatter={'float_kind': float_formatter})

def getQuaternionFromEuler(roll, pitch, yaw):
    """
    Converte ângulos de Euler para quaternions.
    """
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    return [qw, qx, qy, qz]

def applyKalmanFilter():
    # Constantes e configurações iniciais
    n_sensor = 5
    filePath = 'data/imu/espDataFiltered.txt'
    outputFilePath = 'data/quaternions.sto'

    #variáveis para gráficos
    anglesPitch = [[] for _ in range(5)]
    anglesRoll = [[] for _ in range(5)]
    anglesYaw = [[] for _ in range(5)]
    kfAnglesPitch = [[] for _ in range(5)]
    kfAnglesRoll = [[] for _ in range(5)]
    kfAnglesYaw = [[] for _ in range(5)]
    graphTime = [[] for _ in range(5)]
    tempo_acumulado = [0] * n_sensor

    # Vetores de calibração
    rateCalibrationRoll = [-2.11, -5.07, 0.29, -1.15, -0.87]
    rateCalibrationPitch = [-0.56, 0.62, -4.36, 1.05, -0.57]  
    rateCalibrationYaw = [-1.97, 1.19, -2.47, -1.09, -0.59]

    acellCalibration = [
        np.array([0.009146, -0.003567, 0.023015]),
        np.array([0.081870, -0.004974, 0.078128]),
        np.array([0.065413, -0.004441, -0.098863]),
        np.array([0.055446, -0.003839, -1.109496]),
        np.array([0.018971, 0.003310, 0.116460])
    ]

    acellInverseMatrix = [
        np.array([
            [1.003814, 0.000162, -0.002811],
            [0.000162, 0.997502, 0.003760],
            [-0.002811, 0.003760, 0.997041]
        ]),
        np.array([
            [1.004997, 0.000331, 0.000655],
            [0.000331, 1.006226, -0.000795],
            [0.000655, -0.000795, 0.979038]
        ]),
        np.array([
            [0.994337, 0.000083, 0.000567],
            [0.000083, 0.995980, -0.001658],
            [0.000567, -0.001658, 0.967582]
        ]),
        np.array([
            [1.000174, -0.000115, 0.000493],
            [-0.000115, 1.000997, -0.003509],
            [0.000493, -0.003509, 0.980719]
        ]),
        np.array([
            [0.998681, -0.000003, -0.001392],
            [-0.000003, 1.004926, -0.002661],
            [-0.001392, -0.002661, 0.990277]
        ])
    ]

    offsets = [
       {'roll': 0, 'pitch': 90, 'yaw': 90}, 
       {'roll': 190, 'pitch': -100, 'yaw': 90},
       {'roll': 0, 'pitch': -100, 'yaw': -100},
       {'roll': -5, 'pitch': -130, 'yaw': -130},
       {'roll': 180, 'pitch': -110, 'yaw': 70}
    ]

    n_real = [
        {'roll': 2.5, 'pitch': 2.5, 'yaw': 2.5},
        {'roll': 2.5, 'pitch': 2.5, 'yaw': 2.5},
        {'roll': 2.5, 'pitch': 2.5, 'yaw': 2.5},
        {'roll': 2.5, 'pitch': 2.5, 'yaw': 2.5},
        {'roll': 2.5, 'pitch': 2.5, 'yaw': 2.5}
    ]

    m_real = [
        {'roll': 0.05, 'pitch': 0.05, 'yaw': 0.05},
        {'roll': 0.05, 'pitch': 0.05, 'yaw': 0.05},
        {'roll': 0.05, 'pitch': 0.05, 'yaw': 0.05},
        {'roll': 0.05, 'pitch': 0.05, 'yaw': 0.05},
        {'roll': 0.05, 'pitch': 0.05, 'yaw': 0.05}
    ]

    kalman_filters = []
    for num in range(n_sensor):
        kf = KalmanFilter(dim_x=6, dim_z=3)
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])

        kf.R = np.array([
            [n_real[num]['roll'], 0.0, 0.0],
            [0.0, n_real[num]['pitch'], 0.0],
            [0.0, 0.0, n_real[num]['yaw']]
        ])

        kf.x = np.zeros((6, 1))
        kf.P = np.zeros((6, 6))

        kalman_filters.append(kf)

    quart = []
    with open(filePath, 'r') as f, open(outputFilePath, 'w') as outputFile:
        outputFile.write('DataRate=100.000000\nDataType=Quaternion\nversion=3\nOpenSimVersion=4.1\nendheader\ntime\tpelvis_imu\tfemur_r_imu\tfemur_l_imu\ttibia_l_imu\ttibia_r_imu\n')
        dadosIMUs = ""
        linhaCalcular = None
        temposg = 0
    
        while True:
            dadosBloco = f.read(20)  # Ler um bloco maior para capturar múltiplas linhas
            if not dadosBloco:
                break

            dadosIMUs += dadosBloco
            linhas = dadosIMUs.split(';')

            # Define quantidade de linhas esperadas e verifica se há dados suficientes
            qtdLinhas = (n_sensor * 2) + 1 if linhaCalcular is None else n_sensor + 1
            if len(linhas) < qtdLinhas:
                continue

            # Atualiza linha de referência e de cálculo
            if linhaCalcular is None:
                linhaReferencia = linhas[:n_sensor]
                linhaCalcular = linhas[n_sensor:2 * n_sensor]
                linhasRestantes = linhas[2 * n_sensor:]
            else:
                linhaReferencia = linhaCalcular
                linhaCalcular = linhas[:n_sensor]
                linhasRestantes = linhas[n_sensor:]

            dadosIMUs = ';'.join(linhasRestantes)

            for num in range(n_sensor):
                linhaCalcularNum = linhaCalcular[num].split(',')
                linhaReferenciaNum = linhaReferencia[num].split(',')
                # Extrai tempos e calcula delta T
                try:
                    tempo_referencia = float(linhaReferenciaNum[7]) if len(linhaReferenciaNum) > 7 else 0
                    tempo_calculo = float(linhaCalcularNum[7]) if len(linhaCalcularNum) > 7 else 0
                except ValueError:
                    print(f"Erro ao converter tempo para o sensor {num}.")
                    continue
                Ti = tempo_calculo - tempo_referencia
                if Ti <= 0:
                    print(f"Delta T inválido para o sensor {num}.")
                    continue

                tempo_acumulado[num] += Ti
                graphTime[num].append(tempo_acumulado[num])

                accelData = acellInverseMatrix[num] @ (np.array([float(linhaCalcularNum[1]), float(linhaCalcularNum[2]), float(linhaCalcularNum[3])]) - acellCalibration[num])

                # Calibração do giroscópio
                rateRoll = float(linhaCalcularNum[4]) - rateCalibrationRoll[num]
                ratePitch = float(linhaCalcularNum[5]) - rateCalibrationPitch[num]
                rateYaw = float(linhaCalcularNum[6]) - rateCalibrationYaw[num]

                # Cálculo dos ângulos de Euler com offsets
                angleRoll = offsets[num]['roll'] + math.atan(float(accelData[1]) / math.sqrt(
                    float(accelData[0])**2 + float(accelData[2])**2)) * (180 / math.pi)
                anglePitch = offsets[num]['pitch'] + -math.atan(float(accelData[0]) / math.sqrt(
                    float(accelData[1])**2 + float(accelData[2])**2)) * (180 / math.pi)
                angleYaw = offsets[num]['yaw'] + math.atan(float(accelData[2]) / math.sqrt(
                    float(accelData[0])**2 + float(accelData[1])**2)) * (180 / math.pi)
        
                anglesRoll[num].append(angleRoll)
                anglesPitch[num].append(anglePitch)
                anglesYaw[num].append(angleYaw)

                # Inicializa o filtro de Kalman
                kalman_filters[num].F = np.array([
                    [1.0, 0.0, 0.0, -Ti, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0, -Ti, 0.0],
                    [0.0, 0.0, 1.0, 0.0, 0.0, -Ti],
                    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                ])

                # Ruído de processo a partir de m_real
                kalman_filters[num].Q = np.array([
                    [m_real[num]['roll'] * Ti * Ti, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, m_real[num]['pitch'] * Ti * Ti, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, m_real[num]['yaw'] * Ti * Ti, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, m_real[num]['roll'], 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, m_real[num]['pitch'], 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, m_real[num]['yaw']]
                ])
    
                kalman_filters[num].B = np.array([
                    [Ti, 0, 0],
                    [0, Ti, 0],
                    [0, 0, Ti],
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0]
                ])

                u = np.array([rateRoll, ratePitch, rateYaw]).reshape((3, 1))
                z = np.array([angleRoll, anglePitch, angleYaw]).reshape((3, 1))

                # Passo de predição e atualização do filtro
                kalman_filters[num].predict(u)
                kalman_filters[num].update(z)

                # Converte os ângulos resultantes em quaternions
                kalman_quaternion = getQuaternionFromEuler(
                    kalman_filters[num].x[0, 0] * (math.pi / 180),
                    kalman_filters[num].x[1, 0] * (math.pi / 180),
                    kalman_filters[num].x[2, 0] * (math.pi / 180)
                )
                quart.append(kalman_quaternion)

                kfAnglesRoll[num].append(kalman_filters[num].x[0, 0])
                kfAnglesPitch[num].append(kalman_filters[num].x[1, 0])
                kfAnglesYaw[num].append(kalman_filters[num].x[2, 0])

            # Escreve os dados no arquivo após o processamento de todos os sensores
            if len(quart) == n_sensor:
                if tempo_calculo >= 2:
                    outputFile.write(f"{tempo_calculo - 2}\t" + '\t'.join(
                        ','.join(map(str, quat)) for quat in quart) + '\n')
                quart = []  # Limpa para o próximo conjunto de sensores

    for i in range(n_sensor):
        plt.figure(figsize=(20, 6))
        plt.plot(graphTime[i], anglesRoll[i], color='r', label='AcelX')  # Vermelho
        plt.plot(graphTime[i], anglesPitch[i], color='b', label='AcelY')  # Azul escuro
        plt.plot(graphTime[i], anglesYaw[i], color='g', label='AcelZ')  # Verde
        plt.plot(graphTime[i], kfAnglesRoll[i], color='y', label='KalmanX')  # Amarelo
        plt.plot(graphTime[i], kfAnglesPitch[i], color='c', label='KalmanY')  # Azul claro
        plt.plot(graphTime[i], kfAnglesYaw[i], color='m', label='KalmanZ')  # Rosa

        # Configurações do gráfico
        plt.xlabel("Tempo [s]")
        plt.ylabel("Graus [º]")
        plt.title(f"IMU {i+1}")
        plt.legend()
        plt.xticks(np.arange(0, 70, step=1))
        plt.yticks(np.arange(-180, 180, step=10))
        plt.grid(color='g', linestyle='--', linewidth=0.5)
        plt.show()