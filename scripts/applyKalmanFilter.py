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
    rateCalibrationRoll = [-1.19, -5.05, -0.02, -0.02, -2.87]
    rateCalibrationPitch = [1.46, 0.37, -0.02, -0.02, -0.45]  
    rateCalibrationYaw = [7.01, 1.2, -0.02, -0.02, -0.19]

    acelCalibrationRoll = [0.017993, 0.082357, 0.046716, 0.055446, 0.022596]
    acelCalibrationPitch = [-0.008373, -0.006382, -0.019804, -0.003839, -0.012704]
    acelCalibrationYaw = [0.030728, 0.078694, -0.007731, -1.109496, -0.018545]

    offsets = [
        {'roll': 90, 'pitch': 90, 'yaw':0}, 
        {'roll': 0, 'pitch': -135, 'yaw': -90},
        {'roll': -29, 'pitch': -120, 'yaw': -95},
        {'roll': -20, 'pitch': -110, 'yaw': -90},
        {'roll': 0, 'pitch': -90, 'yaw': -90}
    ]

    n_real = [
        {'roll': 0.5, 'pitch': 0.5, 'yaw': 0.5},
        {'roll': 0.5, 'pitch': 0.5, 'yaw': 0.5},
        {'roll': 0.5, 'pitch': 0.5, 'yaw': 0.5},
        {'roll': 0.5, 'pitch': 0.5, 'yaw': 0.5},
        {'roll': 0.5, 'pitch': 0.5, 'yaw': 0.5}
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
        kf.H = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0]])
        kf.x = np.zeros((6, 1))  # Estado inicial
        kf.P = np.eye(6)
        # Ruído de medição a partir de n_real
        kf.R = np.array([
            [n_real[num]['roll'], 0.0, 0.0],
            [0.0, n_real[num]['pitch'], 0.0],
            [0.0, 0.0, n_real[num]['yaw']]
        ])
        kf.Q = np.eye(6)  # Ajuste conforme o ruído do processo
        kf.F = np.eye(6)  # Ajuste F conforme necessário
        kalman_filters.append(kf)

    quart = []
    initial_kalman_states = []
    epsilon = 1e-6
    with open(filePath, 'r') as f, open(outputFilePath, 'w') as outputFile:
        outputFile.write('DataRate=100.000000\nDataType=Quaternion\nversion=3\nOpenSimVersion=4.1\nendheader\ntime\tpelvis_imu\tfemur_l_imu\tfemur_r_imu\ttibia_r_imu\ttibia_l_imu\n')
        dadosIMUs = ""
        linhaCalcular = None
        temposg = 0
        primeira = True

        initial_kalman_states = []
    
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
                primeira = False
                linhaReferencia = linhaCalcular
                linhaCalcular = linhas[:n_sensor]
                linhasRestantes = linhas[n_sensor:]

            dadosIMUs = ';'.join(linhasRestantes)

            for num in range(n_sensor):
                linhaCalcularNum = linhaCalcular[num].split(',')
                linhaReferenciaNum = linhaReferencia[num].split(',')

                if primeira:
                    linhaReferenciaNum = linhaReferencia[num].split(',')
                    # Calcular os ângulos de Euler iniciais
                    roll_init = offsets[num]['roll'] + math.atan(
                        float(linhaReferenciaNum[1]) / math.sqrt(float(linhaReferenciaNum[0])**2 + float(linhaReferenciaNum[2])**2 + epsilon)) * (180 / math.pi)
                    pitch_init = offsets[num]['pitch'] + -math.atan(
                        float(linhaReferenciaNum[0]) / math.sqrt(float(linhaReferenciaNum[1])**2 + float(linhaReferenciaNum[2])**2 + epsilon)) * (180 / math.pi)
                    yaw_init = offsets[num]['yaw'] + math.atan(
                        float(linhaReferenciaNum[2]) / math.sqrt(float(linhaReferenciaNum[0])**2 + float(linhaReferenciaNum[1])**2 + epsilon)) * (180 / math.pi)

                    # Inicializa o estado do filtro de Kalman para cada sensor
                    initial_kalman_states.append([roll_init, pitch_init, yaw_init, 0, 0, 0])

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

                # Calibração do acelerômetro
                linhaCalcularNum[1] = float(linhaCalcularNum[1]) - acelCalibrationRoll[num]
                linhaCalcularNum[2] = float(linhaCalcularNum[2]) - acelCalibrationPitch[num]
                linhaCalcularNum[3] = float(linhaCalcularNum[3]) - acelCalibrationYaw[num]

                # Calibração do giroscópio
                rateRoll = float(linhaCalcularNum[4]) - rateCalibrationRoll[num]
                ratePitch = float(linhaCalcularNum[5]) - rateCalibrationPitch[num]
                rateYaw = float(linhaCalcularNum[6]) - rateCalibrationYaw[num]

                # Cálculo dos ângulos de Euler com offsets
                angleRoll = offsets[num]['roll'] + math.atan(float(linhaCalcularNum[1]) / math.sqrt(
                    float(linhaCalcularNum[0])**2 + float(linhaCalcularNum[2])**2)) * (180 / math.pi)
                anglePitch = offsets[num]['pitch'] + -math.atan(float(linhaCalcularNum[0]) / math.sqrt(
                    float(linhaCalcularNum[1])**2 + float(linhaCalcularNum[2])**2)) * (180 / math.pi)
                angleYaw = offsets[num]['yaw'] + math.atan(float(linhaCalcularNum[2]) / math.sqrt(
                    float(linhaCalcularNum[0])**2 + float(linhaCalcularNum[1])**2)) * (180 / math.pi)

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
                kalman_filters[num].Q = np.array([ #FICA
                    [m_real[num]['roll'] * Ti * Ti, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, m_real[num]['pitch'] * Ti * Ti, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, m_real[num]['yaw'] * Ti * Ti, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, m_real[num]['roll'], 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, m_real[num]['pitch'], 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, m_real[num]['yaw']]
                ])
    
                kalman_filters[num].B = np.array([ #FICA
                    [Ti, 0, 0],
                    [0, Ti, 0],
                    [0, 0, Ti],
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0]
                ])

                kalman_filters[num].u = np.array([rateRoll, ratePitch, rateYaw]).reshape((3, 1)) #FICA

                # Medida z com ângulos de Euler
                z = np.array([angleRoll, anglePitch, angleYaw]).reshape((3, 1)) #FICA

                # Passo de predição e atualização do filtro
                kalman_filters[num].predict()
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
            if len(quart) == n_sensor and tempo_calculo >= 2:
                outputFile.write(f"{tempo_calculo}\t" + '\t'.join(
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
        plt.yticks(np.arange(-270, 100, step=10))
        plt.grid(color='g', linestyle='--', linewidth=0.5)
        plt.show()