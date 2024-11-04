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

def getCofactor(mat, temp, p, q, n):
    i = 0
    j = 0

    # Looping for each element
    # of the matrix
    for row in range(n):

        for col in range(n):

            # Copying into temporary matrix
            # only those element which are
            # not in given row and column
            if (row != p and col != q):

                temp[i][j] = mat[row][col]
                j += 1

                # Row is filled, so increase
                # row index and reset col index
                if (j == n - 1):
                    j = 0
                    i += 1

def determinantOfMatrix(mat):
    return np.linalg.det(mat)

def applyKalmanFilter():
    # Constants
    n_sensor = 5
    filePath = 'data/imu/espDataFiltered.txt'
    outputFilePath = 'data/quaternions.sto'

    # Calibration vectors
    rateCalibrationRoll = [-0.2, -5.05, -0.02, -1.19, -2.87]
    rateCalibrationPitch = [-0.2, 0.37, -0.02, 1.46, -0.45]
    rateCalibrationYaw = [-0.2, 1.2, -0.02, 7.01, -0.19]

    acelCalibrationRoll = [1.030966, 1.027678, 1.041764, 1.074233, 1.042983]
    acelCalibrationPitch = [1.003606, 0.983924, 0.987624, 0.98639, 0.98764]
    acelCalibrationYaw = [0.96429, 1.072670, 0.972605, 0.749889, 1.014723]

    a_shape = (6, 6)
    P = np.zeros(a_shape)

    nstate_nobs = (6, 3)
    nobs_nobs = (3, 3)
    n_z = (3,1)

    S = np.zeros(nobs_nobs)
    k = np.zeros(nstate_nobs)
    z = np.zeros(n_z)

    offsets = [
        {'roll': -10, 'pitch': -105, 'yaw': -80},
        {'roll': -10, 'pitch': -100, 'yaw': -100},
        {'roll': 0, 'pitch': -110, 'yaw': -70},
        {'roll': -20, 'pitch': -110, 'yaw': -90},
        {'roll': -90, 'pitch': 90, 'yaw': 0}
    ]

    n_real = [
        {'roll': 3, 'pitch': 3, 'yaw': 3},
        {'roll': 3, 'pitch': 3, 'yaw': 3},
        {'roll': 3, 'pitch': 3, 'yaw': 3},
        {'roll': 3, 'pitch': 3, 'yaw': 3},
        {'roll': 3, 'pitch': 3, 'yaw': 3}
    ]

    m_real = [
        {'roll': 5, 'pitch': 5, 'yaw': 0.1},
        {'roll': 5, 'pitch': 5, 'yaw': 0.1},
        {'roll': 5, 'pitch': 5, 'yaw': 3},
        {'roll': 5, 'pitch': 5, 'yaw': 0.1},
        {'roll': 5, 'pitch': 5, 'yaw': 0.1}
    ]

    # Inicializa a string que vai armazenar os dados lidos
    dadosIMUs = ""
    linhaUm = True
    tempoInicial = 0
    linhaCalcular = None
    with open(filePath, 'r') as f, open('data/quaternions.sto', 'a') as outputFile:
        while True:
            dadosBloco = f.read(20)  # Ler um bloco maior para capturar múltiplas linhas

            if not dadosBloco:
                break

            dadosIMUs += dadosBloco
            linhas = dadosIMUs.split(';')

            qtdLinhas = n_sensor + 1
            if linhaCalcular is None:
                qtdLinhas = (n_sensor * 2) + 1

            if len(linhas) < qtdLinhas:
                continue  # Continuar se não tiver o suficiente para referência e cálculo

            # A linha de referência será a anterior, e a linha de cálculo será a atual
            if linhaCalcular is None:
                linhaReferencia = linhas[:n_sensor]
                linhaCalcular = linhas[n_sensor:2 * n_sensor]
                linhasRestantes = linhas[2 * n_sensor:]  # Pega o restante após as linhas processadas
            else:
                linhaReferencia = linhaCalcular
                linhaCalcular = linhas[:n_sensor]
                linhasRestantes = linhas[n_sensor:]  # Pega o restante após as linhas processadas

            # Atualiza o buffer para remover apenas as linhas que já foram processadas
            dadosIMUs = ';'.join(linhasRestantes)
            for num in range(0, n_sensor):  # É calculado 1 IMU por vez, -1 pois começa no 0
                x = np.array([
                    [0.0],
                    [0.0],
                    [0.0],
                    [0.0],
                    [0.0],
                    [0.0]
                ])

                u = np.array([
                    [0.0],
                    [0.0],
                    [0.0]
                ])

                P = np.zeros(a_shape)
                S= np.zeros(nobs_nobs)
                k = np.zeros(nstate_nobs)
                z = np.zeros(n_z)

                n_roll_real = n_real[num]['roll']
                n_pitch_real = n_real[num]['pitch']
                n_yaw_real = n_real[num]['yaw']

                m_roll_real = m_real[num]['roll']
                m_pitch_real = m_real[num]['pitch']
                m_yaw_real = n_real[num]['yaw']

                #matriz utilizada no filtro de Kalman
                r = np.array([
                    [n_roll_real, 0.0, 0.0],
                    [0.0, n_pitch_real, 0.0],
                    [0.0, 0.0, n_yaw_real]
                ])

                linhaCalcularNum = linhaCalcular[num].split(',')
                linhaReferenciaNum = linhaReferencia[num].split(',')

                # Ti é o delta T entre a coleta atual e a coleta anterior
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

                #Offsets + cálculo dos ângulos de Euler
                offsetRoll = offsets[num]['roll']
                offsetPitch = offsets[num]['pitch']
                offsetYaw = offsets[num]['yaw']

                angleRoll = offsetRoll + (math.atan((float(linhaCalcularNum[1])) / math.sqrt(
                    (float(linhaCalcularNum[0])) * (float(linhaCalcularNum[0])) +
                    ((float(linhaCalcularNum[2]) ) * (float(linhaCalcularNum[2])))
                ))*(1/(math.pi/180)))

                anglePitch = offsetPitch + (-math.atan((float(linhaCalcularNum[0])) / math.sqrt(
                    ((float(linhaCalcularNum[1])) * (float(linhaCalcularNum[1]))) + 
                    ((float(linhaCalcularNum[2])) * (float(linhaCalcularNum[2])))
                ))*(1/(math.pi/180)))

                angleYaw = offsetYaw +(math.atan(((float(linhaCalcularNum[2]))/math.sqrt((float(linhaCalcularNum[0]))*(float(linhaCalcularNum[0]))+(float(linhaCalcularNum[1]))*(float(linhaCalcularNum[1]))))))*(1/(math.pi/180))

                TT = (float(linhaCalcularNum[7]) - float(tempoInicial)) # TT é o tempo desde o começo do programa

                if num == (n_sensor-1):
                    #Variável temposg é o tempo do último IMU utilizado que será o tempo considerado no arquivo .sto
                    temposg = float(TT)


                #Matriz Z do filtro de Kalman (saída) recebe os valores do acelerometro
                z[0] = angleRoll
                z[1] = anglePitch
                z[2] = angleYaw

                # --- CALIBRAÇÃO DO GIROSCÓPIO ---
                ratePitch =  ((float(linhaCalcularNum[4])) - rateCalibrationPitch[num])
                rateYaw = ((float(linhaCalcularNum[5])) - rateCalibrationYaw[num])
                rateRoll = ((float(linhaCalcularNum[6])) - rateCalibrationRoll[num])

                #Matriz u do filtro de Kalman (entrada) recebe os valores do giroscópio
                u[0] = rateRoll
                u[1] = ratePitch
                u[2] = rateYaw

                # --- INÍCIO DO CÁLCULO DE KALMAN ---
                # --- ETAPA 1: PREDIÇÃO ---
                x, P = predict(x, P, a, q, u, b)

                # --- ETAPA 2: ATUALIZAÇÃO ---
                S = dot(dot(c, P), c.T) + r

                is_nosingular = determinantOfMatrix(S)

                k = dot(dot(P, c.T), S)

                if is_nosingular:
                    x, P = update(x, P, z, r, c)

                    # Conversão para quaterniões
                    kalman_quaternion = getQuaternionFromEuler(
                        x[0, 0] * (math.pi / 180),
                        x[1, 0] * (math.pi / 180),
                        x[2, 0] * (math.pi / 180)
                    )

                    quart.append(kalman_quaternion)

            if len(quart) == n_sensor:
                linha = ''
                outputFile.write(str(temposg))
                outputFile.write('\t')
                for quat in quart:
                    linha += ','.join(str(val) for val in quat)  # Transformar cada valor do quaternion em string e separar por tab
                    linha += '\t'
                    colunasEscritas += 1
                    if colunasEscritas == n_sensor:
                        linha += '\n'  # Adicionar nova linha após cada quaternion
                        colunasEscritas = 0
                outputFile.write(linha)  # Adicionar nova linha após cada quaternion

            quart = []
