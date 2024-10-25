import numpy as np
from numpy import dot
import math
from filterpy.kalman import predict, update
import matplotlib.pyplot as plt

float_formartter = "{:.6f}".format
np.set_printoptions(formatter={'float_kind':float_formartter})

def getQuaternionFromEuler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.

  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
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
    # --- CONSTANTES ---
    n_sensor = 5
    colunasEscritas = 0
    filePath = 'data/imu/espDataFiltered.txt'

    # -------
    i = 0
    count = 0
    quart = []
    temposg = []

    #Vetores com os valores de calibração do giroscópio nos 3 eixos
    rateCalibrationRoll = [-0.09, -3.68, -4.72, -4.2, 7.55, -3.95, 2.65, -26.7, -1.0, -3.65, 0.0, 0.0, 0.0]
    rateCalibrationPitch = [0.63, -2.89, 1.93, 2.45, 1.07, -2.74, -3.26, -1.29, 0.72, -0.31, 0.0, 0.0, 0.0]
    rateCalibrationYaw = [0.44, -0.42, -0.91, -1.15, 1.87, -0.34, -0.59, 0.65, 0.62, 0.45, 0.0, 0.0, 0.0]

    #Vetores com os valores de calibração do acelerômetro nos 3 eixos
    acelCalibrationRoll = [1.030966, 1.027678, 1.041764,  1.074233, 1.042983, 1.03, 1.06, 1.04, 1.03, 0.96, 0.0, 0.0, 0.0]
    acelCalibrationPitch =[1.003606, 0.983924, 0.987624, 0.98639, 0.98764, 1.0, 0.99, 0.98, 0.99, 1.01, 0.0, 0.0, 0.0]
    acelCalibrationYaw = [0.96429, 1.072670, 0.972605,  0.749889, 1.014723, 0.99, 0.74, 1.0, 0.95, 0.89, 0.0, 0.0, 0.0]

    #Matrizes utilizadas no filtro de Kalman
    #Matriz C do filtro de Kalman
    c = np.array([
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    ])

    #Matriz Xponto do filtro de Kalman
    x = np.array([
        [0.0],
        [0.0],
        [0.0],
        [0.0],
        [0.0],
        [0.0]
    ])

    #Matriz u do filtro de Kalman
    u = np.array([
        [0.0],
        [0.0],
        [0.0]
    ])

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
                    tempo_referencia = float(linhaReferenciaNum[7])  # Pega o último valor da linha
                    tempo_calculo = float(linhaCalcularNum[7])  # Pega o último valor da linha
                except (IndexError, ValueError):
                    print(linhaReferenciaNum)
                    print(linhaCalcularNum)
                    print(f"Erro ao acessar tempo para o sensor {num}.")
                    continue

                Ti = tempo_calculo - tempo_referencia
                if Ti <= 0:
                    print(linhaCalcular)
                    print(linhaReferencia)
                    print(f"Delta T inválido para o sensor {num}.")
                    continue

                #Matriz Q do filtro de Kalman
                q = np.array([
                    [m_roll_real * Ti * Ti, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, m_pitch_real *Ti * Ti, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, m_yaw_real *Ti *Ti, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, m_roll_real, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, m_pitch_real, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, m_yaw_real]
                ])

                #Matriz B do filtro de Kalman
                b = np.array([
                    [Ti, 0.0, 0.0],
                    [0.0, Ti, 0.0],
                    [0.0, 0.0, Ti],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0]
                ])

                #Matriz A do filtro de Kalman
                a = np.array([
                    [1.0, 0.0, 0.0, -Ti, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0, -Ti, 0.0],
                    [0.0, 0.0, 1.0, 0.0, 0.0, -Ti],
                    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                ])

                # --- CALIBRAÇÃO DO ACELERÔMETRO ---
                linhaCalcularNum[1] = (float(linhaCalcularNum[1]) - (acelCalibrationRoll[num]))
                linhaCalcularNum[2] = (float(linhaCalcularNum[2]) - (acelCalibrationPitch[num]))
                linhaCalcularNum[3] = (float(linhaCalcularNum[3]) - (acelCalibrationYaw[num]))

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