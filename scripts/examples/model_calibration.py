# Exemplo de código para calibrar um modelo IMU

import opensim as osim
from math import pi
import datetime
# Caminho relativo para o arquivo do modelo
modelPath = 'Rajagopal_2015.osim'

# Path para o arquivo de orientações
orientationsFileName = 'data/mock/orientations.sto'

# Configurar outras variáveis
sensor_to_opensim_rotations = osim.Vec3(-pi/2, 0, 0)
baseIMUName = 'pelvis_imu'
baseIMUHeading = 'z'
visulizeCalibration = True

# Instanciar um objeto IMUPlacer
imuPlacer = osim.IMUPlacer()

# Configurar propriedades para o IMUPlacer
imuPlacer.set_model_file(modelPath)
imuPlacer.set_orientation_file_for_calibration(orientationsFileName)
imuPlacer.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations)
imuPlacer.set_base_imu_label(baseIMUName)
imuPlacer.set_base_heading_axis(baseIMUHeading)

# Executar o IMUPlacer
imuPlacer.run(visulizeCalibration)

# Obter o modelo calibrado
model = imuPlacer.getCalibratedModel()

# Salvar o modelo calibrado como arquivo XML
nome_arquivo_saida = f'calibrated_example.osim'
model.printToXML(nome_arquivo_saida)
