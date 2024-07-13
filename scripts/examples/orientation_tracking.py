# Exemplo de código para realizar rastreamento de orientação com OpenSense.
# Este script utiliza as funções da biblioteca OpenSense e faz parte dos arquivos de exemplo do OpenSense.

# Importar as bibliotecas do OpenSim
import opensim as osim
from math import pi

# Definir variáveis a serem utilizadas
modelFileName = 'calibrated_example.osim';                # Caminho para o modelo de entrada
orientationsFileName = 'data/mock/orientations.sto';   # Caminho para os dados de orientações para calibração
sensor_to_opensim_rotation = osim.Vec3(-pi/2, 0, 0);             # Rotação dos dados do IMU para o quadro mundial do OpenSim
visualizeTracking = True;  # Booleano para visualizar a simulação de rastreamento
startTime = 7.25;          # Tempo inicial (em segundos) da simulação de rastreamento
endTime = 15;              # Tempo final (em segundos) da simulação de rastreamento
resultsDirectory = 'IKResults/examples';  # Diretório para armazenar os resultados

# Instanciar uma ferramenta de Cinemática Inversa com IMU (IMUInverseKinematicsTool)
imuIK = osim.IMUInverseKinematicsTool();

# Configurar as propriedades da ferramenta
imuIK.set_model_file(modelFileName);              # Definir o arquivo do modelo
imuIK.set_orientations_file(orientationsFileName); # Definir o arquivo de orientações
imuIK.set_sensor_to_opensim_rotations(sensor_to_opensim_rotation)  # Definir a rotação dos sensores para o quadro do OpenSim
imuIK.set_results_directory(resultsDirectory)     # Definir o diretório de resultados

# Definir o intervalo de tempo em segundos
imuIK.set_time_range(0, startTime);   # Intervalo de tempo inicial
imuIK.set_time_range(1, endTime);     # Intervalo de tempo final

# Executar a Cinemática Inversa
imuIK.run(visualizeTracking);  # Executar com visualização de rastreamento se True
