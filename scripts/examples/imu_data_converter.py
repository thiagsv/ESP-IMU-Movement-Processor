# Exemplo de código para ler e converter dados do sensor IMU XSENS para
# o formato compatível com OpenSense.

import opensim as osim

# Construir um objeto de configurações Xsens.
# Instanciar a classe de configurações do leitor
xsensSettings = osim.XsensDataReaderSettings('opensim/models/myIMUMappings.xml')
# Instanciar um XsensDataReader
xsens = osim.XsensDataReader(xsensSettings)
# Ler tabelas separadas de dados do arquivo(s) IMU especificado(s)
tables = xsens.read('data/mock/IMUData/')
# Obter o nome do experimento/trial das configurações
trial = xsensSettings.get_trial_prefix()
print(trial)
# Obter dados de Orientação como quaternions
quatTable = xsens.getOrientationsTable(tables)
# Escrever no arquivo
osim.STOFileAdapterQuaternion.write(quatTable,  trial + '_orientations.sto')
# Obter dados de Aceleração
accelTable = xsens.getLinearAccelerationsTable(tables)
# Escrever no arquivo
osim.STOFileAdapterVec3.write(accelTable, trial + '_linearAccelerations.sto')
# Obter dados de Cabeçalho Magnético (Norte)
magTable = xsens.getMagneticHeadingTable(tables)
# Escrever no arquivo
osim.STOFileAdapterVec3.write(magTable, trial + '_magneticNorthHeadings.sto')
# Obter dados de Velocidade Angular
angVelTable = xsens.getAngularVelocityTable(tables)
# Escrever no arquivo
osim.STOFileAdapterVec3.write(angVelTable, trial + '_angularVelocities.sto')
