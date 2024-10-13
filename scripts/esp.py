import requests

buffer = ""  # Buffer global para armazenar dados parciais

def processChunk(chunk, fileHandle):
    """
    Processa um chunk de dados e salva no arquivo sempre que uma linha completa for identificada.

    Args:
    chunk (str): O chunk de dados recebido.
    fileHandle (File): O arquivo onde os dados serão gravados.
    """
    global buffer
    buffer += chunk  # Adiciona o novo chunk ao buffer

    # Continua processando enquanto houver uma linha completa terminada por ';\n'
    while ';\n' in buffer:  
        # Divide o buffer na primeira ocorrência de ';\n', mantendo a parte incompleta no buffer
        line, buffer = buffer.split(';\n', 1)
        # Processa a linha completa, removendo qualquer espaço em branco e o ';'
        processedLine = processLine(line.strip())

        if processedLine:
            # Converte a lista de dados para string antes de escrever no arquivo
            outputLine = ','.join(map(str, processedLine)) + ';\n'
            fileHandle.write(outputLine.encode('utf-8'))  # Grava os dados como bytes

def processLine(line):
    """
    Processa uma linha de dados do IMU.
    A linha está no formato: 'AcX,AcY,AcZ,GyX,GyY,GyZ'

    Args:
    line (str): A linha contendo os dados do IMU no formato esperado.

    Returns:
    list: Uma lista de valores float correspondentes aos dados do IMU.
    """
    # Separa os valores que estão separados por vírgula
    values = line.split(',')

    # Converte os valores para float, ignorando qualquer valor inválido
    processedValues = []
    for value in values:
        try:
            processedValues.append(float(value))  # Converte para float
        except ValueError:
            print(f"Erro ao converter o valor para float: '{value}' na linha '{line}'")
            # Se o valor for inválido, ignoramos ele

    return processedValues

def requestIMUData(ip):
    """
    Requests IMU data from an ESP32 device.

    Args:
    ip (str): The IP of the ESP32 device to request data from.

    Returns:
    str: The path to the processed data file or an error message in case of connection issues.
    """
    filePath = 'data/imu/espData.txt'
    url = f'http://{ip}/getData'
    
    try:
        with requests.get(url, stream=True) as response:
            response.raise_for_status()

            content_length = response.headers.get('Content-Length')
            if content_length:
                chunk_size = min(1024, int(content_length) // 10)  # Ajusta dinamicamente o chunk
            else:
                chunk_size = 1024  # Valor padrão

            with open(filePath, 'wb') as f:
                for chunk in response.iter_content(chunk_size=1024):
                    chunkDecoded = chunk.decode('utf-8')  # Decodifica o chunk para string
                    processChunk(chunkDecoded, f)  # Processa o chunk e escreve diretamente no arquivo

        return filePath
    except requests.RequestException as e:
        print(f'Error connecting to ESP32: {e}')
        return None

def startCollectIMUData(ip):
    """
    Inicia a coleta de dados do IMU no ESP32.

    Args:
    ip (str): O endereço IP do ESP32.
    """
    requestCollectESP32(ip)

def finishCollectIMUData(ip):
    """
    Finaliza a coleta de dados do IMU no ESP32.

    Args:
    ip (str): O endereço IP do ESP32.
    """
    requestCollectESP32(ip)

def requestCollectESP32(ip):
    """
    Faz uma requisição POST para iniciar ou finalizar a coleta de dados no ESP32.

    Args:
    ip (str): O endereço IP do ESP32.
    """
    url = f'http://{ip}/toggle'

    headers = {
        'Content-Type': 'application/json'
    }

    try:
        response = requests.post(url, json={}, headers=headers)
        response.raise_for_status()
    except requests.exceptions.RequestException as err:
        print(f'Ocorreu um erro ao iniciar a coleta de dados: {err}')
