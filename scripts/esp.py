import requests

buffer = ""  # Buffer global para armazenar dados parciais
incomplete_value = ""  # Variável global para armazenar valores incompletos

def processChunk(chunk, fileHandle):
    """
    Processa um chunk de dados e salva no arquivo sempre que uma linha completa for identificada.

    Args:
    chunk (str): O chunk de dados recebido.
    fileHandle (File): O arquivo onde os dados serão gravados.
    """
    global buffer, incomplete_value
    buffer += chunk  # Adiciona o novo chunk ao buffer

    # Continua processando enquanto houver uma linha completa terminada por ';\n'
    while ';\n' in buffer:
        # Divide o buffer na primeira ocorrência de ';\n', mantendo a parte incompleta no buffer
        line, buffer = buffer.split(';\n', 1)

        # Se havia um valor incompleto (por exemplo, '-12'), concatenamos ao primeiro valor da linha
        if incomplete_value:
            line = incomplete_value + line
            incomplete_value = ""  # Resetar a variável após a concatenação

        # Processa a linha completa
        processedLine = processLine(line.strip())

        if processedLine:
            # Converte a lista de dados para string antes de escrever no arquivo
            outputLine = ','.join(map(str, processedLine)) + ';\n'
            fileHandle.write(outputLine.encode('utf-8'))  # Grava os dados como bytes

    # Se sobrar um número incompleto no buffer (que não foi seguido por ';\n')
    if buffer and not buffer.endswith(';\n'):
        incomplete_value = buffer  # Armazena o valor incompleto
        buffer = ""  # Limpa o buffer para aguardar o próximo chunk
    else:
        incomplete_value = ""  # Limpar o valor incompleto se não houver mais dados pendentes

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
        # Remove qualquer ponto e vírgula ou newline residual que possa estar no valor
        cleaned_value = value.replace(';\n', '').strip()

        try:
            processedValues.append(float(cleaned_value))  # Converte para float
        except ValueError:
            print(f"Erro ao converter o valor para float: '{cleaned_value}' na linha '{line}'")
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

            # Não precisa mais verificar o Content-Length se a resposta é chunked
            chunk_size = 1024  # Valor padrão, já adequado

            with open(filePath, 'wb') as f:
                for chunk in response.iter_content(chunk_size=chunk_size):
                    chunkDecoded = chunk.decode('utf-8')  # Decodifica o chunk para string
                    print(f"Chunk recebido: {chunkDecoded}")  # Log para depuração
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
