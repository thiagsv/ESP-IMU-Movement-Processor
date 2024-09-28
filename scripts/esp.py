import requests

buffer = ""  # Buffer global para dados parciais

def process_chunk(chunk):
    global buffer
    buffer += chunk  # Adiciona o novo chunk ao buffer
    lines = buffer.split(';')  # Divide o buffer em linhas completas
    processed_data = []  # Lista para armazenar dados processados

    # Processa todas as linhas completas, exceto a última
    for line in lines[:-1]:
        try:
            processed_data.append(process_line(line))  # Adiciona os dados processados à lista
        except ValueError as e:
            print(f"Erro ao processar a linha: {line}. Erro: {e}")

    # Guarda a última parte da linha incompleta no buffer
    buffer = lines[-1]

    return processed_data  # Retorna os dados processados

def process_line(line):
    line = line.strip()
    
    if line:  # Certifique-se de que a linha não está vazia
        values = [float(value) for value in line.split(',') if value]
        return values
    return []

def requestIMUData(ip):
    filePath = 'data/imu/espData.txt'
    url = f'http://{ip}/getData'
    try:
        with requests.get(url, stream=True) as response:
            response.raise_for_status()
            with open(filePath, 'wb') as f:  # Abre o arquivo para escrita (sobrescreve o arquivo)
                for chunk in response.iter_content(chunk_size=1024):
                    chunk_decoded = chunk.decode('utf-8')  # Decodifica o chunk para string
                    processed_data = process_chunk(chunk_decoded)  # Processa o chunk
                    # Grava os dados processados no arquivo
                    for data in processed_data:
                        f.write(','.join(map(str, data)) + ';\n')  # Escreve os dados processados no formato desejado

            # Limpa o buffer após o processamento completo
            buffer = ""  # Reseta o buffer após a coleta
        return filePath
    except requests.RequestException as e:
        print(f"Error connecting to ESP32: {e}")
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

def requestCollectESP32(ip, collect):
    """
    Faz uma requisição POST para iniciar ou finalizar a coleta de dados no ESP32.

    Args:
    ip (str): O endereço IP do ESP32.
    collect (str): '1' para iniciar a coleta, '0' para finalizar.
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

