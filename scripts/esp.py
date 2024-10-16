import requests

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

            # Define o tamanho do chunk
            chunk_size = 1024

            # Abre o arquivo para escrita
            with open(filePath, "a") as f:
                for chunk in response.iter_content(chunk_size=chunk_size):
                    if chunk:  # Apenas processa chunks não vazios
                        try:
                            chunk_decoded = chunk.decode('utf-8')  # Decodifica o chunk para string
                            # print(f"Chunk recebido: {chunk_decoded}")  # Log para depuração
                            f.write(chunk_decoded)  # Escreve o chunk decodificado no arquivo
                        except UnicodeDecodeError as e:
                            print(f"Erro de decodificação: {e}")
                            return None

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
