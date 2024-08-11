import requests

def requestIMUData(ip):
    """
    Requests IMU data from an ESP32 device.

    Args:
    ip (str): The IP of the ESP32 device to request data from.

    Returns:
    str: The processed data from the ESP32 device or an error message in case of connection issues.
    """
    filePath = 'data/imu/espData.txt'
    url = f'http://{ip}/getData'
    try:
        with requests.get(url, stream=True) as response:
            response.raise_for_status()
            with open(filePath, 'wb') as f:
                for chunk in response.iter_content(chunk_size=1024):
                    f.write(chunk)
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
    requestCollectESP32(ip, '1')

def finishCollectIMUData(ip):
    """
    Finaliza a coleta de dados do IMU no ESP32.

    Args:
    ip (str): O endereço IP do ESP32.
    """
    requestCollectESP32(ip, '0')

def requestCollectESP32(ip, collect):
    """
    Faz uma requisição POST para iniciar ou finalizar a coleta de dados no ESP32.

    Args:
    ip (str): O endereço IP do ESP32.
    collect (str): '1' para iniciar a coleta, '0' para finalizar.
    """
    url = f'http://{ip}/toggle'
    payload = {
        'collect': collect
    }
    headers = {
        'Content-Type': 'application/json'
    }

    try:
        response = requests.post(url, json=payload, headers=headers)
        response.raise_for_status()
    except requests.exceptions.RequestException as err:
        print(f'Ocorreu um erro ao iniciar a coleta de dados: {err}')

