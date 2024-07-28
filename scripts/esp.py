import requests

def requestIMUData(url):
    """
    Requests IMU data from an ESP32 device.

    Args:
    url (str): The URL of the ESP32 device to request data from.

    Returns:
    str: The processed data from the ESP32 device or an error message in case of connection issues.
    """
    filePath = 'data/imu/espData.txt'
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
