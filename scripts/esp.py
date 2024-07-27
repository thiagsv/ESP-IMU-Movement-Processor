import aiohttp
from scripts. kalmanFilter import processEspData

async def requestEspData(url):
    """
    Requests data from an ESP32 device.

    This function establishes an asynchronous HTTP GET request to the specified URL,
    retrieves the data in chunks, and processes the data using the `processEspData` function.

    Args:
    url (str): The URL of the ESP32 device to request data from.

    Returns:
    str: The processed data from the ESP32 device or an error message in case of connection issues.
    """
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(url) as response:
                response.raise_for_status()
                espData = ''
                async for chunk in response.content.iter_chunked(1024):
                    espData += chunk.decode('utf-8')
                return processEspData(espData)
    except aiohttp.ClientError as e:
        return f'Error connecting to ESP32: {e}'
