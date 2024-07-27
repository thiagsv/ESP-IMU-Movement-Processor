import aiohttp
from scripts. kalmanFilter import processEspData

async def requestEspData(url):
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
