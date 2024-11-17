import requests

def clearIMUData(ip):
    """
    Envia uma requisição para limpar o arquivo de dados no ESP32.

    Args:
    ip (str): O endereço IP do ESP32.

    Returns:
    bool: True se a exclusão for bem-sucedida, False caso contrário.
    """
    url = f'http://{ip}/clearData'
    try:
        response = requests.post(url)
        response.raise_for_status()
        print("Arquivo de dados limpo com sucesso no ESP32.")
        return True
    except requests.RequestException as e:
        print(f"Erro ao limpar o arquivo de dados no ESP32: {e}")
        return False

def requestIMUData(ip):
    """
    Requests IMU data from an ESP32 device.
    
    Args:
    ip (str): The IP of the ESP32 device to request data from.
    
    Returns:
    str: The path to the processed data file or an error message in case of connection issues.
    """
    filePath = 'data/imu/espData.txt'
    filteredFilePath = 'data/imu/espDataFiltered.txt'

    url = f'http://{ip}/getData'
    
    try:
        with requests.get(url, stream=True) as response:
            response.raise_for_status()

            # Define o tamanho do chunk
            chunk_size = 1024

            # Abre o arquivo para escrita
            with open(filePath, 'w') as f:
                for chunk in response.iter_content(chunk_size=chunk_size):
                    if chunk:  # Apenas processa chunks não vazios
                        try:
                            chunk_decoded = chunk.decode('utf-8')  # Decodifica o chunk para string
                            # print(f"Chunk recebido: {chunk_decoded}")  # Log para depuração
                            f.write(chunk_decoded)  # Escreve o chunk decodificado no arquivo
                        except UnicodeDecodeError as e:
                            print(f"Erro de decodificação: {e}")
                            return None
        
        validateAndFilterSets(filePath, filteredFilePath)
        clearIMUData(ip)
        return True
    except requests.RequestException as e:
        print(f'Error connecting to ESP32: {e}')
        clearIMUData(ip)
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


def validateAndFilterSets(inputFile, outputFile):
    result = []
    currentSet = []
    currentLine = ""
    searchingNextSet = False  # Indica se estamos procurando o próximo conjunto
    removedSets = 0

    with open(inputFile, 'r') as infile, open(outputFile, 'w') as outfile:
        while True:
            char = infile.read(1)  # Lê um caractere por vez
            if not char:
                break  # Fim do arquivo
            
            if searchingNextSet:
                # Acumula os caracteres até encontrar ";0,"
                currentLine += char
                if currentLine.endswith(';0,'):
                    searchingNextSet = False  # Encontramos o início do próximo conjunto
                    currentSet = []  # Reseta o conjunto atual
                    currentLine = '0,'  # Reseta a linha inicial para começar do conjunto
                continue

            if char == ';':
                # Fim de uma linha detectado
                columns = currentLine.split(',')

                # Verifica se há exatamente 8 colunas
                if len(columns) == 8:
                    try:
                        # Verifica se o primeiro valor da linha corresponde ao índice esperado
                        expectedIndex = len(currentSet)
                        if int(columns[0]) == expectedIndex:
                            # Adiciona a linha completa (incluindo o ;) ao conjunto atual
                            currentSet.append(currentLine + ';')
                        else:
                            # Linha do conjunto falhou, busca o próximo conjunto começando com ";0,"
                            searchingNextSet = True
                            currentLine = ""
                            removedSets += 1
                            continue
                    except ValueError:
                        # Índice inválido, busca o próximo conjunto começando com ";0,"
                        searchingNextSet = True
                        currentLine = ""
                        removedSets += 1
                        continue
                else:
                    # Número incorreto de colunas, busca o próximo conjunto começando com ";0,"
                    searchingNextSet = True
                    currentLine = ""
                    removedSets += 1
                    continue
                
                # Se o conjunto tem 5 linhas, adiciona ao resultado e reseta
                if len(currentSet) == 5:
                    outfile.write(''.join(currentSet))
                    currentSet = []

                # Reseta a linha atual após o processamento
                currentLine = ""
            else:
                # Acumula o caractere na linha atual
                currentLine += char
    
    print("Conjuntos removidos: ", removedSets)

