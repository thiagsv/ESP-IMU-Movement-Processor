import tkinter as tk
from tkinter import simpledialog, messagebox
from scripts.opensim import getCalibrationDataPath, generateCalibratedModel, trackingMovement
from scripts.esp import requestIMUData, startCollectIMUData, finishCollectIMUData
from scripts.kalmanFilter import processIMUData

startCollectBtn = None
simulateBtn = None
finishCollectBtn = None
ipEsp = None

def runSimulation(mock, startAt=None, endAt=None):
    """
    Executa a simulação usando dados calibrados.

    Args:
    mock (bool): Indica se os dados mock devem ser usados.
    startAt (str, opcional): Data e hora de início da simulação.
    endAt (str, opcional): Data e hora de término da simulação.
    """
    getCalibrationDataPath(mock)
    generateCalibratedModel()
    trackingMovement(mock, startAt, endAt)

def startSimulation():
    """
    Inicia a simulação. Pergunta ao usuário se deseja usar dados mock e coleta as datas se necessário.
    """
    mock = messagebox.askyesno('Mock Data', 'Deseja usar dados mock?')
    if not mock:
        realStartAt = simpledialog.askstring('Entrada', 'Digite o tempo de início de análise desejada:')
        if realStartAt is None:
            showOtherButtons()
            return

        realEndAt = simpledialog.askstring('Entrada', 'Digite o tempo de final de análise desejada:')
        if realEndAt is None:
            showOtherButtons()
            return

        runSimulation(mock, realStartAt, realEndAt)
    else:
        runSimulation(mock)

def getEspData():
    """
    Obtém os dados do ESP32 e processa os dados IMU.
    """
    filePath = requestIMUData(ipEsp)
    if filePath:
        processIMUData(filePath)
    else:
        messagebox.showerror('Erro', 'Falha ao obter dados do ESP32')

def startCollecting():
    """
    Inicia a coleta de dados IMU do ESP32 e oculta os outros botões.
    """
    global ipEsp
    ipEsp = simpledialog.askstring('Entrada', 'Digite o IP do ESP32:', initialvalue='192.168.4.1')
    if ipEsp:
        startCollectIMUData(ipEsp)
        hideOtherButtons()
    else:
        messagebox.showerror('Erro', 'Falha ao iniciar a coleta de dados do ESP32')

def finishCollect():
    """
    Finaliza a coleta de dados IMU do ESP32, obtém os dados e mostra novamente os botões ocultos.
    """
    finishCollectIMUData(ipEsp)
    getEspData()
    showOtherButtons()

def hideOtherButtons():
    """
    Oculta os botões 'Iniciar a coleta de dados', 'Realizar simulação' e 'Obter Dados do ESP32'.
    """
    global startCollectBtn, startCollectBtn, finishCollectBtn

    startCollectBtn.pack_forget()
    simulateBtn.pack_forget()
    finishCollectBtn.pack(pady=10)

def showOtherButtons():
    """
    Mostra os botões 'Iniciar a coleta de dados', 'Realizar simulação' e 'Obter Dados do ESP32', e oculta o botão 'Finalizar a coleta de dados'.
    """
    global startCollectBtn, startCollectBtn, finishCollectBtn

    startCollectBtn.pack(pady=10, padx=10)
    simulateBtn.pack(pady=10, padx=10)
    finishCollectBtn.pack_forget()

def main():
    """
    Configura a interface gráfica e inicializa a aplicação.
    """
    global startCollectBtn, simulateBtn, finishCollectBtn
    window = tk.Tk()
    window.title('OpenSim GUI')
    window.geometry('300x200')

    startCollectBtn = tk.Button(window, text='Iniciar a coleta de dados', command=startCollecting, height=2, width=20)
    simulateBtn = tk.Button(window, text='Realizar simulação', command=startSimulation, height=2, width=20)
    finishCollectBtn = tk.Button(window, text='Finalizar a coleta de dados', command=finishCollect, height=2, width=20)

    startCollectBtn.pack(pady=10)
    simulateBtn.pack(pady=10)
    finishCollectBtn.pack_forget()

    window.mainloop()

if __name__ == '__main__':
    main()
