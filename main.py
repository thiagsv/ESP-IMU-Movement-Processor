import tkinter as tk
from tkinter import simpledialog, messagebox
from scripts.opensim import getCalibrationDataPath, generateCalibratedModel, trackingMovement
from scripts.esp import requestIMUData, startCollectIMUData
from scripts.kalmanFilter import processIMUData

def runSimulation(mock, startAt = None, endAt = None):
    getCalibrationDataPath(mock)
    generateCalibratedModel()
    trackingMovement(mock, startAt, endAt)

def startSimulation():
    mock = messagebox.askyesno('Mock Data', 'Deseja usar dados mock?')
    
    if not mock:
        realStartAt = simpledialog.askstring('Entrada', 'Digite a data de início (YYYY-MM-DD HH:ii:ss):')
        realEndAt = simpledialog.askstring('Entrada', 'Digite a data de término (YYYY-MM-DD HH:ii:ss):')
        runSimulation(mock, realStartAt, realEndAt)
    else:
        runSimulation(mock)

def getEspData():
    ip = simpledialog.askstring('Entrada', 'Digite o IP do ESP32 (e.g., 192.168.4.1):')
    if ip:
        filePath = requestIMUData(ip)
        if filePath:
            processIMUData(filePath)
        else:
            messagebox.showerror('Erro', 'Falha ao obter dados do ESP32')

def startCollecting():
    ip = simpledialog.askstring('Entrada', 'Digite o IP do ESP32 (e.g., 192.168.4.1):')
    if ip:
        startCollectIMUData(ip)
    else:
        messagebox.showerror('Erro', 'Falha ao iniciar a coleta de dados do ESP32')    

def main():
    window = tk.Tk()
    window.title('OpenSim GUI')
    window.geometry('300x150')

    startCollectBtn = tk.Button(window, text='Iniciar a coleta de dados', command=startCollecting)
    startCollectBtn.pack(pady=20)

    simulateBtn = tk.Button(window, text='Executar Scripts', command=startSimulation)
    simulateBtn.pack(pady=20)

    getEspDataBtn = tk.Button(window, text='Obter Dados do ESP32', command=getEspData)
    getEspDataBtn.pack(pady=20)    
    
    window.mainloop()

if __name__ == '__main__':
    main()
