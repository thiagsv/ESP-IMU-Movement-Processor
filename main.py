import asyncio
import tkinter as tk
from tkinter import simpledialog, messagebox
from scripts.opensim import getCalibrationDataPath, generateCalibratedModel, trackingMovement
from scripts.esp import requestEspData

def runSimulation(mock, startAt=None, endAt=None):
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
    url = simpledialog.askstring('Entrada', 'Digite o URL do ESP32 (e.g., http://192.168.4.1/data):')
    if url:
        loop = asyncio.get_event_loop()
        dados = loop.run_until_complete(requestEspData(url))
        if dados:
            messagebox.showinfo('Dados do ESP32', f'Dados recebidos: {dados}')
        else:
            messagebox.showerror('Erro', 'Falha ao obter dados do ESP32')

def main():
    window = tk.Tk()
    window.title('OpenSim GUI')
    window.geometry('300x150')

    simulateBtn = tk.Button(window, text='Executar Scripts', command=startSimulation)
    simulateBtn.pack(pady=20)

    getEspDataBtn = tk.Button(window, text='Obter Dados do ESP32', command=getEspData)
    getEspDataBtn.pack(pady=20)
    window.mainloop()

if __name__ == '__main__':
    main()
