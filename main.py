import datetime
import tkinter as tk
from tkinter import simpledialog, messagebox
from scripts.opensim import getCalibrationDataPath, generateCalibratedModel, getTrackingDataPath, trackingMovement

def runSimulation(mock, startAt=None, endAt=None):
    dateTime = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

    getCalibrationDataPath(mock, dateTime)
    generateCalibratedModel(dateTime)
    getTrackingDataPath(mock, dateTime)
    trackingMovement(mock, startAt, endAt)

def getInfo():
    mock = messagebox.askyesno("Mock Data", "Deseja usar dados mock?")
    
    if not mock:
        realStartAt = simpledialog.askstring("Entrada", "Digite a data de início (YYYY-MM-DD):")
        realEndAt = simpledialog.askstring("Entrada", "Digite a data de término (YYYY-MM-DD):")
        runSimulation(mock, realStartAt, realEndAt)
    else:
        runSimulation(mock)

def main():
    window = tk.Tk()
    window.title("OpenSim GUI")

    botao_executar = tk.Button(window, text="Executar Scripts", command=getInfo)
    botao_executar.pack(pady=20)

    window.mainloop()

if __name__ == '__main__':
    main()
