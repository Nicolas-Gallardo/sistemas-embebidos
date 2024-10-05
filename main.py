import sys

import PyQt5.QtWidgets as pw
import pyqtgraph as pg
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QIntValidator

import receiver

# pip install pyqtgraph

class MainWindow(pw.QMainWindow):
    def __init__(self):
        super().__init__()
        #Ajustes de parametros iniciales
        self.title = 'Visualizador información de BME'
        self.left = 50
        self.top = 50
        self.width = 700
        self.height = 800
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        # Linea de input para el nuevo tamaño de ventana
        windowLine = pw.QLineEdit(str(data_window_size), self)
        # Validador de solo numeros
        windowLine.setValidator(QIntValidator())

        # Boton para cambiar ventana
        windowBtn = pw.QPushButton('Cambiar ventana de datos', self)
        # Conectar con funcion update_window_size
        windowBtn.clicked.connect(lambda: self.update_window_size(int(windowLine.text())))

        # Label para mostrar el window_size actual
        self.windowLabel = pw.QLabel('Tamaño de ventana de datos es ' + str(data_window_size))

        # Boton que pide datos
        requestBtn = pw.QPushButton('Solicitar datos', self)
        # Conectar a funcion request
        requestBtn.clicked.connect(self.request)

        # Boton para cerrar conexion
        closeBtn = pw.QPushButton('Cerrar conexión', self)
        # Conectar con funcion end
        closeBtn.clicked.connect(self.end)

        # Grafico para temperatura
        self.plotTemp = pg.PlotWidget()
        self.plotTemp.plot(temp)
        # Leyenda del grafico
        self.plotTemp.setTitle("Temperatura vs Tiempo")
        self.plotTemp.setLabel("left", "Temperatura (°C)")
        self.plotTemp.setLabel("bottom", "Tiempo (s)")
        # Grafico para presion
        self.plotPress = pg.PlotWidget()
        self.plotPress.plot(press)
        # Leyenda del grafico
        self.plotPress.setTitle("Presión vs Tiempo")
        self.plotPress.setLabel("left", "Presión (hPa)")
        self.plotPress.setLabel("bottom", "Tiempo (s)")
        # Grafico para humedad
        self.plotHum = pg.PlotWidget()
        self.plotHum.plot(hum)
        # Leyenda del grafico
        self.plotHum.setTitle("Humedad vs Tiempo")
        self.plotHum.setLabel("left", "Humedad (%)")
        self.plotHum.setLabel("bottom", "Tiempo (s)")
        # Grafico para co
        self.plotCO = pg.PlotWidget()
        self.plotCO.plot(co)
        # Leyenda del grafico
        self.plotCO.setTitle("Co vs Tiempo")
        self.plotCO.setLabel("left", "Concentracion de CO (kΩ)")
        self.plotCO.setLabel("bottom", "Tiempo (s)")

        # Metricas para RMS
        self.tempRMS = pw.QLabel('RMS de temperatura ' + str(temp_rms))
        self.pressRMS = pw.QLabel('RMS de presión ' + str(press_rms))
        self.humRMS = pw.QLabel('RMS de humedad ' + str(hum_rms))
        self.coRMS = pw.QLabel('RMS de concentracion de CO ' + str(co_rms))

        # Metricas para FFT
        self.tempFFT = pw.QLabel('FFT de temperatura ' + str(temp_fft))
        self.pressFFT = pw.QLabel('FFT de presión ' + str(press_fft))
        self.humFFT = pw.QLabel('FFT de humedad ' + str(hum_fft))
        self.coFFT = pw.QLabel('FFT de concentracion de CO ' + str(co_fft))

        # Metricas para five peaks
        self.tempFP = pw.QLabel('Cinco peaks de temperatura ' + str(temp_fp))
        self.pressFP = pw.QLabel('Cinco peaks de presión ' + str(press_fp))
        self.humFP = pw.QLabel('Cinco peaks de humedad ' + str(hum_fp))
        self.coFP = pw.QLabel('Cinco peaks de concentracion de CO ' + str(co_fp))

        # Crear layouts
        mainLayout = pw.QVBoxLayout()
        btnLayout = pw.QGridLayout()
        graphLayout = pw.QGridLayout()


        # Agregar widgets
        btnLayout.addWidget(windowLine, 0, 0)
        btnLayout.addWidget(windowBtn, 0, 1)
        btnLayout.addWidget(self.windowLabel, 1, 0, 1, 2)
        btnLayout.addWidget(requestBtn, 2, 0, 1, 2)
        btnLayout.addWidget(closeBtn, 3, 0, 1, 2)
        graphLayout.addWidget(self.plotTemp, 0, 0)
        graphLayout.addWidget(self.tempRMS, 1, 0)
        graphLayout.addWidget(self.tempFFT, 2, 0)
        graphLayout.addWidget(self.tempFP, 3, 0)
        graphLayout.addWidget(self.plotPress, 0, 1)
        graphLayout.addWidget(self.pressRMS, 1, 1)
        graphLayout.addWidget(self.pressFFT, 2, 1)
        graphLayout.addWidget(self.pressFP, 3, 1)
        graphLayout.addWidget(self.plotHum, 4, 0)
        graphLayout.addWidget(self.humRMS, 5, 0)
        graphLayout.addWidget(self.humFFT, 6, 0)
        graphLayout.addWidget(self.humFP, 7, 0)
        graphLayout.addWidget(self.plotCO, 4, 1)
        graphLayout.addWidget(self.coRMS, 5, 1)
        graphLayout.addWidget(self.coFFT, 6, 1)
        graphLayout.addWidget(self.coFP, 7, 1)

        # Agregar sublayouts al principal
        mainLayout.addLayout(btnLayout)
        mainLayout.addLayout(graphLayout)

        # Set layout
        widget = pw.QWidget()
        widget.setLayout(mainLayout)
        self.setCentralWidget(widget)

    @pyqtSlot()
    def request(self):
        print("Request")
        # Tomar los datos globales
        global temp, temp_rms, temp_fft, temp_fp
        global press, press_rms, press_fft, press_fp
        global hum, hum_rms, hum_fft, hum_fp
        global co, co_rms, co_fft, co_fp

        # Añadir los datos a sus respectivas variables
        receiver.request_window_data()
        temp, press, hum, co = receiver.receive_raw()
        temp_fp, press_fp, hum_fp, co_fp = receiver.receive_max()
        temp_rms, press_rms, hum_rms, co_rms = receiver.receive_rms()
        temp_fft, press_fft, hum_fft, co_fft = receiver.receive_fft()

        #Actualizar la data
        self.update_data()


    @pyqtSlot()
    def update_data(self):
        time = range(0, data_window_size)
        self.plotTemp.clear()
        self.plotPress.clear()
        self.plotHum.clear()
        self.plotCO.clear()
        self.plotTemp.plot(time, temp)
        self.plotPress.plot(time, press)
        self.plotHum.plot(time, hum)
        self.plotCO.plot(time, co)
        self.tempRMS.setText('RMS de temperatura ' + str(temp_rms))
        self.pressRMS.setText('RMS de presión ' + str(press_rms))
        self.humRMS.setText('RMS de humedad ' + str(hum_rms))
        self.coRMS.setText('RMS de concentracion de CO ' + str(co_rms))
        self.tempFFT.setText('FFT de temperatura ' + str(temp_fft))
        self.pressFFT.setText('FFT de presión ' + str(press_fft))
        self.humFFT.setText('FFT de humedad ' + str(hum_fft))
        self.coFFT.setText('FFT de concentracion de CO ' + str(co_fft))
        self.tempFP.setText('Cinco peaks de temperatura ' + str(temp_fp))
        self.pressFP.setText('Cinco peaks de presión ' + str(press_fp))
        self.humFP.setText('Cinco peaks de humedad ' + str(hum_fp))
        self.coFP.setText('Cinco peaks de concentracion de CO ' + str(co_fp))

    @pyqtSlot()
    def end(self):
        #Cerrar conexion, reiniciando ESP
        print("Close")
        if receiver.restart_ESP():
            self.close()

    @pyqtSlot()
    def update_window_size(self, window):
        #Actualizar ventana de datos
        print("Update")
        global data_window_size
        if receiver.set_window_size(window):
            data_window_size = window
        self.windowLabel.setText('Tamaño de ventana de datos es ' + str(data_window_size))


#Variables globales
data_window_size = 10
press = []
press_rms = 0
press_fft = 0
press_fp = 0
temp = []
temp_rms = 0
temp_fft = 0
temp_fp = 0
hum = []
hum_rms = 0
hum_fft = 0
hum_fp = 0
co = []
co_rms = 0
co_fft = 0
co_fp = 0

if __name__ == '__main__':
    receiver.start_conn()
    tries = 0
    while True:
        if tries > 3:
            break
        temp_window_size = receiver.get_window_size()
        if temp_window_size > 0:
            data_window_size = temp_window_size
            break
        tries += 1
    app = pw.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()
