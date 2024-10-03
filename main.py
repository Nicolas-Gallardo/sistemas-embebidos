import sys
import receiver
import PyQt5.QtWidgets as pw
from PyQt5.QtGui import QIntValidator
from PyQt5.QtCore import pyqtSlot
import pyqtgraph as pg
# pip install pyqtgraph

class MainWindow(pw.QMainWindow):
    def __init__(self):
        super().__init__()
        #Ajustes de parametros iniciales
        self.title = 'Visualizador información de BME'
        self.left = 50
        self.top = 50
        self.width = 600
        self.height = 450
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

        # Metricas para RMS
        self.tempRMS = pw.QLabel('RMS de temperatura ' + str(temp_rms))
        self.pressRMS = pw.QLabel('RMS de presión ' + str(press_rms))

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
        graphLayout.addWidget(self.plotPress, 0, 1)
        graphLayout.addWidget(self.tempRMS, 1, 0)
        graphLayout.addWidget(self.pressRMS, 1, 1)
        
        # Agregar sublayouts al principal
        mainLayout.addLayout(btnLayout)
        mainLayout.addLayout(graphLayout)        

        # Set layout
        widget = pw.QWidget()
        widget.setLayout(mainLayout)
        self.setCentralWidget(widget)
    
    @pyqtSlot()
    def request(self):
        # Añadir los datos a TEMP, PRESS, TEMP_RMS y PRESS_RMS
        print("Request")
        global temp, press, hum, gas, temp_rms, press_rms, hum_rms, gas_rms
        temp, press, hum, gas, temp_rms, press_rms, hum_rms, gas_rms = receiver.recieve_window_data()
        #temp, press, temp_rms, press_rms = receiver.recieve_window_data()
        self.update_data()
                
    
    @pyqtSlot()
    def update_data(self):
        time = range(0, data_window_size)
        self.plotTemp.clear()
        self.plotPress.clear()
        self.plotTemp.plot(time, temp)
        self.plotPress.plot(time, press)
        self.tempRMS.setText('RMS de temperatura ' + str(temp_rms))
        self.pressRMS.setText('RMS de presión ' + str(press_rms))

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
temp = []
temp_rms = 0
hum = []
hum_rms = 0
gas = []
gas_rms = 0

if __name__ == '__main__':
    receiver.start_conn()
    tries = 0;
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
