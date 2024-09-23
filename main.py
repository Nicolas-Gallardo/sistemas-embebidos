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
        windowLine = pw.QLineEdit(str(DATA_WINDOW_SIZE), self)
        # Validador de solo numeros
        windowLine.setValidator(QIntValidator())

        # Boton para cambiar ventana
        windowBtn = pw.QPushButton('Cambiar ventana de datos', self)
        # Conectar con funcion update_window_size
        windowBtn.clicked.connect(lambda: self.update_window_size(int(windowLine.text())))

        # Label para mostrar el window_size actual
        self.windowLabel = pw.QLabel('Tamaño de ventana de datos es ' + str(DATA_WINDOW_SIZE))

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
        self.plotTemp.plot(TEMP)
        # Leyenda del grafico
        self.plotTemp.setTitle("Temperatura vs Tiempo")
        self.plotTemp.setLabel("left", "Temperatura (°C)")
        self.plotTemp.setLabel("bottom", "Tiempo (s)")
        # Grafico para presion
        self.plotPress = pg.PlotWidget()
        self.plotPress.plot(PRESS)
        # Leyenda del grafico
        self.plotPress.setTitle("Presión vs Tiempo")
        self.plotPress.setLabel("left", "Presión (hPa)")
        self.plotPress.setLabel("bottom", "Tiempo (s)")

        # Metricas para RMS
        self.tempRMS = pw.QLabel('RMS de temperatura ' + str(TEMP_RMS))
        self.pressRMS = pw.QLabel('RMS de presión ' + str(PRESS_RMS))

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
        TEMP, PRESS, TEMP_RMS, PRESS_RMS = receiver.recieve_window_data()
        print(f"temp: {TEMP}")
        print(f"press: {PRESS}")
        print(f"temp_rms: {TEMP_RMS}")
        print(f"press_rms: {PRESS_RMS}")
        self.update_plot()
                
    
    @pyqtSlot()
    def update_plot(self):
        self.plotTemp.plot(TEMP, list(range(0,DATA_WINDOW_SIZE)))
        self.plotPress.plot(PRESS, list(range(0,DATA_WINDOW_SIZE)))

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
        if receiver.set_window_size(window):
            DATA_WINDOW_SIZE = window
        self.windowLabel.setText('Tamaño de ventana de datos es ' + str(DATA_WINDOW_SIZE))

        
#Variables globales
DATA_WINDOW_SIZE = 10
PRESS = [0, 10 , 20]
PRESS_RMS = 0
TEMP = [0, 15, 55]
TEMP_RMS = 0

if __name__ == '__main__':
    receiver.start_conn()
    DATA_WINDOW_SIZE = receiver.get_window_size()
    app = pw.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()
