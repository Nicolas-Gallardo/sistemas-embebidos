import sys
import receiver
import PyQt5.QtWidgets  as pw
from PyQt5.QtGui import QIntValidator
from PyQt5.QtCore import pyqtSlot
import pyqtgraph as pg

class MainWindow(pw.QMainWindow):
    def __init__(self):
        super().__init__()
        self.title = 'Visualizador infromacion de BME'
        self.left = 50
        self.top = 50
        self.width = 320
        self.height = 200
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        # Input line that sets window size
        windowLine = pw.QLineEdit(text = str(DATA_WINDOW_SIZE), parent=self)
        windowLine.setValidator(QIntValidator())
        windowLine.textEdited.connect(self.update_window_size)
        # Button that requests the data
        requestBtn = pw.QPushButton('Solicitar datos', self)
        requestBtn.setFixedSize(200, 120)
        requestBtn.clicked.connect(self.request)
        # Button that closes connection
        closeBtn = pw.QPushButton('Cerrar conexión', self)
        closeBtn.setFixedSize(200, 120)
        closeBtn.clicked.connect(self.end)
        # Graphs
        plot_graph = pg.PlotWidget()
        plot_graph.plot(TEMP, PRESS)
        # Metrics

        # Create layouts
        mainLayout = pw.QVBoxLayout()
        btnLayout = pw.QVBoxLayout()
        graphLayout = pw.QHBoxLayout()
        # Add widgets
        btnLayout.addWidget(windowLine)
        btnLayout.addWidget(requestBtn)
        btnLayout.addWidget(closeBtn)
        graphLayout.addWidget(plot_graph)
        # Add sub layouts to main
        mainLayout.addLayout(btnLayout)
        mainLayout.addLayout(graphLayout)
        #Set layout
        widget = pw.QWidget()
        widget.setLayout(mainLayout)
        self.setCentralWidget(widget)
    
    @pyqtSlot()
    def request(self):
        data_list = receiver.recieve_window_data(DATA_WINDOW_SIZE)
        print(data_list)
        # Graficar los datos
    
    @pyqtSlot()
    def end(self):
        receiver.restart_ESP()
        self.close()
    
    @pyqtSlot()
    def update_window_size(self):
        DATA_WINDOW_SIZE = self.windowLine.text()
        receiver.set_window_size(DATA_WINDOW_SIZE)

DATA_WINDOW_SIZE = 10
PRESS = [0]
TEMP = [0]

if __name__ == '__main__':
    app = pw.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()
