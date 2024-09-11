import sys
import receiver

from PyQt6.QtCore import QSize
from PyQt6.QtGui import (QIcon,
    QIntValidator
)
from PyQt6.QtWidgets import (
    QApplication,
    QPushButton,
    QVBoxLayout,
    QFormLayout,
    QLabel,
    QLineEdit,
    QWidget,
)

DATA_WINDOW_SIZE = 10

class Window(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        # Input line that sets window size
        self.windowLine = QLineEdit(text = str(DATA_WINDOW_SIZE), parent=self)
        self.windowLine.setValidator(QIntValidator())
        self.windowLine.textEdited.connect(self.update_window_size)
        
        # Button that requests the data
        self.requestBtn = QPushButton(text="Solicitar ventana de datos", parent=self)
        self.requestBtn.setFixedSize(200, 120)
        self.requestBtn.clicked.connect(self.request)
        
        # Button that closes connection
        self.closeBtn = QPushButton(text="Cerrar conexión", parent=self)
        self.closeBtn.setFixedSize(200, 120)
        self.closeBtn.clicked.connect(self.end)

        # Create layout of type VBox
        layout = QVBoxLayout()
        # Add widgets
        layout.addWidget(self.windowLine)
        layout.addWidget(self.requestBtn)
        layout.addWidget(self.closeBtn)
        #Set layout
        self.setLayout(layout)

    def request(self):
        data_list = receiver.recieve_window_data(DATA_WINDOW_SIZE)
        print(data_list)
        # Graficar los datos
    
    def end(self):
        receiver.restart_ESP()
        self.close()
    
    def update_window_size(self):
        DATA_WINDOW_SIZE = self.windowLine.text()
        receiver.set_window_size(DATA_WINDOW_SIZE)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec())