import sys

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
        # Set window size input line
        int_line_edit = QLineEdit(text = str(DATA_WINDOW_SIZE), parent=self)
        int_line_edit.setValidator(QIntValidator())
        # Button that requests the data
        requestBtn = QPushButton(text="Solicitar ventana de datos", parent=self)
        requestBtn.setFixedSize(200, 120)
        # Button that closes connection
        closeBtn = QPushButton(text="Cerrar conexión", parent=self)
        closeBtn.setFixedSize(200, 120)
        requestBtn.clicked.connect(self.request)
        closeBtn.clicked.connect(self.close)
        layout = QVBoxLayout()
        layout.addWidget(int_line_edit)
        layout.addWidget(requestBtn)
        layout.addWidget(closeBtn)
        self.setLayout(layout)
    def request():
        return
    def close():
        return

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec())