import sys
import glob 
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt6.QtCore import QTimer

from battery_ui import Ui_MainWindow
from pzem017 import PZEM017

def find_available_usb_ports():
    """Trả về danh sách tất cả các cổng /dev/ttyUSB*"""
    return glob.glob("/dev/ttyUSB*")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Setup UI
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("PZEM017 Battery Monitor")

        # Init PZEM
        self.pzem = None
        ports_to_try = ["/dev/ttyUSB1"] + [p for p in find_available_usb_ports() if p != "/dev/ttyUSB1"]
        for port in ports_to_try:
            try:
                self.pzem = PZEM017(port=port, addr=0x01)
                print(f"Connected to {port}")
                break
            except Exception as e:
                print(f"Failed to connect to {port}: {e}")

        if self.pzem is None:
            QMessageBox.critical(self, "Serial Error", "Could not connect to any USB port")
            sys.exit(1)

        # Timer update (1s)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_values)
        self.timer.start(1000)

        # Button reset
        self.ui.btnReset.clicked.connect(self.reset_energy)

    def update_values(self):
        try:
            data = self.pzem.read_values()

            self.ui.voltage.setText(f"{data['voltage']:.2f}")
            self.ui.ampe.setText(f"{data['current']:.2f}")
            self.ui.power.setText(f"{data['power']:.1f}")
            self.ui.energy.setText(f"{data['energy']:.3f}")

            self.statusBar().showMessage("Connected")

        except Exception as e:
            self.statusBar().showMessage(f"Error: {e}")

    def reset_energy(self):
        try:
            self.pzem.reset_energy()
            QMessageBox.information(self, "Reset", "Energy reset OK")
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))

    def closeEvent(self, event):
        self.pzem.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
