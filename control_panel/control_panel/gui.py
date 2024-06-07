import signal
import sys
import threading

import rclpy

from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QLabel
from control_panel.publisher import Panel_Publisher

RANGE = 10000
LINE_EDIT_WIDTH = 45

# Defined by style - currently using the default style
DEFAULT_WINDOW_MARGIN = 11
DEFAULT_CHILD_MARGIN = 9
DEFAULT_BTN_HEIGHT = 25

# Calculate default minimums for window sizing
MIN_WIDTH = DEFAULT_CHILD_MARGIN * 4 + DEFAULT_WINDOW_MARGIN * 2
MIN_HEIGHT = (
    DEFAULT_BTN_HEIGHT * 2 + DEFAULT_WINDOW_MARGIN * 2 + DEFAULT_CHILD_MARGIN * 2
)


class ControlPanelGui(QMainWindow):
    def __init__(self, title, publisher: Panel_Publisher):
        super(ControlPanelGui, self).__init__()

        self.setWindowTitle(title)
        
        self.title2=QLabel(self)
        self.title2.setText("GAME STATUS")
        self.title2.setStyleSheet("font-size: 15px;") # color: red;

        self.text4=QLabel(self)
        self.text4.setText("game_progess")
        self.game_progess_text = QLabel(self)
        self.game_progess_text.setText("GAME_UNSTARTED")
        self.game_progess_text.setStyleSheet("color: blue;") # color: red;
        self.game_progess = QLineEdit(self)
        self.game_progess.setText("0")

        self.text5=QLabel(self)
        self.text5.setText("stage_remaining_time")
        self.timer_text = QLabel(self)
        self.timer_text.setText("0")
        self.timer_text.setStyleSheet("color: red;") # color: red;
        self.stage_remaining_time = QLineEdit(self)
        self.stage_remaining_time.setText("32767")

        self.text12=QLabel(self)
        self.text12.setText("current_hp")
        self.current_hp = QLineEdit(self)
        self.current_hp.setText("600")
        self.PubGameStatusButton=QPushButton("set game status", self)
        self.PubGameStatusButton.clicked.connect(self.PubGameStatusEvent)

        self.text6=QLabel(self)
        self.text6.setText("bullet_remaining_num_17mm")
        self.bullet_remaining_num_17mm = QLineEdit(self)
        self.bullet_remaining_num_17mm.setText("400")
        
        self.text7=QLabel(self)
        self.text7.setText("my_outpost_hp")
        self.my_outpost_hp=QLineEdit(self)
        self.my_outpost_hp.setText("1000")

        self.text8=QLabel(self)
        self.text8.setText("enemy_outpost_hp")
        self.enemy_outpost_hp=QLineEdit(self)
        self.enemy_outpost_hp.setText("1000")

        self.text9=QLabel(self)
        self.text9.setText("my_base_hp")
        self.my_base_hp=QLineEdit(self)
        self.my_base_hp.setText("1000")

        self.text10=QLabel(self)
        self.text10.setText("enemy_base_hp")
        self.enemy_base_hp=QLineEdit(self)
        self.enemy_base_hp.setText("1000")

        # Main layout
        self.main_layout = QVBoxLayout()
        
        self.main_layout.addWidget(self.title2)
        self.main_layout.addWidget(self.text4)
        self.main_layout.addWidget(self.game_progess_text)
        self.main_layout.addWidget(self.game_progess)
        self.main_layout.addWidget(self.text5)
        self.main_layout.addWidget(self.timer_text)
        self.main_layout.addWidget(self.stage_remaining_time)
        self.main_layout.addWidget(self.text12)
        self.main_layout.addWidget(self.current_hp)
        self.main_layout.addWidget(self.text6)
        self.main_layout.addWidget(self.bullet_remaining_num_17mm)
        self.main_layout.addWidget(self.text7)
        self.main_layout.addWidget(self.my_outpost_hp)
        self.main_layout.addWidget(self.text8)
        self.main_layout.addWidget(self.enemy_outpost_hp)
        self.main_layout.addWidget(self.text9)
        self.main_layout.addWidget(self.my_base_hp)
        self.main_layout.addWidget(self.text10)
        self.main_layout.addWidget(self.enemy_base_hp)

        self.main_layout.addWidget(self.PubGameStatusButton)

        # central widget
        self.central_widget = QWidget()
        self.central_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.central_widget)
        self.publisher = publisher
        self.running = True

        
    def PubGameStatusEvent(self,event):
        self.publisher.get_logger().info("Pub Game Status!")
        self.timer_text.setText(self.stage_remaining_time.text())
        self.publisher.set_game_state(
            int(self.game_progess.text()),
            #uint16 stage_remain_time
            int(self.stage_remaining_time.text()),
            #uint16 current_hp
            int(self.current_hp.text()),
            #uint16 projectile_allowance_17mm
            int(self.bullet_remaining_num_17mm.text()),
            #uint16 my_outpost_hp
            int(self.my_outpost_hp.text()),
            #uint16 enemy_outpost_hp
            int(self.enemy_outpost_hp.text()),
            #uint16 my_base_hp
            int(self.my_base_hp.text()),
            #uint16 enemy_base_hp
            int(self.enemy_base_hp.text())
        )

    def loop(self):
        while self.running:
            rclpy.spin_once(self.publisher, timeout_sec=0.01)
            self.timer_text.setText(self.publisher.game_state.stage_remain_time.__str__())
            if self.publisher.game_state.game_progress==0:
                self.game_progess_text.setText("GAME_UNSTARTED")
            elif self.publisher.game_state.game_progress==1:
                self.game_progess_text.setText("GAME_READY")
            elif self.publisher.game_state.game_progress==2:
                self.game_progess_text.setText("GAME_INITIAL")
            elif self.publisher.game_state.game_progress==3:
                self.game_progess_text.setText("GAME_START_COUNTDOWN")
            elif self.publisher.game_state.game_progress==4:
                self.game_progess_text.setText("GAME_RUNNING")
            elif self.publisher.game_state.game_progress==5:
                self.game_progess_text.setText("GAME_STOP")
                
            self.publisher.game_state_publisher_.publish(self.publisher.game_state)

def main():
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    app = QApplication(sys.argv)
    panel_pub = Panel_Publisher()
    pannel = ControlPanelGui("Control Pannel", panel_pub)

    pannel.show()

    threading.Thread(target=pannel.loop).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
