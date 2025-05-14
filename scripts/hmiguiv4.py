from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel,
    QComboBox, QPushButton, QWidget, QStackedLayout, QGraphicsDropShadowEffect, QMessageBox
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap, QFont, QPalette, QColor
# from PyQt5.QtWebEngineWidgets import QWebEngineView
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8

import pandas as pd


class HMIApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # Set the main window properties
        self.setWindowTitle("Autonomous Car HMI")
        self.setGeometry(100, 100, 800, 600)

        # Set background gradient
        palette = QPalette()
        gradient = QColor(235, 235, 250)  # Light lavender
        palette.setColor(QPalette.Background, gradient)
        self.setPalette(palette)

        # Central Widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.layout = QStackedLayout()
        central_widget.setLayout(self.layout)

        # Add Startup Animation Screen
        self.startup_screen = self.create_startup_screen()
        self.layout.addWidget(self.startup_screen)

        # Add HMI Interface
        self.hmi_interface = self.create_hmi_interface()
        self.layout.addWidget(self.hmi_interface)

        # Timer for the startup animation (5 seconds)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.show_hmi_interface)
        self.timer.start(5000)  # 2.5 seconds

    def create_startup_screen(self):
        """Create the startup animation screen."""
        startup_widget = QWidget()
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)

        # Logo Image
        logo_label = QLabel()
        pixmap = QPixmap("/home/autodrive/GP_test/ADC2Y4/src/a3_astar/misc/hmi_logo.webp") ### change with appropriate directory
        logo_label.setPixmap(pixmap.scaled(800, 800, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        logo_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(logo_label)

        # Title Text
        title_label = QLabel("Aggies' Autonomous Auto")
        title_label.setFont(QFont("Poppins", 24, QFont.Bold))
        title_label.setStyleSheet("color: #fdb927; text-align: center;")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        startup_widget.setLayout(layout)
        return startup_widget

    def create_hmi_interface(self):
        """Create the HMI interface."""
        hmi_widget = QWidget()
        hmi_layout = QVBoxLayout()
        hmi_layout.setAlignment(Qt.AlignTop)
        hmi_layout.setContentsMargins(20, 20, 20, 20)

        


        # Autonomous Mode Toggle Button
        toggle_layout = QVBoxLayout()
        self.toggle_button = QPushButton("Enable Autonomous Mode")
        self.toggle_button.setFixedHeight(50)
        self.toggle_button.setStyleSheet("""
            QPushButton {
                background-color: #fdb927;
                color: black;
                padding: 10px 20px;
                border-radius: 10px;
                font-size: 16px;
                border: none;
            }
            QPushButton:hover {
                background-color: #ffcb42;
            }
            QPushButton:pressed {
                background-color: #ffa500;
            }
        """)
        self.toggle_button.clicked.connect(self.toggle_autonomous_mode)
        toggle_layout.addWidget(self.toggle_button, alignment=Qt.AlignCenter)
        hmi_layout.addLayout(toggle_layout)

        # Start button for Autonomous mode
        self.start_button = QPushButton("Start")
        self.start_button.setFixedHeight(50)
        self.start_button.setStyleSheet("""
                                        background-color: #4CAF50;
                                        color: white;
                                        padding: 10px 20px
                                        border-radius: 10px;
                                        font-size: 16px
                                        """)
        
        self.start_button.clicked.connect(self.enable_autonomous_mode)
        toggle_layout.addWidget(self.start_button, alignment=Qt.AlignCenter)
        
        self.autonomous_enabled = False

        # left layout (Buttons)
        self.button_layout = QVBoxLayout()
        self.create_buttons()

        # Right layout (Information Display)
        self.info_layout = QVBoxLayout()
        self.info_label = QLabel("Select a Destination", self)
        self.info_label.setStyleSheet("font-size: 16px; color: #000080;")
        self.info_layout.addWidget(self.info_label)

        # Add both Layouts to the HMI layout
        hmi_layout.addLayout(self.button_layout)
        hmi_layout.addLayout(self.info_layout)

        #self.setLayout(hmi_layout)

        # Initiate ROS node for GPS coordinates
        rospy.init_node("autonomy_gps_publisher", anonymous=True)
        self.gps_pub = rospy.Publisher("/gps_coordinates", NavSatFix, queue_size = 10)

        # Initiate ROS node for Autonomy
        self.autonomy_pub = rospy.Publisher("/autonomyengage", UInt8, queue_size=10)

        self.autonomy_active = False

        self.data = self.load_data("/home/autodrive/GP_test/ADC2Y4/src/a3_astar/data/just_A_DYOC.xlsx") ### change with appropriate directory

        self.show()

        # Assemble all sections
        hmi_widget.setLayout(hmi_layout)

        return hmi_widget 

    def create_buttons(self):
        self.buttons = {}
        button_labels = [chr(i) for i in range(65, 91)]
        button_index = 0

        for i in range(7):
            row_layout = QHBoxLayout()
            for j in range(4):
                if button_index < 26:
                    label = button_labels[button_index]
                    button = QPushButton(f"Location {label}")
                    button.setFixedHeight(50)                
                    self.buttons[button] = button.text()
                    button.setStyleSheet("""
                                         QPushButton {
                                         background-color: #fdb927;
                                         color: #000080;
                                         padding: 10px 20 px;
                                         border-radius: 10px;
                                         font-size: 16px;
                                         border: none;
                                         }
                                         QPushButton: hover {
                                         background-color: #ffcb42;
                                         }
                                         QPushButton:pressed {
                                         background-color: #ffa500
                                         }
                                         """)
                    button.clicked.connect(lambda checked, b=button, lbl=label: self.on_button_click(b, lbl))
                    row_layout.addWidget(button)
                    button_index += 1
                self.button_layout.addLayout(row_layout)


    def load_data(self, filename):
        """Loads data from csv file and returns a dictionary where row index is the key"""
        try:
            df = pd.read_excel(filename, engine="openpyxl")
            data_dict = df.to_dict(orient="index")
            # print("Loaded Data Dictionary:", data_dict)
            return data_dict
        except Exception as e:
            print(f"Error loading file: {e}")
            return {}
    
    def on_button_click(self, clicked_button, button_label):
        """Handle the toggle button to grap the destination longitude and Latitude Informations."""
        button_index = ord(button_label) - 65
        if button_index in self.data:
            
            row_data = self.data[button_index]
            #print(f"Location {button_label} Data:", row_data)

            # store selected data
            Longitude = row_data.get("Lon", "Unknown")
            #print(f"Location {button_number} Longitude is:", Longitude)
            
            Latitude = row_data.get("Lat", "N/A")
            #print(f"Location {button_number} Latitude is:", Latitude)
            
            Description = row_data.get("Description", "N/A")

            # store in global variable
            self.Location_Long = Longitude
            self.Location_Lat = Latitude

            # Update Qlabel in the infor layout
            for button, original_text in self.buttons.items():
                button.setText(original_text)

            clicked_button.setText(f"Destination is: {Description}")
            
            self.info_label.setText(f"Selected Destination: {Description}\nLatitude: {Latitude}\nLongitude: {Longitude}")
            
            # Publish the longitude and latitude of the selected location to the route planning algorithm
            gps_msg = NavSatFix()
            gps_msg.latitude = Latitude
            gps_msg.longitude = Longitude

            rospy.loginfo(f"Publishing GPS Coordinates: Lat={Latitude}, Lon={Longitude}")
            self.gps_pub.publish(gps_msg)
            
            
        else:
            QMessageBox.warning(self, "Error", f"No data found for Location {button_label}")
            self.info_label.setText(f"No data found for Location: {button_label}\nLatitude: N/A\nLongitude: N/A")


    def show_hmi_interface(self):
        """Show the HMI interface after the startup animation ends."""
        self.timer.stop()
        self.layout.setCurrentWidget(self.hmi_interface)

    def enable_autonomous_mode(self):
        """Enable the autonomous mode toggle functionality."""
        if self.start_button.text() == "Start":
            self.start_button.setText("End")
            self.autonomous_enabled = True
        
        else:
            self.start_button.setText("Start")
            self.autonomous_enabled = False
            self.publish_autonomy_status(0)

    def toggle_autonomous_mode(self):
        """Handle the toggle button to enable/disable autonomous mode."""
        if not self.autonomous_enabled:
            return
        
        if self.toggle_button.text() == "Enable Autonomous Mode":
            self.toggle_button.setText("Disable Autonomous Mode")
            self.toggle_button.setStyleSheet("""
                background-color: #ff4d4d;
                color: black;
                padding: 10px 20px;
                border-radius: 10px;
                font-size: 16px;
            """)
            self.autonomy_active = True
        else:
            self.toggle_button.setText("Enable Autonomous Mode")
            self.toggle_button.setStyleSheet("""
                background-color: #fdb927;
                color: black;
                padding: 10px 20px;
                border-radius: 10px;
                font-size: 16px;
            """)
            self.autonomy_active = False
        self.publish_autonomy_status(1 if self.autonomy_active else 0)
    
    def publish_autonomy_status(self, status):
        """Publish the autonomy engagement message to ROS"""
        msg = UInt8()
        msg.data = status
        self.autonomy_pub.publish(msg)
        rospy.loginfo(f"Publishing: {msg.data}")
        


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = HMIApp()
    window.show()
    sys.exit(app.exec_())