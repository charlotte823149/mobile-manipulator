#!/usr/bin/env python3
import os
import sys
import rospy
from std_msgs.msg import String, Float32
from functools import partial
from PyQt5.QtGui import QIcon, QFont, QPixmap, QTextCharFormat, QTextCursor, QColor
from PyQt5.QtCore import Qt, QTimer, QSize, QMetaObject, Q_ARG
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout, QFrame, QLabel, QWidget, QTextEdit, QSizePolicy

os.environ['ROS_MASTER_URI'] = 'http://192.168.1.148:11311'

class ToolUrgentApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.estimated_times = {"A": "", "B": ""}
        self.tool_statuses = {"A": "", "B": ""}


        self.init_ui()  
        self.init_ros()

        self.button_disabled = False
        self.tool_name = ""  # Define the tool_name attribute
        self.urgent_pressed = False
        self.normal_pressed = False
    
        # Define dictionaries to store estimated times and tool statuses for frames A and B
        
    

    def init_ros(self):
        rospy.init_node('Urgent_tool')  # Initialize ROS node
        self.publisher_pub = rospy.Publisher('/gui_node/tool_task', String, queue_size=1)  # Create ROS publisher
        self.list_sub = rospy.Subscriber('/main_node/task_list', String, self.task_list_shown) # Create ROS Subscriber for task list
        self.tool_status_sub = rospy.Subscriber('/main_node/status', String, self.update_tool_status)  # Create ROS Subscriber for Tool Status
        self.time_sub = rospy.Subscriber('/main_node/estimated_time', String, self.estimated_time) # Create a ROS Subscriber for Estimated Time

  

    def init_ui(self):
        self.setWindowTitle('Mobile Manipulator')
        self.setGeometry(100, 100, 1280, 720)

        central_widget = QWidget(self)
        layout = QHBoxLayout()  # Main layout for frames A, spacer, frame for text edit, and frame B

        frame_a = self.create_frame("A", "A_")
        frame_a.setMaximumWidth(600)

        # spacer_frame = QFrame()  # Blank frame as spacer
        # spacer_frame.setFrameShape(QFrame.StyledPanel)
        # spacer_frame.setMinimumWidth(200)

        text_edit_frame = QFrame()  # Frame for the text edit
        text_edit_frame.setFrameShape(QFrame.StyledPanel)
        text_edit_frame.setMaximumWidth(450)
        text_edit_frame.setMaximumHeight(800)
        text_edit_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        text_edit_layout = QVBoxLayout()

           # Create a label for the image
        image_label = QLabel()
        pixmap = QPixmap('/home/usc-cam2/Shantanu_ws/Human_handoff/src/gui2/scripts/MM-removebg-preview(1).png')  # Replace with the actual image path
        image_label.setPixmap(pixmap)
        image_label.setAlignment(Qt.AlignCenter)  # Center the image in the label
        text_edit_layout.addWidget(image_label)

        # Add a spacer to create vertical space
        text_edit_layout.addSpacing(40)  # Adjust the spacing value as needed


         # Create a label for the tool status
        self.tool_status_label = QLabel("Tool Status:")
        self.tool_status_label.setFont(QFont("Arial", 20))  # Set font size here
        self.tool_status_label.setStyleSheet("color: Black;")  # Set font color here
        text_edit_layout.addWidget(self.tool_status_label)

         # Add a spacer to create vertical space
        text_edit_layout.addSpacing(20)  # Adjust the spacing value as needed

        self.estimated_time_label = QLabel("Estimated Time:")
        self.estimated_time_label.setFont(QFont("Arial", 20))  # Set font size here
        self.estimated_time_label.setStyleSheet("color: Black;")  # Set font color here
        text_edit_layout.addWidget(self.estimated_time_label)

        # Add a spacer to create vertical space
        text_edit_layout.addSpacing(30)  # Adjust the spacing value as needed

        text_edit = QTextEdit()
        text_edit.setReadOnly(True)
        text_edit_layout.addWidget(text_edit)


        # Increase the font size for the text displayed in the QTextEdit
        text_format = QTextCharFormat()
        text_format.setFontPointSize(18)  # Adjust the font size as needed
        cursor = text_edit.textCursor()
        cursor.select(QTextCursor.Document)
        cursor.mergeCharFormat(text_format)
        text_edit.setCurrentCharFormat(text_format)


        text_edit_frame.setLayout(text_edit_layout)

        frame_b = self.create_frame("B", "B_")
        frame_b.setMaximumWidth(600)

        layout.addWidget(frame_a)
        # layout.addWidget(spacer_frame)
        layout.addWidget(text_edit_frame)
        layout.addWidget(frame_b)

        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
        self.text_edit = text_edit



    def task_list_shown(self, msg):
        QMetaObject.invokeMethod(self.text_edit, "clear", Qt.QueuedConnection)

        task_list = msg.data.split(",")
        for task in task_list:
            QMetaObject.invokeMethod(self.text_edit, "append", Qt.QueuedConnection, Q_ARG(str, task))


   

    def create_frame(self, frame_title, prefix):
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        frame_layout = QHBoxLayout()  # Use QHBoxLayout for the main frame layout
        
        title_label = QLabel(frame_title)
        title_label.setFixedWidth(100)
        title_label.setFixedHeight(200)
        title_label.setAlignment(Qt.AlignTop | Qt.AlignCenter)
        

         # Increase the font size of the title label
        title_font = QFont("Arial", 24)  # Adjust the font size as needed
        title_label.setFont(title_font)
        frame_layout.addWidget(title_label)

        # Create a subframe for tool buttons
        tool_buttons_frame = QFrame()
        tool_buttons_layout = QVBoxLayout()  # Create a QVBoxLayout for tool buttons
        buttons = ["Hammer", "Spanner", "Pliers", "Allen", "Suspension"]
        for button_text in buttons:
            button = QPushButton(button_text)
            button.setMinimumHeight(100)
            button.setMinimumWidth(120)
            button.setFont(QFont("Times New Roman", 22))  # Set font size here
            
            button.clicked.connect(lambda _, btn=button, p=prefix, b=button_text: self.tool_button_click(btn, p + b))
            tool_buttons_layout.addWidget(button)  # Add tool buttons to tool_buttons_layout
        tool_buttons_frame.setLayout(tool_buttons_layout)  # Set tool_buttons_layout for tool_buttons_frame
        frame_layout.addWidget(tool_buttons_frame)  # Add tool_buttons_frame to frame_layout

            
       
        # Create a subframe for the urgent and normal buttons
        buttons_subframe = QFrame()
        buttons_subframe_layout = QVBoxLayout()  # Use QVBoxLayout for the buttons subframe


        # Add a spacer above the buttons to push them downward
        spacer = QWidget()
        spacer.setFixedHeight(150)  # Adjust the height as needed
        buttons_subframe_layout.addWidget(spacer)


        urgent_button = QPushButton("")
        urgent_button.setIcon(QIcon('/home/usc-cam2/Shantanu_ws/Human_handoff/src/gui2/scripts/Untitled_1_-removebg-preview.png'))  # Replace with the actual image path
        urgent_button.setIconSize(QSize(250, 250))
        urgent_button.clicked.connect(lambda _, p=prefix, b="Urgent": self.urgent_button_click(p + b))
        buttons_subframe_layout.addWidget(urgent_button)  # Add the urgent button to buttons_subframe_layout

        normal_button = QPushButton("")
        normal_button.setIcon(QIcon('/home/usc-cam2/Shantanu_ws/Human_handoff/src/gui2/scripts/EM_1_-removebg-preview(3).png'))  # Replace with the actual image path
        normal_button.setIconSize(QSize(250, 250))
        normal_button.clicked.connect(lambda _, p=prefix, b="Normal": self.normal_button_click(p + b))
        buttons_subframe_layout.addWidget(normal_button)  # Add the normal button to buttons_subframe_layout

        # # Set the text alignment to center both horizontally and vertically
        # normal_button.setStyleSheet("text-align: center; padding: 0px; margin: 0px;")

        buttons_subframe_layout.addStretch(50)  # Add a stretch to push the buttons to the top
        buttons_subframe.setLayout(buttons_subframe_layout)  # Set buttons_subframe_layout for buttons_subframe
        frame_layout.addWidget(buttons_subframe)  # Add buttons_subframe to frame_layout
        
        
        frame.setLayout(frame_layout)
        return frame

    def tool_button_click(self, button, tool_name):
        if not self.button_disabled:
            self.tool_name = tool_name
            self.urgent_pressed = False
            self.normal_pressed = False
            self.button_disabled = True

            button.setStyleSheet("background-color: green;")
            QTimer.singleShot(1000, lambda: self.enable_buttons(button))

    def urgent_button_click(self, _):
        if self.button_disabled:
            self.urgent_pressed = True

    def normal_button_click(self, _):
        if self.button_disabled:
            self.normal_pressed = True


    def enable_buttons(self, button):
        self.button_disabled = False
        if self.urgent_pressed:
            u = f"{self.tool_name}_Urgent"
            print(u)

        elif self.normal_pressed:
            u = f"{self.tool_name}_Regular"
            print(u)
        else:
            u = ""
            print(u)
         
        # Publish the direction information to ROS topic
        rate = rospy.Rate(10)
        publisher_msg = String()
        publisher_msg.data = u
        self.publisher_pub.publish(publisher_msg)    
        
        button.setStyleSheet("")

    def update_tool_status(self, msg):
        # Use QMetaObject.invokeMethod to update the GUI from a non-GUI thread
        QMetaObject.invokeMethod(self.tool_status_label, "setText", Qt.QueuedConnection, Q_ARG(str, f"Tool Status: {msg.data}"))

    def estimated_time(self, msg):
        QMetaObject.invokeMethod(self.estimated_time_label, "setText", Qt.QueuedConnection, Q_ARG(str, f"Estimated Time: {msg.data}"))

    

if __name__ == '__main__':  
    app = QApplication(sys.argv)
    tool_urgent_app = ToolUrgentApp()
    tool_urgent_app.show()
    sys.exit(app.exec_())
