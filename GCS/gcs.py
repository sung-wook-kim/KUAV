from PyQt5 import QtWidgets,QtCore,QtGui
import sys, time
from PyQt5.QtCore import Qt,QUrl ,QTimer
from PyQt5 import QtWebEngineWidgets
from PyQt5 import QtWebEngineCore
from PyQt5.QtWebEngineWidgets import QWebEngineSettings
import folium
import io
import socket
import pandas as pd 
import serial
import threading
class window(QtWidgets.QMainWindow):
    def __init__(self):
        QWebEngineSettings.globalSettings().setAttribute(QWebEngineSettings.PluginsEnabled,True)
        super(window,self).__init__()
        self.centralwid=QtWidgets.QWidget(self)
        self.vlayout_main=QtWidgets.QVBoxLayout()

        self.btnSerial = QtWidgets.QPushButton('connect : STM32',self) ; self.btnSocket = QtWidgets.QPushButton('connect : NX',self)
        self.btnAVOID = QtWidgets.QPushButton('assign avoid ',self)
        self.btnTakeoff = QtWidgets.QPushButton('takeoff',self) ; self.btnRTH = QtWidgets.QPushButton('return to home',self)
        self.btnFT = QtWidgets.QPushButton('force termination : motor stop',self)
        self.btnDataSave = QtWidgets.QPushButton('data save',self)
        
        # streaming
        self.webview=QtWebEngineWidgets.QWebEngineView()
        self.webview.setUrl(QUrl("http://223.171.80.232:5001"))
        # self.webview.setUrl(QUrl("http://127.0.0.1"))

        # plot
        self.m = folium.Map(location=[37.5872530,127.0307692], tiles='cartodbpositron',zoom_start=13)
        self.data = io.BytesIO()
        self.m.save(self.data, close_file=False)

        self.statuslayout = QtWidgets.QHBoxLayout()
        self.lat = QtWidgets.QLabel("위도 : ",self)
        self.lon = QtWidgets.QLabel("경도 : ",self)
        self.altstatus = QtWidgets.QLabel("고도 : ",self)
        self.mission_lat = QtWidgets.QLineEdit("Mission lat",self)
        self.mission_lon = QtWidgets.QLineEdit("Mission lon",self)
        self.AVOID_lat = QtWidgets.QLineEdit("AVOID lat",self)
        self.AVOID_lon = QtWidgets.QLineEdit("AVOID lon",self)
        self.mission_lat.returnPressed.connect(self.missionlatFunction)
        self.mission_lon.returnPressed.connect(self.missionlonFunction)
        self.AVOID_lat.returnPressed.connect(self.AVOIDlatFunction)
        self.AVOID_lon.returnPressed.connect(self.AVOIDlonFunction)
        self.statuslayout.addWidget(self.lat)
        self.statuslayout.addWidget(self.lon)
        self.statuslayout.addWidget(self.altstatus)
        self.statuslayout.addWidget(self.mission_lat)
        self.statuslayout.addWidget(self.mission_lon)
        self.statuslayout.addWidget(self.AVOID_lat)
        self.statuslayout.addWidget(self.AVOID_lon)

        self.webview2=QtWebEngineWidgets.QWebEngineView()
        self.webview2.setHtml(self.data.getvalue().decode())
        self.weblayout = self.btnlayout=QtWidgets.QHBoxLayout()
        self.weblayout.addWidget(self.webview)
        self.weblayout.addWidget(self.webview2)
        self.btnlayout=QtWidgets.QHBoxLayout()
        self.btnlayout.addWidget(self.btnSerial) ; self.btnlayout.addWidget(self.btnSocket) ; self.btnlayout.addWidget(self.btnAVOID)
        self.btnlayout.addWidget(self.btnTakeoff) ; self.btnlayout.addWidget(self.btnRTH)
        self.btnlayout.addWidget(self.btnFT)      ; self.btnlayout.addWidget(self.btnDataSave)
        self.vlayout_main.addLayout(self.statuslayout)
        self.vlayout_main.addLayout(self.weblayout)
        self.vlayout_main.addLayout(self.btnlayout)
        self.centralwid.setLayout(self.vlayout_main)
        self.setCentralWidget(self.centralwid)
        self.show()
        self.timer = QTimer(self)
        self.timer.start(1000)
        self.rad = 1
        self.HOST = '223.171.80.232'
        #self.HOST = '192.168.43.185'
        self.port = 9998
        self.lat_drone_li = [] ;self.lon_drone_li = [] ; self.GPStime = [] ; self.lat_person = [] ;self.lon_person = [] ;self.altitude = []
        self.server_socket = 0 ; self.client_socket = 0 ; self.addr = 0;
        self.NX_data = b'0'
        self.btnSerial.clicked.connect(self.connectSerial) ; self.btnSocket.clicked.connect(self.connectSocket)
        self.btnAVOID.clicked.connect(self.avoid) ; self.btnTakeoff.clicked.connect(self.takeoff) ; self.btnRTH.clicked.connect(self.RTH)
        self.btnDataSave.clicked.connect(self.datasave) ; self.btnFT.clicked.connect(self.forceTerminate)
        self.serBase = 0
        self.serTele = 0
        self.threadRTK = threading.Thread(target=self.RTK)
        self.header_1 = b'0x44'
        self.header_2 = b'0x77'

        self.MISSION_LAT = 0
        self.MISSION_LON = 0
        self.RTH_LAT = 0
        self.RTH_LON = 0
        self.lat_drone = 0
        self.lon_drone = 0
        self.automatic = 0
        self.mode_li = []
        self.automatic_li = []
        self.GPStime = []

    def missionlatFunction(self):
        self.MISSION_LAT = int(float(self.mission_lat.text())*10**7)
        print(self.MISSION_LAT)

    def missionlonFunction(self):
        self.MISSION_LON = int(float(self.mission_lon.text())*10**7)
        print(self.MISSION_LON)
        
    def AVOIDlatFunction(self):
        self.AVOID_LAT = int(float(self.AVOID_lat.text())*10**7)
        print(self.AVOID_LAT)

    def AVOIDlonFunction(self):
        self.AVOID_LON = int(float(self.AVOID_lon.text())*10**7)
        print(self.AVOID_LON)

    # GCS - STM32 serial ( telemetry maybe)
    def connectSerial(self):
        self.serBase = serial.Serial('COM6', 115200, timeout=1)
        self.serTele =  serial.Serial('COM7', 115200, timeout=1)
        self.serBase.flush()
        self.serTele.flush()
        self.threadRTK.start()
        self.btnSerial.setText("connect FC with Serial ")

    def RTK(self):
        while True:
            self.serTele.write(self.serBase.read(1)) # 읽어온값을 그대로 전송
    # GCS - NX Socket ( wifi )
    def connectSocket(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.HOST, self.port))
        self.timer.timeout.connect(self.update_gps)
        self.btnSocket.setText("connect FCC with Socket ")

    # Socket을 통해 NX에서 GPS 데이터 받아온 후 저장 및 지도에 그리기    
    def update_gps(self):
        self.client_socket.sendall(self.NX_data)
        self.NX_data = self.client_socket.recv(1024)
        print(self.NX_data) # 값 들어오는거 확인용 
        mode , self.lat_drone , self.lon_drone , gps_time , lat_person , lon_person , altitude  = self.NX_data.decode().split('\n')
        print(self.rad) # 역시 확인용
        self.automatic_li.append(self.automatic)
        self.mode_li.append(mode)  
        self.lat_drone_li.append(self.lat_drone) ; self.lon_drone_li.append(self.lon_drone) ; self.GPStime.append(gps_time)
        self.lat_person.append(lat_person) ; self.lon_person.append(lon_person) 
        self.altitude.append(altitude) # 비행 데이터 제출용

        # 마킹코드
        if self.rad % 3 == 0 :
            self.m = folium.Map(location=[37.6977308,127.6068025],  tiles='cartodbpositron', zoom_start=13)
            folium.CircleMarker(color = 'red' , fill_color = 'red' ,location=[float(self.lat_drone),float(self.lon_drone)],radius=5 , popup="célula",).add_to(self.m)
            folium.CircleMarker(color = 'blue' , fill_color = 'blue' ,location=[float(lat_person),float(lon_person)],radius=5 , popup="célula",).add_to(self.m)
            self.data = io.BytesIO()
            self.m.save(self.data, close_file=False)
            self.webview2.setHtml(self.data.getvalue().decode())

        self.rad +=1
        self.lat.setText(f"위도 : {float(self.lat_drone)  }")
        self.lon.setText(f"경도 : {float(self.lon_drone)  }")
        self.altstatus.setText(f"고도 : {altitude}m")
    # 이륙 명령 -> NX한테 이륙 명령 코드를 보낸다.
    def takeoff(self):
        msgT = f"1\n{self.MISSION_LAT}\n{self.MISSION_LON}"
        self.client_socket.sendall(msgT.encode())
        _ = self.client_socket.recv(512) # don't need this
        self.automatic = 1
    
    # 리턴투홈
    def RTH(self):
        msgR = f"6\n{self.RTH_LAT}\n{self.RTH_LON}"
        self.client_socket.sendall(msgR.encode())
        _ = self.client_socket.recv(512)
    
    def avoid(self):
        msgA = f"7\n{self.AVOID_LAT}\n{self.AVOID_LON}"
        self.client_socket.sendall(msgA.encode())
        _ = self.client_socket.recv(512)    
    # 강제종료
    def forceTerminate(self):
        self.client_socket.sendall(b"9\n")
        _ = self.client_socket.recv(512)
    # 데이터 저장 
    # 비행 데이터  : (자동 : 0 , 수동 : 1) / mode / Gps time / lat / lon / alt
    def datasave(self):
        df = pd.DataFrame()
        # 자릿수 자르기
        for i in range(0,len(self.lat_drone_li)):
            if len(self.lat_drone_li[i]) == 10:
                self.lat_drone_li[i] = self.lat_drone_li[i][:-1] 
        for j in range(0,len(self.lon_drone_li)):
            if len(self.lon_drone_li[j]) == 11:
                self.lon_drone_li[j] = self.lon_drone_li[j][:-1]

        df['automatic'] = self.automatic_li ; df['point'] = self.mode_li 
        df['GPS Time'] = self.GPStime
        df['lat_drone'] = self.lat_drone_li ; df['lon_drone'] = self.lon_drone_li 
        df['altitude'] = self.altitude
        now = time.localtime()
        timevar = time.strftime('%Y%m%d%H%M%S', now)
        df.to_csv(f"{timevar}_Flight_data.csv")
        print("data saved")

app=QtWidgets.QApplication([])
ex=window()
sys.exit(app.exec_())

