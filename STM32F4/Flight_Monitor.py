###############################################################
# DONGYANG E&P PV INVERTER(ESP3K5) MONITORING PROGRAM  v0.61  #
# MONITORING : Make output csv FILE & PYQT GRAPH              #
# COPYRIGHT BY RAISON - fantasy297.tistory.com                #
###############################################################

import os
import pyqtgraph
from pyqtgraph.Qt import QtGui, QtCore
from PyQt5.QtWidgets import QWidget, QApplication, QHBoxLayout, QVBoxLayout, QGroupBox, QLabel
from PyQt5.QtCore import pyqtSlot, QTimer, Qt
from pandas import DataFrame
import pandas
import serial
import time
import sys

class Main(QWidget):

    ############### 전역변수 설정 및 기존 데이터 취득 #####################################
    # 초기 입력값 설정
    Port = 'COM7'
    BaudRate = 115200
    bytesize = 8
    NetID = 7
    DataLength = 32  # 수신 데이터 개수
    Error_Count = 0
    Cycle_Time = 10 #per millisecond
    DataColumns = ['current Altitude', 'target Altitude', 'Altitude Error', 'Empty']
    print("Press Ctrl + C to Ending Monitoring")

    # 출력 파일 이름 설정
    tm = time.localtime(time.time())
    DF_Date = time.strftime('%m%d_%H%M%S', tm)
    OutFileName = "Drone_Monitoring_Data_%s" % DF_Date

    # 출력 폴더 생성
    try:
        if not (os.path.isdir('./Data')):
            os.makedirs(os.path.join('./Data'))
    except OSError as e:
        if e.errno != e.errno.EEXIST:
            print("Failed to create directory!!!!!")
            raise
    # 기존 파일 존재시 데이터 읽기
    if not os.path.exists('./Data/%s.csv' % OutFileName):
        DF_Result = DataFrame(columns=DataColumns)
    else:
        DF_Result = pandas.read_csv('./Data/%s.csv' % OutFileName, index_col=0)

    def __init__(self):
        super().__init__()

        # 통신 설정
        self.ser = serial.Serial(Main.Port, Main.BaudRate)

        # 레이아웃 생성
        hbox1 = QHBoxLayout()
        hbox2 = QHBoxLayout()
        hbox3 = QHBoxLayout()
        vbox1 = QVBoxLayout()
        Gbox0 = QVBoxLayout()
        Gbox1 = QVBoxLayout()
        Gbox2 = QVBoxLayout()
        Gbox3 = QVBoxLayout()
        Gbox4 = QVBoxLayout()
        Gbox5 = QVBoxLayout()
        Gbox6 = QVBoxLayout()
        Gbox7 = QVBoxLayout()
        Gbox8 = QVBoxLayout()
        Gbox9 = QVBoxLayout()

        # 기본 글꼴 설정
        font_tick = QtGui.QFont('Bahnschrift SemiLight', 8)
        font = QtGui.QFont('Bahnschrift SemiLight', 12)
        font.setBold(True)
        self.setFont(font)

        # 그래프 객체 4개 생성 및 X축을 STRING축으로 설정
        self.stringaxisP = pyqtgraph.AxisItem(orientation='bottom')
        self.stringaxisE = pyqtgraph.AxisItem(orientation='bottom')
        self.stringaxisV = pyqtgraph.AxisItem(orientation='bottom')
        self.stringaxisT = pyqtgraph.AxisItem(orientation='bottom')
        self.Power = pyqtgraph.PlotWidget(axisItems={'bottom': self.stringaxisP})
        self.Energy = pyqtgraph.PlotWidget(axisItems={'bottom': self.stringaxisE})
        self.Voltage = pyqtgraph.PlotWidget(axisItems={'bottom': self.stringaxisV})
        self.Temperature = pyqtgraph.PlotWidget(axisItems={'bottom': self.stringaxisT})

        # 그래프 제목 생성
        self.Power.setTitle(Main.DataColumns[0], color="#828282", size="12pt")
        self.Energy.setTitle(Main.DataColumns[1], color="#828282", size="12pt")
        self.Voltage.setTitle(Main.DataColumns[2], color="#828282", size="12pt")
        self.Temperature.setTitle(Main.DataColumns[3], color="#828282", size="12pt")

        # 그래프 제목 글꼴 설정
        self.Power.getPlotItem().titleLabel.item.setFont(font)
        self.Energy.getPlotItem().titleLabel.item.setFont(font)
        self.Voltage.getPlotItem().titleLabel.item.setFont(font)
        self.Temperature.getPlotItem().titleLabel.item.setFont(font)

        # X, Y축 이름 스타일 설정
        labelStyle = {'color': '#828282', 'font-size': '9pt' }

        # X, Y축 이름 생성
        self.Power.setLabel('left', 'Power', units='W', **labelStyle)
        self.Energy.setLabel('left', 'Energy', units='Wh', **labelStyle)
        self.Voltage.setLabel('left', 'Voltage', units='V', **labelStyle)
        self.Temperature.setLabel('left', 'Temperature', units='℃', **labelStyle)
        self.Power.setLabel('bottom', 'Time', **labelStyle)
        self.Energy.setLabel('bottom', 'Time', **labelStyle)
        self.Voltage.setLabel('bottom', 'Time', **labelStyle)
        self.Temperature.setLabel('bottom', 'Time', **labelStyle)

        # x,y 눈금 글꼴 설정
        self.Power.getAxis('bottom').setStyle(tickFont = font_tick, tickTextOffset=6)
        self.Energy.getAxis('bottom').setStyle(tickFont = font_tick, tickTextOffset=6)
        self.Voltage.getAxis('bottom').setStyle(tickFont = font_tick, tickTextOffset=6)
        self.Temperature.getAxis('bottom').setStyle(tickFont = font_tick, tickTextOffset=6)
        self.Power.getAxis('left').setStyle(tickFont = font_tick, tickTextOffset=6)
        self.Energy.getAxis('left').setStyle(tickFont = font_tick, tickTextOffset=6)
        self.Voltage.getAxis('left').setStyle(tickFont = font_tick, tickTextOffset=6)
        self.Temperature.getAxis('left').setStyle(tickFont = font_tick, tickTextOffset=6)

        # 그래프 그리드 설정
        self.Power.showGrid(x=True, y=True)
        self.Energy.showGrid(x=True, y=True)
        self.Voltage.showGrid(x=True, y=True)
        self.Temperature.showGrid(x=True, y=True)

        # 그래프 배경색 지정
        self.Power.setBackground((240,240,240))
        self.Energy.setBackground((240,240,240))
        self.Voltage.setBackground((240,240,240))
        self.Temperature.setBackground((240,240,240))

        # Data Indicator 그룹 박스 생성
        self.groupbox_SV = QGroupBox('Altitude_Error')
        self.groupbox_SC = QGroupBox(' ')
        self.groupbox_SP = QGroupBox('Currnet_Altitude')
        self.groupbox_LV = QGroupBox(' ')
        self.groupbox_LC = QGroupBox(' ')
        self.groupbox_LP = QGroupBox(' ')
        self.groupbox_T = QGroupBox(' ')
        self.groupbox_TTL = QGroupBox('Target_Altitude')
        self.groupbox_LIFE = QGroupBox(' ')
        self.groupbox_Status = QGroupBox(' ')

        # Data Indicator 라벨 생성
        self.label_SV = QLabel('0', self)
        self.label_SC = QLabel('0', self)
        self.label_SP = QLabel('0', self)
        self.label_LV = QLabel('0', self)
        self.label_LC = QLabel('0', self)
        self.label_LP = QLabel('0', self)
        self.label_T = QLabel('0', self)
        self.label_TTL = QLabel('0', self)
        self.label_LIFE = QLabel('0', self)
        self.label_Status = QLabel('Ready', self)

        # Data Indicator 가운데 정렬
        self.label_SV.setAlignment(Qt.AlignCenter)
        self.label_SC.setAlignment(Qt.AlignCenter)
        self.label_SP.setAlignment(Qt.AlignCenter)
        self.label_LV.setAlignment(Qt.AlignCenter)
        self.label_LC.setAlignment(Qt.AlignCenter)
        self.label_LP.setAlignment(Qt.AlignCenter)
        self.label_T.setAlignment(Qt.AlignCenter)
        self.label_TTL.setAlignment(Qt.AlignCenter)
        self.label_LIFE.setAlignment(Qt.AlignCenter)
        self.label_Status.setAlignment(Qt.AlignCenter)

        # Data Indicator 배경색 및 테두리 설정
        self.label_SV.setStyleSheet("color:rgb(203, 26, 126);" "background-color:rgb(250,250,250);"
                                    "border-style: solid;" "border-width: 1px;" "border-color: rgb(200,200,200);"
                                    "border-radius: 5px")
        self.label_SC.setStyleSheet("color:rgb(203, 26, 126);" "background-color:rgb(250,250,250);"
                                    "border-style: solid;" "border-width: 1px;" "border-color: rgb(200,200,200);"
                                    "border-radius: 5px")
        self.label_SP.setStyleSheet("color:rgb(203, 26, 126);" "background-color:rgb(250,250,250);"
                                    "border-style: solid;" "border-width: 1px;" "border-color: rgb(200,200,200);"
                                    "border-radius: 5px")
        self.label_LV.setStyleSheet("color:rgb(44, 106, 180);" "background-color:rgb(250,250,250);"
                                    "border-style: solid;" "border-width: 1px;" "border-color: rgb(200,200,200);"
                                    "border-radius: 5px")
        self.label_LC.setStyleSheet("color:rgb(44, 106, 180);" "background-color:rgb(250,250,250);"
                                    "border-style: solid;" "border-width: 1px;" "border-color: rgb(200,200,200);"
                                    "border-radius: 5px")
        self.label_LP.setStyleSheet("color:rgb(44, 106, 180);" "background-color:rgb(250,250,250);"
                                    "border-style: solid;" "border-width: 1px;" "border-color: rgb(200,200,200);"
                                    "border-radius: 5px")
        self.label_T.setStyleSheet("color:rgb(244, 121, 40);" "background-color:rgb(250,250,250);"
                                   "border-style: solid;" "border-width: 1px;" "border-color: rgb(200,200,200);"
                                   "border-radius: 5px")
        self.label_TTL.setStyleSheet("color:rgb(145, 122, 184);" "background-color:rgb(240,240,240);"
                                     "border-style: solid;" "border-width: 1px;" "border-color: rgb(200,200,200);"
                                     "border-radius: 5px")
        self.label_LIFE.setStyleSheet("color:rgb(120, 120, 120);" "background-color:rgb(240,240,240);"
                                     "border-style: solid;" "border-width: 1px;" "border-color: rgb(200,200,200);"
                                     "border-radius: 5px")
        self.label_Status.setStyleSheet("color:rgb(44, 106, 180)")

        # Data Indicator 글꼴 설정
        labelfont1 = self.label_SV.font()
        labelfont1.setFamily('Bahnschrift SemiLight')
        labelfont1.setPointSize(15)
        labelfont1.setBold(True)
        self.label_SV.setFont(labelfont1)
        self.label_SC.setFont(labelfont1)
        self.label_SP.setFont(labelfont1)
        self.label_LV.setFont(labelfont1)
        self.label_LC.setFont(labelfont1)
        self.label_LP.setFont(labelfont1)
        self.label_T.setFont(labelfont1)
        self.label_TTL.setFont(labelfont1)
        self.label_LIFE.setFont(labelfont1)
        self.label_Status.setFont(labelfont1)

        # 그룹박스와 Data Indicator 라벨 그룹화
        Gbox0.addWidget(self.label_SV)
        Gbox1.addWidget(self.label_SC)
        Gbox2.addWidget(self.label_SP)
        Gbox3.addWidget(self.label_LV)
        Gbox4.addWidget(self.label_LC)
        Gbox5.addWidget(self.label_LP)
        Gbox6.addWidget(self.label_T)
        Gbox7.addWidget(self.label_TTL)
        Gbox8.addWidget(self.label_LIFE)
        Gbox9.addWidget(self.label_Status)
        self.groupbox_SV.setLayout(Gbox0)
        self.groupbox_SC.setLayout(Gbox1)
        self.groupbox_SP.setLayout(Gbox2)
        self.groupbox_LV.setLayout(Gbox3)
        self.groupbox_LC.setLayout(Gbox4)
        self.groupbox_LP.setLayout(Gbox5)
        self.groupbox_T.setLayout(Gbox6)
        self.groupbox_TTL.setLayout(Gbox7)
        self.groupbox_LIFE.setLayout(Gbox8)
        self.groupbox_Status.setLayout(Gbox9)

        # 수평방향으로 창 그룹화 (1행 그래프 2개), (2행 그래프 2개), (3행 라벨 10개)
        hbox1.addWidget(self.Power)
        hbox1.addWidget(self.Energy)
        hbox2.addWidget(self.Voltage)
        hbox2.addWidget(self.Temperature)
        hbox3.addWidget(self.groupbox_SV)
        hbox3.addWidget(self.groupbox_SC)
        hbox3.addWidget(self.groupbox_SP)
        hbox3.addWidget(self.groupbox_LV)
        hbox3.addWidget(self.groupbox_LC)
        hbox3.addWidget(self.groupbox_LP)
        hbox3.addWidget(self.groupbox_T)
        hbox3.addWidget(self.groupbox_TTL)
        hbox3.addWidget(self.groupbox_LIFE)
        hbox3.addWidget(self.groupbox_Status)
        
        # 그룹화된 창 수직방향으로 그룹화
        vbox1.addLayout(hbox1)
        vbox1.addLayout(hbox2)
        vbox1.addLayout(hbox3)

        # 윈도우창생성 및 레이아웃 배치
        self.setLayout(vbox1)
        self.setGeometry(100, 100, 2600, 1000)  # 창 위치(x, y), width, height
        self.setWindowTitle("DONGYANG E&P INVERTER MONITORING PROGRAM  v0.61 by RAISON  -  %s  -"% Main.DF_Date)

        # X축 범위 생성
        self.Power.enableAutoRange(axis='x')
        self.Energy.enableAutoRange(axis='x')
        self.Voltage.enableAutoRange(axis='x')
        self.Temperature.enableAutoRange(axis='x')

        # Y축 범위 생성
        self.Power.setYRange(0, 1000)
        self.Energy.setYRange(1000, 2000)
        #self.Voltage.setYRange(50, 250)
        #self.Temperature.setYRange(-20, 70)
        #self.Power.enableAutoRange(axis='y')
        #self.Energy.enableAutoRange(axis='y')
        self.Voltage.enableAutoRange(axis='y')
        self.Temperature.enableAutoRange(axis='y')

        # 그래프 펜 설정
        self.SolP_curve = self.Power.plot(pen=pyqtgraph.mkPen(color=(203, 26, 126), width=3, style=QtCore.Qt.SolidLine ))
        self.LineP_curve = self.Power.plot(pen=pyqtgraph.mkPen(color=(44,106, 180), width=3, style=QtCore.Qt.DotLine ))
        self.Energy_curve = self.Energy.plot(pen=pyqtgraph.mkPen(color=(145, 122, 184), width=4, style=QtCore.Qt.SolidLine ))
        self.SolV_curve = self.Voltage.plot(pen=pyqtgraph.mkPen(color=(203, 26, 126), width=3, style=QtCore.Qt.SolidLine ))
        self.LineV_curve = self.Voltage.plot(pen=pyqtgraph.mkPen(color=(44, 106, 180), width=3, style=QtCore.Qt.SolidLine ))
        self.Efficency_curve = self.Voltage.plot(pen=pyqtgraph.mkPen(color=(120, 120, 120), width=3, style=QtCore.Qt.SolidLine ))
        self.Temperature_curve = self.Temperature.plot(pen=pyqtgraph.mkPen(color=(244, 121, 40), width=4, style=QtCore.Qt.SolidLine ))

        # 일정 시간 마다 그래프 및 라벨값 갱신
        self.Data_Get_Timer = QTimer()
        self.Data_Get_Timer.setInterval(Main.Cycle_Time)
        self.Data_Get_Timer.timeout.connect(self.update)
        self.Data_Get_Timer.start()
        self.show()

    @pyqtSlot()
    def update(self):
        # INDEX용 시간 계산
        self.DF_Time = time.strftime('%H:%M:%S', time.localtime(time.time()))
        # 인버터 데이터 수신
        Telemetry = self.Telemetry_RX(Main.Port, Main.BaudRate, Main.bytesize, Main.NetID, Main.DataLength)
        # 정상 수신시 여부 확인
        if type(Telemetry) == dict:
            # Error 카운트 리셋
            Error_Count = 0
            # 인버터 작동 상태 저장
            if Telemetry['RUN'] == 1:
                Inverter_Status = "Inverter RUN"
            else:
                Inverter_Status = "Inverter STOP"
            # 데이터프레임 변환 및 csv파일로 누적 저장
            Result_Temp = DataFrame(Telemetry, index = [self.DF_Time])
            DF_Result = Main.DF_Result.append(Result_Temp)
            if not os.path.exists('./Data/%s.csv'% Main.OutFileName):
                Result_Temp.to_csv('./Data/%s.csv'% Main.OutFileName, mode='w')
            else:
                Result_Temp.to_csv('./Data/%s.csv'% Main.OutFileName, mode='a', header=False)
            print("%s Monitoring... %s" % (self.DF_Time, Inverter_Status))
    
            # X축 최대 눈금 제한 (Tick간 겹치지않게 숫자 설정)
            xMAXTickN = 100
            xdict = dict(enumerate(DF_Result.index))
            xlist = list(xdict.items())
            xticklist = []
            if len(xlist) > xMAXTickN:
                tick_interval = len(xlist)//xMAXTickN
                tick_key = 0
                for i in range(xMAXTickN+1):
                    xticklist.append(xlist[tick_key])
                    tick_key += tick_interval
                    # Tick키가 리스트 범위를 넘지 않도록 조건문 추가
                    if tick_key >= len(xlist):
                        tick_key = len(xlist)-1
                self.stringaxisP.setTicks([xticklist])
                self.stringaxisE.setTicks([xticklist])
                self.stringaxisV.setTicks([xticklist])
                self.stringaxisT.setTicks([xticklist])
            else:
                self.stringaxisP.setTicks([xdict.items()])
                self.stringaxisE.setTicks([xdict.items()])
                self.stringaxisV.setTicks([xdict.items()])
                self.stringaxisT.setTicks([xdict.items()])
    
            # 그래프 갱신
            self.SolP_curve.setData(DF_Result.Solar_Power, name="Solar Power")
            self.LineP_curve.setData(DF_Result.Line_Power, name="Line Power")
            self.Energy_curve.setData(DF_Result.Today_GEN_Power, name="Today Gen. Energy")
            self.SolV_curve.setData(DF_Result.Solar_Voltage, name="Solar Voltage")
            self.LineV_curve.setData(DF_Result.Line_Voltage, name="Line Voltage")
            self.Efficency_curve.setData(DF_Result.Inverter_Efficiency, name="Inverter Efficiency")
            self.Temperature_curve.setData(DF_Result.Temperature, name="Inverter Temperature")
    
            # Inverter_Status Indicator 글꼴 재정의
            labelfont2 = self.label_Status.font()
            labelfont2.setFamily('Bahnschrift SemiLight')
            labelfont2.setPointSize(12)
            labelfont2.setBold(True)
            self.label_Status.setFont(labelfont2)
    
            # Data Indicator 갱신
            self.label_SV.setText("%.1f V" % Result_Temp['Solar_Voltage'].values.tolist()[0])
            self.label_SC.setText("%.1f A" % Result_Temp['Solar_Current'].values.tolist()[0])
            self.label_SP.setText("%.1f W" % Result_Temp['Solar_Power'].values.tolist()[0])
            self.label_LV.setText("%.1f V" % Result_Temp['Line_Voltage'].values.tolist()[0])
            self.label_LC.setText("%.1f A" % Result_Temp['Line_Current'].values.tolist()[0])
            self.label_LP.setText("%.1f W" % Result_Temp['Line_Power'].values.tolist()[0])
            self.label_T.setText("%.1f ℃" % Result_Temp['Temperature'].values.tolist()[0])
            self.label_TTL.setText("%.0f Wh" % Result_Temp['Today_GEN_Power'].values.tolist()[0])
            self.label_LIFE.setText("%.0f kWh" % Result_Temp['TTL_GEN_Power'].values.tolist()[0])
            
            # 인버터 상태에 따라 색상변경
            if Inverter_Status == "Inverter STOP":
                self.label_Status.setStyleSheet("color:rgb(240, 5, 50)")
                self.label_Status.setText(Inverter_Status)
            else:
                self.label_Status.setStyleSheet("color:rgb(44, 106, 180)")
                self.label_Status.setText(Inverter_Status)
    
        # 연속 Error 발생시 글자 색상, 크기 변경 후 출력 및 강제 종료
        else:
            Inverter_Status = Telemetry
            Main.Error_Count += 1
            #print(Inverter_Status)
    
            # Inverter_Status Indicator 글꼴 재정의
            labelfont2 = self.label_Status.font()
            labelfont2.setFamily('Bahnschrift SemiLight')
            labelfont2.setPointSize(8)
            labelfont2.setBold(False)
            self.label_Status.setFont(labelfont2)
            self.label_Status.setStyleSheet("color:rgb(240, 5, 50)")
            self.label_Status.setText(Inverter_Status)
            if Main.Error_Count > 15:
                print("Ending Monitoring...Too many Error")
                self.Data_Get_Timer.stop()


    # 데이터 송수신 함수 정의
    def Telemetry_RX(self, Port, BaudRate, bytesize, NetID, RS485RXANumber):
        global alt
        global target
        global error
        try:

            """
            # 통신포트 설정 및 연결
            ser = serial.Serial(Port, BaudRate)
            ser.port = Port
            ser.baudrate = BaudRate
            ser.stopbits = serial.STOPBITS_ONE
            ser.bytesize = bytesize
            ser.parity = serial.PARITY_NONE
            ser.rtscts = 0
            ser.timeout = 0.6
            ser.open()
            # 수신 문자열 정의
            RS485RXArray =[]
            # 데이터 송신
            ser.write(bytes(bytearray([0x0A,0x96,NetID,0x54,0x18,0x05,108+NetID])))
            # 데이터 문자열 수신    
            for i in  range(RS485RXANumber):
                mHex = ser.read()
                RS485RXArray.append(binascii.hexlify(bytearray(mHex)))
            # 수신데이터 길이 검증
            if len(RS485RXArray) != RS485RXANumber:
                ser.close()
                return 'RX Err : Wrong Length'
            # 수신데이터 헤더 검증
            if RS485RXArray[0] != b'b1' or RS485RXArray[1] != b'b7':
                ser.close()
                return 'RX Err : Wrong Header'
            # 수신데이터 Station ID 검증
            if int(RS485RXArray[2],16) != StationID :
                ser.close()
                return 'RX Err : Wrong Station ID'
            # 수신데이터 CheckSum 검증
            Check = 0
            RX_Check = int(RS485RXArray[31],16)
            for i in range(RS485RXANumber-1):
                Check = Check ^ int(RS485RXArray[i],16)
            if Check != RX_Check :
                ser.close()
                return 'RX Err : Wrong CheckSUM'
            # FaultCode 변환
            RX_FaultCode = RS485RXArray[23] +RS485RXArray[22] + RS485RXArray[21] + RS485RXArray[20]
            if RX_FaultCode  == bytes(b'00000000'):
                CX_Fault  = "No Fault"
            elif RX_FaultCode  == bytes(b'00000001'):
                CX_Fault  = "Solar Over Current"
            elif RX_FaultCode  == bytes(b'00000002'):
                CX_Fault  = "Solar Over Voltage"
            elif RX_FaultCode  == bytes(b'00000004'):
                CX_Fault  = "Solar Low Voltage"
            elif RX_FaultCode  == bytes(b'00000008'):
                CX_Fault  = "DCLink Over Voltage"
            elif RX_FaultCode  == bytes(b'00000010'):
                CX_Fault  = "DCLink Low Voltage"
            elif RX_FaultCode  == bytes(b'00000020'):
                CX_Fault  = "Inverter Over Current"
            elif RX_FaultCode  == bytes(b'00000040'):
                CX_Fault  = "ACLine Over Voltage"
            elif RX_FaultCode  == bytes(b'00000080'):
                CX_Fault  = "ACLine Low Voltage"
            elif RX_FaultCode  == bytes(b'00000100'):
                CX_Fault  = "Internal temperature exceeded"
            elif RX_FaultCode  == bytes(b'00000200'):
                CX_Fault  = "ACLine Over Frequency"
            elif RX_FaultCode  == bytes(b'00000400'):
                CX_Fault  = "ACLine Low Frequency"
            elif RX_FaultCode  == bytes(b'00000800'):
                CX_Fault  = "Solar Over Power"
            elif RX_FaultCode  == bytes(b'00001000'):
                CX_Fault  = "DC Component Exceeds the value"
            elif RX_FaultCode  == bytes(b'00002000'):
                CX_Fault  = "DC Short Circuit"
            elif RX_FaultCode  == bytes(b'00010000'):
                CX_Fault  = "Single Operation"
            elif RX_FaultCode  == bytes(b'00020000'):
                CX_Fault  = "Inverter Over Current HW"
            # FaultCode 확인
            if CX_Fault  != "No Fault":
                ser.close()
                return 'Fault : %d' % CX_Fault
            #  수신 Data 변환
            RX_SolarV1 = int(RS485RXArray[4] + RS485RXArray[3],16)/10
            RX_SolarV2 = int(RS485RXArray[8] + RS485RXArray[7],16)/10
            RX_SolarCurrent = int(RS485RXArray[6] + RS485RXArray[5],16)/10
            RX_LineVoltage = int(RS485RXArray[10] + RS485RXArray[9],16)/10
            RX_LineCurrent = int(RS485RXArray[12] + RS485RXArray[11],16)/10
            RX_Temperature = int(RS485RXArray[14] + RS485RXArray[13],16)/10
            RX_TodayTTL = int(RS485RXArray[16] + RS485RXArray[15],16)*10
            RX_EnergyTTL = int(RS485RXArray[19] + RS485RXArray[18] + RS485RXArray[17],16)
            RX_Frequency = int(RS485RXArray[26] + RS485RXArray[25],16)/10
            RX_OperationTime = int(RS485RXArray[28] + RS485RXArray[27],16)
            RX_PowerFactor = int(RS485RXArray[29],16)/100
            RX_DSPVersion = int(RS485RXArray[30],16)/100
            RX_RUN = int(RS485RXArray[24],16)
            # 전력값 및 효율 계산
            CX_SolarVoltage = round((RX_SolarV1 + RX_SolarV2)/2,2)
            CX_SolPower = round(CX_SolarVoltage * RX_SolarCurrent,3)
            CX_LinePower = round(RX_LineVoltage * RX_LineCurrent,3)
            if CX_SolPower != 0:
                CX_InverterEfficiency = round((CX_LinePower/CX_SolPower)*100,2)
            else:
                CX_InverterEfficiency = 0
            """

            a = int(self.ser.read(1).hex(), 16)

            if a == 0x88:
                b = int(self.ser.read(1).hex(), 16)
                if b == 0x18:

                    alt_1 = int(self.ser.read(1).hex(), 16) & 0xff
                    alt_2 = int(self.ser.read(1).hex(), 16) & 0xff
                    alt_3 = int(self.ser.read(1).hex(), 16) & 0xff
                    alt_4 = int(self.ser.read(1).hex(), 16) & 0xff
                    alt_sign = alt_1 >> 7

                    target_1 = int(self.ser.read(1).hex(), 16) & 0xff
                    target_2 = int(self.ser.read(1).hex(), 16) & 0xff
                    target_3 = int(self.ser.read(1).hex(), 16) & 0xff
                    target_4 = int(self.ser.read(1).hex(), 16) & 0xff
                    target_sign = target_1 >> 7

                    error_1 = int(self.ser.read(1).hex(), 16) & 0xff
                    error_2 = int(self.ser.read(1).hex(), 16) & 0xff
                    error_3 = int(self.ser.read(1).hex(), 16) & 0xff
                    error_4 = int(self.ser.read(1).hex(), 16) & 0xff
                    error_sign = error_1 >> 7

                    alt = alt_1 << 24 | alt_2 << 16 | alt_3 << 8 | alt_4
                    target = target_1 << 24 | target_2 << 16 | target_3 << 8 | target_4
                    error = error_1 << 24 | error_2 << 16 | error_3 << 8 | error_4

                    if alt_sign == 1: alt = (alt & 0x7fffffff) - 2 ** 31
                    if target_sign == 1 : target = (target & 0x7fffffff) - 2 ** 31
                    if error_sign == 1: error = (error & 0x7fffffff) - 2 ** 31

            #  출력값 반환
            CX_Result = {'Solar_Power': alt,
                         'Today_GEN_Power': target,
                         'Solar_Voltage': error,
                         'Solar_Current': 0,
                         'Line_Voltage': 0,
                         'Line_Current': 0,
                         'Line_Power': 0,
                         'Inverter_Efficiency': 0,
                         'Temperature': 0,
                         'TTL_GEN_Power': 0,
                         'AC_Frequency' : 0,
                         'RUN_Time' : 0,
                         'PF' : 0,
                         'DSP_ver' : 0,
                         'RUN' : 0}

            return CX_Result
        except serial.SerialException as err:
            self.ser.close()
            return 'Serial Port Error : %s' % err

if __name__ == "__main__":

    app = QApplication(sys.argv)
    ex = Main()
    sys.exit(app.exec_())
