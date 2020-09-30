
"""
[Desktop Entry]
Name=Dyna
Exec=/home/alejandro/.virtualenvs/Dyna/bin/python /home/alejandro/Documentos/Dyna/MainGUI.py
Terminal=true
Type=Application
Icon=/Dyna/img/AnalogGaugeWidgetDemo.png
"""
# pyuic5 -x screen.ui -o screen.py
# pip install pyserial

from screen import Ui_mainWindow
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QInputDialog, QFileDialog
from PyQt5.QtCore import QThread, pyqtSignal, QSize, QRect
from PyQt5.QtGui import QPixmap
import time
import pyqtgraph as pg
import math
from serial.tools import list_ports
from serial import Serial
import warnings
import struct
from scipy.signal import savgol_filter, medfilt
import numpy as np
import os
from sys import platform
import pandas as pd


# Child class from Ui object
class App(QMainWindow, Ui_mainWindow):
    def __init__(self):
        super(App, self).__init__()

        # Set up the user interface from Designer
        self.setupUi(self)

        # Se crea un evento para cada vez que termine el while en DAQ.animate ( el evento llega gracias al emit)
        # Se crea una instancia del objeto que crea eventos (DAQ)
        self.thread = DAQ(self)
        # Se conectan las señales (dataChanged) con la funcion a realizar (onDataChanged)(slots)
        self.thread.dataChanged.connect(self.onDataChanged)
        self.thread.start()

        # Variables para graficar
        self.torque = np.array([0, 0, 0, 0, 0, 0, 0])
        self.torque_vs_rpm = np.array([0, 0, 0, 0, 0, 0, 0])
        self.horse_power = np.array([0, 0, 0, 0, 0, 0, 0])
        self.horse_power_vs_rpm = np.array([0, 0, 0, 0, 0, 0, 0])
        self.elapsedTime = np.array([0, 0, 0, 0, 0, 0, 0])
        self.rpm = np.array([0, 0, 0, 0, 0, 0, 0])
        self.plot_rpm = np.array([0, 0])
        self.plot_torque_vs_rpm = np.array([0, 0])
        self.plot_hp_vs_rpm = np.array([0, 0])
        self.loaded_RPM_data = np.array([0, 0])
        self.loaded_torque_data = np.array([0, 0])
        self.loaded_hp_data = np.array([0, 0])

        # Variables para calculo
        self.max_rpm = 0
        self.max_torque = 0
        self.max_horse_power = 0
        self.old_time = 0
        self.old_hz = 0
        self.old_hz2 = 0
        self.old_torque_rod = 0
        self.old_torque_rod2 = 0
        self.first_iteration = True
        self.relacion_reduccion = 1
        self.counter = 0
        self.delta_t = 100  # current_time - self.old_time  # en segundos
        self.stop_count = 0
        self.initime = 0

        # Leds
        path = os.path.dirname(os.path.abspath(__file__))
        print(path)
        led_off = path + "/img/led_off.png"
        led_on = path + "/img/led_on.png"
        self.led_off_img = (QPixmap(led_off)).scaled(30, 30)
        self.led_on_img = (QPixmap(led_on)).scaled(30, 30)
        self.led_label.setPixmap(self.led_off_img)

        # Variables de estados
        self.state = False
        self.reset_flag = False
        self.first = True
        self.setting_flag = False
        self.csv_name = ' '

        # plot de class pg  --- plot settings

        self.plot = pg.PlotWidget()
        self.wideband_plot = pg.PlotWidget()
        # self.plot.setXRange(13,)                                                               # trying

        self.verticalLayout_3.addWidget(self.plot)
        self.verticalLayout_11.addWidget(self.wideband_plot)
        print(self.verticalLayout_11.maximumSize())


        self.p1 = self.plot.plotItem
        self.p1.showGrid(x=True, y=True, alpha=0.5)

        self.wideband_p1 = self.wideband_plot.plotItem
        self.wideband_p1.showGrid(x=True, y=True, alpha=0.5)
        axis_x_wideband = self.wideband_p1.getAxis('bottom')
        axis_x_wideband.setTickSpacing(500, 1)

        axis_x = self.p1.getAxis('bottom')
        axis_x.setTickSpacing(500, 1)

        axis_torque = self.p1.getAxis('left')
        axis_torque.setLabel('Torque (Nm)', color='#0000ff')

        self.TorqueCurve = self.p1.plot()
        self.TorqueCurve.setPen(pg.mkPen(color='#0000ff', width=2))

        self.p2 = pg.ViewBox()
        self.p1.setAutoVisible()

        self.loaded_torque = pg.PlotCurveItem()
        self.loaded_torque.setPen(pg.mkPen(color="#5050ff", width=2))
        self.loaded_hp = pg.PlotCurveItem()
        self.loaded_hp.setPen(pg.mkPen(color="#ff5050", width=2))

        self.p1.addItem(self.loaded_torque)
        self.p2.addItem(self.loaded_hp)

        self.HorsePowerCurve = pg.PlotCurveItem()
        self.HorsePowerCurve.setPen(pg.mkPen(color="#ff0000", width=2))
        self.p2.addItem(self.HorsePowerCurve)
        self.p1.scene().addItem(self.p2)

        self.p1.showAxis('right')
        self.p1.getAxis('right').setLabel('Potencia (HP)', color="#ff0000")
        self.p1.getAxis('right').linkToView(self.p2)
        # Resize

        self.p1.vb.sigResized.connect(self.updateViews)

        def mouse_moved(evt):
            if self.plot.sceneBoundingRect().contains(evt):
                mouse_point = self.p1.vb.mapSceneToView(evt)
                index = int(mouse_point.x())
                if index > 0:
                    idx = (np.abs(self.plot_rpm - index)).argmin()
                    idx2 = (np.abs(self.loaded_RPM_data - index)).argmin()
                    text = "<span>RPM  = {:.0f} </span> |" \
                           "<span style='color: red'>  Potencia (HP) = {:.2f} </span> |" \
                           "<span style='color: blue'>  Torque (Nm) = {:.2f} </span> |" \
                           "<span style='color: red'>  Potencia (Prueba cargada) (HP) = {:.2f} </span> |" \
                           "<span style='color: blue'>  Torque (Prueba cargada) (Nm) = {:.2f} </span>" \
                           "".format(mouse_point.x(), self.plot_hp_vs_rpm[idx], self.plot_torque_vs_rpm[idx],
                                     self.loaded_hp_data[idx2], self.loaded_torque_data[idx2])
                    self.results_label.setText(text)

        self.p1.scene().sigMouseMoved.connect(mouse_moved)

        # Connect up the buttons.
        self.Start.clicked.connect(self.startFcn)
        self.Stop.clicked.connect(self.stopFcn)
        self.resetBtn.clicked.connect(self.resetFcn)
        self.setear_rpm_motor.clicked.connect(self.set_rpm_moto)
        self.setear_rt.clicked.connect(self.set_rt_moto)
        self.actionSave.triggered.connect(self.shoot)
        self.actionOpen.triggered.connect(self.open_csv)
        self.ResetRR.clicked.connect(self.resetrr)

        self.results_label.setText('')
        self.RPMcheckBox.setChecked(True)
        self.rpm_ref.setText("3000")
        self.rt_manual.setText("3")

        # Gauge Settings
        self.Gauge.set_MaxValue(250)
        self.Gaugerpm.set_MaxValue(12000)

    def updateViews(self):
        self.p2.setGeometry(self.p1.vb.sceneBoundingRect())
        self.p2.linkedViewChanged(self.p1.vb, self.p2.XAxis)

    def onDataChanged(self, pulsos, wideband):  # Envia rpm del rodillo
        print(wideband)
        current_time = time.time()
        rpm_rod = pulsos  # (pulsos / 0.1)  * (1/600) * (60)  = 1* pulsos
        hz_rod_temp = (rpm_rod * 2 * math.pi) / 60
        # hz_rod_temp = rpm_rod * 0.0166

        if self.setting_flag:
            # Se calcula la relacion de reducción
            self.relacion_reduccion = (rpm_rod / int(self.rpm_ref.text()))
            self.max_rpm = 0
            self.max_torque = 0
            self.max_horse_power = 0
            self.setting_flag = False

        hz_rod = hz_rod_temp * 0.9 + self.old_hz * 0.1

        rpm_moto = (hz_rod * 60 / (self.relacion_reduccion * 2 * math.pi))
        self.Gaugerpm.update_value(rpm_moto)
        self.LCDrr.display(self.relacion_reduccion)
        kmh = (hz_rod * 0.54864)  # 0.1524)*3.6  # Multiplicando por el radio del rodillo en m

        self.Gauge.update_value(kmh)
        # print("{:12.2f}".format(rpm_rod,"rpm"))
        # rpm_rod = hz_rod * 60

        if hz_rod_temp <= self.old_hz:
            exit_var = True
        else:
            exit_var = False

        if self.stop_count > 20:
            self.state = False
            self.first_iteration = True

        # Si esta en modo run
        if self.state:
            self.led_label.setPixmap(self.led_on_img)

            #  En la primera iteracion reciba los valores parala siguiente
            if self.first_iteration:
                self.old_hz2 = self.old_hz
                self.old_hz = hz_rod
                self.old_time = current_time
                self.old_torque_rod = 0
                self.old_torque_rod2 = 0
                self.first_iteration = False

            else:
                # Calculos
                disc_inertia = 0.6243
                inertia_total = disc_inertia + 3.87 # Kg * m2
                delta_w = hz_rod - self.old_hz
                acc = delta_w / self.delta_t
                # Se calcula el torque en el rodillo con sumatoria de torques
                '''
                roller_load = (80 + 100) * 9.81  # Equivalent Load on the bearing
                flywheel_load = 25 * 9.81
                u = 0.015
                dm = 0.05  # Pitch diameter of bearing
                fr_bearings = u * dm * 2 * (roller_load + flywheel_load * 4)
                0.5 x 0.0015 x radial load in Newtons* x bearing bore (mm)
                0.0015 x radial load in Newtons* x 25 = 66Nmm
                chain_friction = 0.02 * disc_inertia * 1000 * 4 * acc  # aprox 50 * acc   # 2 % of transmission loss
                '''
                chain_friction = 50 * acc  # 2 % of transmission loss
                torque_rod_temp = (inertia_total * 1000 * acc) + chain_friction + 0.0662  # + fr_bearings Nm

                # fir filter
                torque_rod = torque_rod_temp * 0.8 + self.old_torque_rod * 0.1 + self.old_torque_rod2 * 0.1

                self.old_torque_rod2 = self.old_torque_rod
                self.old_torque_rod = torque_rod

                # --------------------- Add average torque and hp -------------------- #

                # Se transforma a torque en el motor
                torque_moto = torque_rod * self.relacion_reduccion

                # Calculo de potencia
                horse_power = (torque_rod * hz_rod) / 733.04  # Watt to hp

                if not exit_var:

                    # Se Sacan los valores máximos
                    if rpm_moto > self.max_rpm:
                        self.max_rpm = rpm_moto

                        self.rpm = np.append(self.rpm, rpm_moto)
                        self.plot_rpm = self.rpm[(self.rpm > 500)]

                        self.torque_vs_rpm = np.append(self.torque_vs_rpm, torque_moto)
                        self.torque_vs_rpm = savgol_filter(self.torque_vs_rpm, 5, 2)
                        self.plot_torque_vs_rpm = self.torque_vs_rpm[(self.rpm > 500)]
                        self.plot_torque_vs_rpm = np.around(self.plot_torque_vs_rpm, decimals=3)

                        self.horse_power_vs_rpm = np.append(self.horse_power_vs_rpm, horse_power)
                        self.horse_power_vs_rpm = savgol_filter(self.horse_power_vs_rpm, 5, 2)
                        # a = medfilt(self.horse_power_vs_rpm, 15)
                        self.plot_hp_vs_rpm = self.horse_power_vs_rpm[(self.rpm > 500)]
                        # self.plot_hp_vs_rpm = a[(self.rpm > 500)]
                        self.plot_hp_vs_rpm = np.around(self.plot_hp_vs_rpm, decimals=3)

                        self.lcdMaxRPM.display(self.max_rpm)
                        self.lcdMaxTorque.display(round(np.max(self.plot_torque_vs_rpm), 2))
                        self.lcdMaxHP.display(round(np.max(self.plot_hp_vs_rpm), 2))

                    # Si el rodillo esta frenando
                    else:
                        self.stop_count += 1
                        # print(self.stop_count)

                        # Si ha pasado varias iteraciones frenando
                        if self.stop_count > 8:
                            self.state = False
                            self.first_iteration = True

                    if torque_moto > self.max_torque:
                        self.max_torque = torque_moto
                    if horse_power > self.max_horse_power:
                        self.max_horse_power = horse_power

                    # Cuando se grafica respecto al tiempo
                    elapsed = round((current_time - self.initime), 3)
                    self.lcdRunTime.display(elapsed)
                    self.elapsedTime = np.append(self.elapsedTime, elapsed)

                    # self.torque = savgol_filter(self.torque, 3, 2)
                    self.torque = np.append(self.torque, torque_moto)

                    # self.horse_power = savgol_filter(self.horse_power, 3, 2)
                    # self.horse_power.append(rpm_moto)
                    self.horse_power = np.append(self.horse_power, horse_power)

                    if self.RPMcheckBox.isChecked():
                        # self.TorqueCurve.setData(self.rpm, self.torque_vs_rpm)  # los vectores rpm y torque_vs_rpm
                        self.TorqueCurve.setData(self.plot_rpm, self.plot_torque_vs_rpm)  # los vectores rpm y torque_vs_rpm
                        # print(self.rpm,self.torque_vs_rpm)
                        self.HorsePowerCurve.setData(self.plot_rpm, self.plot_hp_vs_rpm)  # los vectores rpm y hp_vs_rpm
                        self.updateViews()
                    else:
                        self.TorqueCurve.setData(self.elapsedTime, self.torque)
                        self.HorsePowerCurve.setData(self.elapsedTime, self.horse_power)
                        self.updateViews()

        # Si no exit var
        else:
            self.led_label.setPixmap(self.led_off_img)

        # Actualiza variables
        self.old_hz2 = self.old_hz
        self.old_hz = hz_rod
        self.old_time = current_time

    def resetrr(self):
        self.relacion_reduccion = 1

    def open_csv(self):
        fname = QFileDialog.getOpenFileName(self, 'Open file', 'pruebas/', 'csv files(*.csv )')
        print(fname[0])
        if not fname == '':
            df = pd.read_csv(fname[0])
            self.loaded_RPM_data = df['RPM'].to_numpy()
            self.loaded_hp_data = df['Hp'].to_numpy()
            self.loaded_torque_data = df['Nm'].to_numpy()
            self.loaded_torque.setData(self.loaded_RPM_data, self.loaded_torque_data)
            self.loaded_hp.setData(self.loaded_RPM_data, self.loaded_hp_data)
            self.updateViews()
            self.lcdMaxRPM.display(round(np.max(self.plot_rpm), 0))
            self.lcdMaxTorque.display(round(np.max(self.plot_torque_vs_rpm), 2))
            self.lcdMaxHP.display(round(np.max(self.plot_hp_vs_rpm), 2))

    def shoot(self):
        # Tome un pantallaso
        name, ok = QInputDialog.getText(self, "Nombre de prueba", "Ingrese nombre de prueba (MOTO_CLIENTE_ESTADO_DD_MM_AA)")
        if ok:
            self.csv_name = 'pruebas/' + name + '.csv'
        frame = {'RPM': self.plot_rpm, 'Hp': self.plot_hp_vs_rpm, 'Nm': self.plot_torque_vs_rpm}
        a = pd.DataFrame(data=frame)
        a.to_csv(self.csv_name, index=False)

    def resetFcn(self):
        self.results_label.setText('')
        self.stop_count = 0    # variable para parar cuendo termine la prueba
        # Numero de resets para guardar las pruebas (pantallazos)
        self.counter += 1

        # Si esta corriendo
        if self.state:
            # Se resetea el millis() relacionado con el runtime
            self.initime = time.time()

        # Las variables
        self.torque = np.array([0, 0, 0, 0, 0, 0])
        self.torque_vs_rpm = np.array([0, 0, 0, 0, 0, 0])
        self.rpm = np.array([0, 0, 0, 0, 0, 0])
        self.horse_power_vs_rpm = np.array([0, 0, 0, 0, 0, 0])
        self.horse_power = np.array([0, 0, 0, 0, 0, 0])
        self.elapsedTime = np.array([0, 0, 0, 0, 0, 0])
        self.plot_rpm = np.array([0, 0])
        self.plot_torque_vs_rpm = np.array([0, 0])
        self.plot_hp_vs_rpm = np.array([0, 0])
        self.loaded_RPM_data = np.array([0, 0])
        self.loaded_torque_data = np.array([0, 0])
        self.loaded_hp_data = np.array([0, 0])

        self.max_rpm = 0
        self.max_torque = 0
        self.max_horse_power = 0
        self.reset_flag = True

        # Los Displays
        self.lcdMaxRPM.display(0)
        self.lcdMaxTorque.display(0)
        self.lcdMaxHP.display(0)
        self.lcdRunTime.display(0)
        self.LCDrr.display(0)
        self.Gauge.update_value(0)
        self.Gaugerpm.update_value(0)

        # Las variables de las gaficas
        self.p1.setXRange(0, 1, padding=0)
        self.p1.setYRange(0, 1, padding=0)
        self.p2.setXRange(0, 1, padding=0)
        self.p2.setYRange(0, 1, padding=0)
        self.p1.enableAutoRange(axis='x')
        self.p1.enableAutoRange(axis='y')
        self.p2.enableAutoRange(axis='x')
        self.p2.enableAutoRange(axis='y')
        self.p1.setAutoVisible(x=True)
        self.p1.setAutoVisible(y=True)
        self.p1.setAutoPan(x=False)
        self.p1.setAutoPan(y=False)
        self.p2.setAutoVisible(x=True)
        self.p2.setAutoVisible(y=True)
        self.p2.setAutoPan(x=False)
        self.p2.setAutoPan(y=False)
        self.TorqueCurve.setData(self.elapsedTime, self.torque)
        self.HorsePowerCurve.setData(self.elapsedTime, self.horse_power)
        self.TorqueCurve.clear()
        self.HorsePowerCurve.clear()
        self.loaded_torque.clear()
        self.loaded_hp.clear()
        self.updateViews()

    def set_rpm_moto(self):
        self.setting_flag = True

    def set_rt_moto(self):
        try:
            self.relacion_reduccion = float(self.rt_manual.text())
        except ValueError:
            pass

    def startFcn(self):
        self.stop_count = 0
        # Si hay un reset previo
        if self.reset_flag or self.first:
            # Resetee el millis cuando arranque
            self.initime = time.time()

        # Para que llegen datos en la primera Iteracion y no sean calculados (divisiones por 0)
        self.first_iteration = True
        self.reset_flag = False
        self.state = True
        self.first = False

    def stopFcn(self):
        self.state = False
        self.first_iteration = True
        # self.close()


class DAQ(QThread):
    dataChanged = pyqtSignal(int, int)

    def __init__(self, parent=None):

        QThread.__init__(self, parent)

        if platform == "linux" or platform == "linux2":
            ports = list_ports.comports()

            if not ports:
                raise IOError("No pic found - is it plugged in?.")
                pass

            if len(ports) > 1:
                warnings.warn('Multiple ports found - using the first')
            self.PIC = Serial(ports[0], 115200, timeout=1)
        elif platform == "win32":
            locations = ["COM1", 'COM2', 'COM3', 'COM4', 'COM5']
            """
            for com in range(10):
                try:
                    port = 'COM'.strip() + str(com).strip()
                    print("Trying...", port)
                    self.PIC = Serial(port, 115200, timeout=1)
                    print("ok...", port)
                    break
                except:
                    print("Failed to connect on", port)
            """
            self.PIC = Serial("COM9", 115200, timeout=2)
            # self.PIC = Serial("COM7", 115200, timeout=2)

    def __del__(self):  # part of the standard format of a QThread
        # self.wait()
        pass

    def run(self):  # also a required QThread function, the working part when called start
        # i = 500
        # ini_t = time.time()
        while 1:
            '''
            # print(i)
            if i > 10000:
                i = 500
            current = time.time()
            delta_t = current - ini_t
            # print(delta_t)
            if (current - ini_t) > 0.1:
                self.dataChanged.emit(i)
                ini_t = time.time()
                i = i + 1 + np.random.random()*i/1000
            '''
            self.PIC.flushInput()
            incoming = self.PIC.read(5)
            incoming = struct.unpack("<bhh", incoming)
            start_byte = incoming[0]
            if start_byte == -52:
                pulses = incoming[1]
                wideband = incoming[2]
                print(pulses, wideband)
                self.dataChanged.emit(pulses, wideband)
            else:
                print("Need correction")
                time.sleep(0.01)
                self.PIC.flushInput()
            '''
            except ValueError:
                pass
            except Exception:
                pass
            '''



if __name__ == "__main__":
    # pg.setConfigOption('background', 'w')
    # pg.setConfigOption('foreground', 'k')
    import sys
    app = QApplication(sys.argv)
    dyno = App()
    screen_resolution = app.desktop().screenGeometry()
    width, height = screen_resolution.width(), screen_resolution.height()
    dyno.resize(width, height)
    dyno.show()
    sys.exit(app.exec_())