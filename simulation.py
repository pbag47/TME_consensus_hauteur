import asyncio
import cf_info
import csv
import consensus_control_law
import logging
import main_ui
import numpy
import qasync
import qwt
import sim_user_interface
import sys

from flight_state_class import FlightState

from PyQt5 import QtCore
from PyQt5.QtGui import QPen
from PyQt5.QtWidgets import QApplication, QMainWindow
from qwt import QwtPlotCurve, QwtPlotGrid, QwtPlotMarker, QwtSymbol

logger = logging.getLogger(__name__)


class Window(QMainWindow, sim_user_interface.Ui_MainWindow):
    def __init__(self, agents, parent=None):
        super().__init__(parent)
        self.common_yaw = 0
        self.agents = agents
        self.stopped = False
        self.delta_t = 0.04  # (s)
        self.graph_display_time = 3  # (s)
        self.graph_length = round(self.graph_display_time / self.delta_t)
        self.graph_time = [0.0] * self.graph_length

        file = open('sim_logs.csv', 'w')
        writer = csv.writer(file)
        writer.writerow(['Crazyflie name', 'QTM packet timestamp (s)',
                         'QTM_x (m)', 'QTM_y (m)', 'QTM_z (m)', 'cf_yaw (°)',
                         'QTM_vx (m/s)', 'QTM_vy (m/s)', 'QTM_vz (m/s)',
                         'x_g (m)', 'y_g (m)', 'z_g (m)', 'yaw_g (°)',
                         'roll_c (°)', 'pitch_c (°)', 'yaw_rate_c (°/s)', 'thrust_c (PWM)'])

        for agent in self.agents:
            agent.csv_logger = writer
            agent.state = FlightState.STANDBY
            agent.position = Coordinates(x=agent.initial_position[0],
                                         y=agent.initial_position[1],
                                         z=agent.takeoff_height)
            agent.velocity = Coordinates(x=agent.initial_position[0],
                                         y=agent.initial_position[1],
                                         z=agent.takeoff_height)

            agent.new_position = Coordinates(x=agent.initial_position[0],
                                             y=agent.initial_position[1],
                                             z=agent.takeoff_height)
            agent.new_velocity = Coordinates(x=agent.initial_position[0],
                                             y=agent.initial_position[1],
                                             z=agent.takeoff_height)

            agent.x_history = [agent.position.x] * self.graph_length
            agent.y_history = [agent.position.y] * self.graph_length
            agent.z_history = [agent.position.z] * self.graph_length

            agent.z_curve = QwtPlotCurve(agent.name)
            agent.z_curve.setData(self.graph_time, agent.z_history)
            agent.z_curve.setPen(QPen(QtCore.Qt.black, 0, QtCore.Qt.SolidLine))
            agent.z_curve.setData(agent.x_history, agent.y_history)

            agent.xy_curve = QwtPlotCurve(agent.name)
            agent.xy_curve.setPen(QPen(QtCore.Qt.black, 0, QtCore.Qt.SolidLine))

            agent.xy_marker = QwtPlotMarker(agent.name)
            symbol = QwtSymbol(QwtSymbol.Ellipse)
            symbol.setSize(QtCore.QSize(10, 10))
            agent.xy_marker.setSymbol(symbol)
            # label = QwtText(agent.name)
            # agent.xy_marker.setLabel(label)
            # agent.xy_marker.setLabel(agent.name)
            # agent.xy_marker.setLabelAlignment(QtCore.Qt.AlignBottom)

        self.setupUi(self)
        self.init_ui()
        self.show()

    def init_ui(self):
        self.close_button.clicked.connect(self.stop_button_callback)
        self.yaw_up.clicked.connect(self.yaw_up_callback)
        self.yaw_up.setEnabled(True)
        self.yaw_down.clicked.connect(self.yaw_down_callback)
        self.yaw_down.setEnabled(True)

        # self.pause.clicked.connect(self.pause_button_callback)
        # self.step.clicked.connect(self.step_button_callback)
        self.reset.clicked.connect(self.reset_button_callback)
        self.circle.clicked.connect(self.circle_button_callback)
        self.circle_wth_tgx.clicked.connect(self.circle_wth_tangent_x_axis_callback)
        # self.POI.clicked.connect(self.point_of_interest_button_callback)

        # self.x_graph.setTitle('Blank graph')
        # self.x_graph.setAxisTitle(2, 'Blank X axis')
        # self.x_graph.setAxisTitle(0, 'Blank Y axis')
        # self.x_graph.setAxisScale(0, -1.25, 1.25)

        self.y_graph.setTitle('Z (m) vs time (s)')
        self.y_graph.setAxisTitle(2, 'Time (s)')
        self.y_graph.setAxisTitle(0, 'Z (m)')
        self.y_graph.setAxisScale(0, -1.25, 1.25)

        self.xy_graph.setTitle('X (m) vs Y (m)')
        self.xy_graph.setAxisTitle(2, 'X (m)')
        self.xy_graph.setAxisTitle(0, 'Y (m)')
        self.xy_graph.setAxisScale(0, -1.5, 1.5)
        self.xy_graph.setAxisScale(2, -1.5, 1.5)

        xy_graph_grid = QwtPlotGrid()
        xy_graph_grid.setPen(QPen(QtCore.Qt.black, 0, QtCore.Qt.DotLine))

        for agent in self.agents:
            agent.z_curve.attach(self.y_graph)
            agent.xy_curve.attach(self.xy_graph)
            agent.xy_marker.attach(self.xy_graph)

        xy_graph_grid.attach(self.xy_graph)

        self.y_graph.insertLegend(qwt.QwtLegend(), qwt.QwtPlot.RightLegend)
        self.xy_graph.insertLegend(qwt.QwtLegend(), qwt.QwtPlot.RightLegend)

    def stop_button_callback(self):
        self.stopped = True
        self.close()

    def yaw_up_callback(self):
        self.common_yaw += 10
        for agent in self.agents:
            agent.yaw = self.common_yaw
        self.yaw.setText(str(self.common_yaw) + '°')

    def yaw_down_callback(self):
        self.common_yaw -= 10
        for agent in self.agents:
            agent.yaw = self.common_yaw
        self.yaw.setText(str(self.common_yaw) + '°')

    def pause_button_callback(self):
        for agent in self.agents:
            agent.state = FlightState.NOT_FLYING

    def reset_button_callback(self):
        for agent in self.agents:
            agent.state = FlightState.STANDBY
            agent.position.x = agent.initial_position[0]
            agent.position.y = agent.initial_position[1]
            agent.position.z = agent.takeoff_height
            agent.x_history = [agent.position.x] * self.graph_length
            agent.y_history = [agent.position.y] * self.graph_length
            agent.z_history = [agent.position.z] * self.graph_length

        self.yaw.setText('0°')

    def step_button_callback(self):
        for agent in self.agents:
            agent.state = FlightState.NOT_FLYING

    def circle_button_callback(self):
        for agent in self.agents:
            agent.state = FlightState.Z_CONSENSUS

    def circle_wth_tangent_x_axis_callback(self):
        for agent in self.agents:
            agent.state = FlightState.XY_CONSENSUS

    def point_of_interest_button_callback(self):
        self.reset_button_callback()

    async def timer(self):
        while not self.stopped:
            await asyncio.sleep(self.delta_t)
            for agent in self.agents:
                agent.timestamp = agent.timestamp + self.delta_t
                self.calculate_uav_response()

    def calculate_uav_response(self):
        """
        Computes the behaviour of the virtual UAV
            - Gets attitude commands from the control law
            - Calculates the response of the UAV model (double-integrator) to update the position of the virtual
              UAV
        """
        for agent in self.agents:
            if not agent.state == FlightState.NOT_FLYING:
                if agent.state == FlightState.Z_CONSENSUS:
                    try:
                        vz = consensus_control_law.z_consensus_control_law(agent, self.agents)
                    except Exception as e:
                        logger.error('Error "' + str(e) + '" detected in z_consensus_control_law function')
                        agent.error = e
                        agent.standby()
                        vz = 0

                    agent.new_velocity.x = 0
                    agent.new_velocity.y = 0
                    agent.new_velocity.z = vz

                    agent.new_position.x = agent.position.x
                    agent.new_position.y = agent.position.y
                    agent.new_position.z = agent.position.z + self.delta_t * vz

                if agent.state == FlightState.XY_CONSENSUS:
                    try:
                        roll, pitch = consensus_control_law.xy_consensus_control_law(agent, self.agents)
                    except Exception as e:
                        logger.error('Error "' + str(e) + '" detected in xy_consensus_control_law function')
                        agent.error = e
                        agent.standby()
                        roll = 0
                        pitch = 0

                    roll = roll * numpy.pi / 180
                    pitch = pitch * numpy.pi / 180

                    ax_n = 9.81 * pitch
                    ay_n = - 9.81 * roll

                    ax_e = ax_n * numpy.cos(agent.yaw * numpy.pi / 180) - ay_n * numpy.sin(
                        agent.yaw * numpy.pi / 180)
                    ay_e = ax_n * numpy.sin(agent.yaw * numpy.pi / 180) + ay_n * numpy.cos(
                        agent.yaw * numpy.pi / 180)

                    agent.new_velocity.x = agent.velocity.x + self.delta_t * ax_e
                    agent.new_velocity.y = agent.velocity.y + self.delta_t * ay_e
                    agent.new_velocity.z = 0

                    agent.new_position.x = agent.position.x + self.delta_t * agent.velocity.x
                    agent.new_position.y = agent.position.y + self.delta_t * agent.velocity.y
                    agent.new_position.z = agent.position.z

                if agent.state == FlightState.STANDBY:
                    agent.new_velocity.x = 0
                    agent.new_velocity.y = 0
                    agent.new_velocity.z = 0
                    agent.new_position.x = agent.position.x
                    agent.new_position.y = agent.position.y
                    agent.new_position.z = agent.position.z

        for agent in self.agents:
            agent.velocity.x = agent.new_velocity.x
            agent.velocity.y = agent.new_velocity.y
            agent.velocity.z = agent.new_velocity.z

            agent.position.x = agent.new_position.x
            agent.position.y = agent.new_position.y
            agent.position.z = agent.new_position.z

            agent.x_history = agent.x_history[1:]
            agent.x_history.append(agent.position.x)
            agent.y_history = agent.y_history[1:]
            agent.y_history.append(agent.position.y)
            agent.z_history = agent.z_history[1:]
            agent.z_history.append(agent.position.z)

            self.graph_time = self.graph_time[1:]
            self.graph_time.append(self.graph_time[-1] + self.delta_t)

    async def update_graph(self):
        while not self.stopped:
            await asyncio.sleep(self.delta_t)
            for agent in self.agents:
                agent.xy_curve.setData(agent.x_history, agent.y_history)
                agent.xy_marker.setValue(QtCore.QPointF(agent.position.x, agent.position.y))
                agent.z_curve.setData(self.graph_time, agent.z_history)

            self.y_graph.setAxisScale(2, self.graph_time[0], self.graph_time[-1])

            self.y_graph.replot()
            self.xy_graph.replot()


class Coordinates:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


def main():
    agents_list = cf_info.init_agents()

    # -- User interface setup ---------------------------------------------------- #
    app_test = QApplication(sys.argv)
    user_window = main_ui.Window(uavs=agents_list, parameters_filename='flight_parameters.txt')

    # -- Asyncio loop setup ------------------------------------------------------ #
    q_loop = qasync.QEventLoop(app_test)
    asyncio.set_event_loop(q_loop)
    q_loop.run_forever()

    agents = user_window.agents_list

    app = QApplication(sys.argv)
    w = Window(agents)
    q_loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(q_loop)
    asyncio.ensure_future(w.update_graph())
    asyncio.ensure_future(w.timer())
    asyncio.get_event_loop().run_forever()


if __name__ == '__main__':
    main()
