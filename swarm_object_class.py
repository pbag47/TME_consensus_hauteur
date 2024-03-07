import consensus_control_law
import logging
import numpy

from agent_class import Agent
from flight_state_class import FlightState
from typing import List, Union


logger = logging.getLogger(__name__)


class SwarmObject:
    def __init__(self):
        # ---- Parameters ---- #
        # Distance tolerance around a waypoint for it to be considered as reached (m)
        self.distance_to_waypoint_threshold: float = 0.05

        # Obstacle detection distance (m)
        self.xy_auto_avoid_d0: float = 0.85

        # ---- Attributes initialization ---- #
        self.manual_x: float = 0.0
        self.manual_y: float = 0.0
        self.manual_z: float = 0.0
        self.manual_yaw: float = 0.0

        self.ready_to_fly: bool = False
        self.swarm_agent_list: List[Agent] = []
        self.agent_name_list: List[str] = []
        self.manual_flight_agents_list: List[Agent] = []
        self.waypoint_number: Union[None, int] = None

    def add_agent(self, agent: Agent):
        self.swarm_agent_list.append(agent)
        self.agent_name_list.append(agent.name)
        logger.info(agent.name + ' added to the swarm')

    def remove_agent(self, agent: Agent):
        try:
            name_index = self.agent_name_list.index(agent.name)
            _ = self.agent_name_list.pop(name_index)
            __ = self.swarm_agent_list.pop(name_index)
            logger.warning(agent.name + ' has been removed from the swarm')
        except ValueError:
            logger.error('Unable to remove ' + agent.name + ' from the swarm')

    def flight_sequence(self):
        if self.ready_to_fly:
            for agent in self.swarm_agent_list:
                if agent.enabled and not agent.state == FlightState.NOT_FLYING:
                    if agent.state == FlightState.STANDBY:
                        self.standby_control_law(agent)

                    if agent.state == FlightState.TAKEOFF:
                        self.takeoff_control_law(agent)

                    if agent.state == FlightState.LAND:
                        self.landing_control_law(agent)

                    if agent.state == FlightState.MANUAL_FLIGHT:
                        self.manual_control_law(agent)

                    if agent.state == FlightState.XY_CONSENSUS:
                        self.xy_consensus_control_law(agent)

                    if agent.state == FlightState.Z_CONSENSUS:
                        self.z_consensus_control_law(agent)

                    if agent.state == FlightState.BACK_TO_INIT:
                        self.back_to_initial_position_ctl_law(agent)
                else:
                    agent.cf.commander.send_stop_setpoint()

        elif all([(agent.battery_test_passed and agent.position_test_passed) for agent in self.swarm_agent_list]):
            self.ready_to_fly = True
            print('Crazyflie connection recap :')
            for agent in self.swarm_agent_list:
                agent.setup_finished = True
                print('    -', agent.name, ':')
                if agent.position.x >= 0:
                    print('        x = ', round(agent.position.x, 3), 'm')
                else:
                    print('        x =', round(agent.position.x, 3), 'm')
                if agent.position.y >= 0:
                    print('        y = ', round(agent.position.y, 3), 'm')
                else:
                    print('        y =', round(agent.position.y, 3), 'm')
                if agent.position.z >= 0:
                    print('        z = ', round(agent.position.z, 3), 'm')
                else:
                    print('        z =', round(agent.position.z, 3), 'm')
                print('        Battery level =', round(agent.initial_battery_level), '%')
                if agent.enabled:
                    print('        Flight enabled')
                else:
                    print('        -- Warning -- : Flight disabled')

    def manual_control_law(self, agent: Agent):
        kp = 0.45
        ks = 0.15

        agent.standby_position[0] = agent.position.x - numpy.sign(self.manual_x) * kp * numpy.sqrt(abs(self.manual_x))
        agent.standby_position[1] = agent.position.y - numpy.sign(self.manual_y) * kp * numpy.sqrt(abs(self.manual_y))
        agent.standby_position[2] = self.manual_z

        if agent.standby_position[0] < agent.x_boundaries[0] + ks:
            agent.standby_position[0] = agent.x_boundaries[0] + ks
        if agent.standby_position[0] > agent.x_boundaries[1] - ks:
            agent.standby_position[0] = agent.x_boundaries[1] - ks
        if agent.standby_position[1] < agent.y_boundaries[0] + ks:
            agent.standby_position[1] = agent.y_boundaries[0] + ks
        if agent.standby_position[1] > agent.y_boundaries[1] - ks:
            agent.standby_position[1] = agent.y_boundaries[1] - ks
        if agent.standby_position[2] < agent.z_boundaries[0] + ks:
            agent.standby_position[2] = agent.z_boundaries[0] + ks
        if agent.standby_position[2] > agent.z_boundaries[1] - ks:
            agent.standby_position[2] = agent.z_boundaries[1] - ks

        agent.standby_yaw = self.manual_yaw
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   'None', 'None', 'None',
                                   'None', 'None', 'None', 'None'])
        agent.cf.commander.send_position_setpoint(agent.standby_position[0], agent.standby_position[1],
                                                  agent.standby_position[2], agent.standby_yaw)

    def takeoff_control_law(self, agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.takeoff_position[0], agent.takeoff_position[1],
                                                  agent.takeoff_position[2], agent.takeoff_yaw)
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   'None', 'None', 'None',
                                   'None', 'None', 'None', 'None'])
        d = distance([agent.position.x, agent.position.y,
                      agent.position.z], agent.takeoff_position)
        if d <= self.distance_to_waypoint_threshold:
            logger.info(agent.name + ' takeoff completed')
            agent.standby()
            agent.standby_position[2] = agent.takeoff_height

    def landing_control_law(self, agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.land_position[0], agent.land_position[1],
                                                  agent.land_position[2], agent.land_yaw)
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   'None', 'None', 'None',
                                   'None', 'None', 'None', 'None'])
        d = vertical_distance(agent.position.z, agent.land_position[2])
        if d <= self.distance_to_waypoint_threshold:
            logger.info(agent.name + ' landing completed')
            agent.stop()
            self.remove_agent(agent)

    def xy_consensus_control_law(self, agent: Agent):
        in_flight_agents = [agt for agt in self.swarm_agent_list if agt.state != FlightState.NOT_FLYING]
        try:
            roll, pitch = consensus_control_law.xy_consensus_control_law(agent, in_flight_agents)
        except Exception as e:
            logger.error('Error "' + str(e) + '" detected in xy_consensus_control_law function')
            agent.error = e
            agent.standby()
            return

        thrust = thrust_control_law(agent, agent.xy_consensus_z)
        manual_agents = [agt for agt in in_flight_agents if agt.state == FlightState.MANUAL_FLIGHT]
        if manual_agents:
            targeted_yaw = numpy.mean([agt.yaw for agt in manual_agents]) * numpy.pi / 180
        else:
            targeted_yaw = 0
        yaw_rate = yaw_rate_control_law(agent, targeted_yaw)
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   'None', 'None', 'None',
                                   roll, pitch, yaw_rate, thrust])
        agent.cf.commander.send_setpoint(roll, pitch, yaw_rate, thrust)

    def z_consensus_control_law(self, agent: Agent):
        in_flight_agents = [agt for agt in self.swarm_agent_list if agt.state != FlightState.NOT_FLYING]
        vx = agent.z_consensus_xy_position[0] - agent.position.x
        vy = agent.z_consensus_xy_position[1] - agent.position.y
        try:
            vz = consensus_control_law.z_consensus_control_law(agent, in_flight_agents)
        except Exception as e:
            logger.error('Error "' + str(e) + '" detected in z_consensus_control_law function')
            agent.error = e
            agent.standby()
            return

        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   vx, vy, vz,
                                   'None', 'None', 'None', 'None'])
        agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)

    def standby_control_law(self, agent: Agent):
        agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_agents_list]
        objective = [agent.standby_position[0], agent.standby_position[1]]
        omega = numpy.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                    [agent.x_boundaries[1], agent.y_boundaries[1]]))
        vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries, agents_to_avoid,
                                                 objective, omega, self.xy_auto_avoid_d0)
        vz = agent.standby_position[2] - agent.position.z
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   vx, vy, vz,
                                   'None', 'None', 'None', 'None'])
        agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)

    def back_to_initial_position_ctl_law(self, agent):
        agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_agents_list]
        objective = [agent.initial_position[0], agent.initial_position[1]]
        omega = numpy.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                    [agent.x_boundaries[1], agent.y_boundaries[1]]))
        vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries, agents_to_avoid,
                                                 objective, omega, self.xy_auto_avoid_d0)
        vz = agent.takeoff_height - agent.position.z
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   vx, vy, vz,
                                   'None', 'None', 'None', 'None'])
        agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)


def get_auto_avoid_velocity_command(agent: Agent, x_limits: [float] * 2, y_limits: [float] * 2,
                                    agents_to_avoid: List[Agent], objective: [float] * 2, omega: float, d0: float):
    vx = 0.0
    vy = 0.0

    # Agents to avoid
    kpo = 2.5
    kvo = 1
    for agt in agents_to_avoid:
        v = numpy.sqrt(agt.velocity.x ** 2 + agt.velocity.y ** 2)
        if v > 0.25:
            kv2 = (kvo * v) ** 2
            xb = agt.position.x + kvo * agt.velocity.x
            yb = agt.position.y + kvo * agt.velocity.y
            xc = agt.position.x + (((xb - agt.position.x) / (kv2 + 0.001))
                                   * ((xb - agt.position.x) * (agent.position.x - agt.position.x)
                                      + (yb - agt.position.y) * (agent.position.y - agt.position.y)))
            yc = agt.position.y + ((xb - agt.position.x) * (agent.position.x - agt.position.x)
                                   + (yb - agt.position.y) * (agent.position.y - agt.position.y)) / (kv2 + 0.001)

            if ((agt.position.x < xc < xb or xb < xc < agt.position.x)
                    and (agt.position.y < yc < yb or yb < yc < agt.position.y)):
                d = horizontal_distance([agent.position.x, agent.position.y], [xc, yc])
                if d <= d0:
                    vx = vx - (((xc - agent.position.x) / (d + 0.001))
                               * kpo * (numpy.exp(-d) - numpy.exp(-d0)))
                    vy = vy - (((yc - agent.position.y) / (d + 0.001))
                               * kpo * (numpy.exp(-d) - numpy.exp(-d0)))
            else:
                da = horizontal_distance([agent.position.x, agent.position.y], [agt.position.x, agt.position.y])
                db = horizontal_distance([agent.position.x, agent.position.y], [xb, yb])
                if da < db and da <= d0:
                    vx = vx - (((agt.position.x - agent.position.x) / (da + 0.001))
                               * kpo * (numpy.exp(-da) - numpy.exp(-d0)))
                    vy = vy - (((agt.position.y - agent.position.y) / (da + 0.001))
                               * kpo * (numpy.exp(-da) - numpy.exp(-d0)))
                elif db < da and db < d0:
                    vx = vx - (((xb - agent.position.x) / (db + 0.001))
                               * kpo * (numpy.exp(-db) - numpy.exp(-d0)))
                    vy = vy - (((yb - agent.position.y) / (db + 0.001))
                               * kpo * (numpy.exp(-db) - numpy.exp(-d0)))
        else:
            d = horizontal_distance([agent.position.x, agent.position.y], [agt.position.x, agt.position.y])
            if d <= d0:
                vx = vx - (((agt.position.x - agent.position.x) / (d + 0.001))
                           * kpo * (numpy.exp(-d) - numpy.exp(-d0)))
                vy = vy - (((agt.position.y - agent.position.y) / (d + 0.001))
                           * kpo * (numpy.exp(-d) - numpy.exp(-d0)))

    # Objective
    kpg = 1
    distance_to_objective = horizontal_distance([agent.position.x, agent.position.y],
                                                [objective[0],
                                                 objective[1]])
    d1 = numpy.pi / (2 * omega)
    vx = vx + ((objective[0] - agent.position.x) * kpg
               / (2 * d1 * numpy.sqrt((distance_to_objective + 0.001) / d1)))
    vy = vy + ((objective[1] - agent.position.y) * kpg
               / (2 * d1 * numpy.sqrt((distance_to_objective + 0.001) / d1)))

    # Borders
    x_min = x_limits[0] + 0.2 * (x_limits[1] - x_limits[0])
    y_min = y_limits[0] + 0.2 * (y_limits[1] - y_limits[0])
    x_max = x_limits[1] - 0.2 * (x_limits[1] - x_limits[0])
    y_max = y_limits[1] - 0.2 * (y_limits[1] - y_limits[0])
    if agent.position.x < x_min and vx < 0:
        vx = 0
    if agent.position.y < y_min and vy < 0:
        vy = 0
    if agent.position.x > x_max and vx > 0:
        vx = 0
    if agent.position.y > y_max and vy > 0:
        vy = 0

    return vx, vy


def thrust_control_law(agent: Agent, targeted_z: float) -> int:
    """
    Controls the height of a drone by adjusting its thrust via a PID
    """

    z_kp = 32500
    z_ki = 8125
    z_kd = 16250
    thrust_at_steady_state = 38000

    z_error = targeted_z - agent.position.z
    pz = z_kp * z_error
    iz = agent.previous_iz + z_ki * z_error * agent.delta_t
    dz = - z_kd * agent.velocity.z
    thrust = thrust_at_steady_state + pz + iz + dz

    thrust = round(thrust)
    if thrust > 65000:
        thrust = 65000
    elif thrust < 0:
        thrust = 0

    agent.previous_iz = iz
    return thrust


def yaw_rate_control_law(agent: Agent, targeted_yaw: float) -> float:
    yaw_kp = 5
    targeted_yaw = targeted_yaw % (2 * numpy.pi)
    if targeted_yaw > numpy.pi:
        targeted_yaw = targeted_yaw - (2 * numpy.pi)

    measured_yaw = agent.yaw * numpy.pi / 180
    measured_yaw = measured_yaw % (2 * numpy.pi)
    if measured_yaw > numpy.pi:
        measured_yaw = measured_yaw - (2 * numpy.pi)

    yaw_error_0 = targeted_yaw - measured_yaw
    yaw_error_1 = targeted_yaw - measured_yaw + 2 * numpy.pi
    yaw_error_2 = targeted_yaw - measured_yaw - 2 * numpy.pi

    if abs(yaw_error_1) < abs(yaw_error_0) and abs(yaw_error_1) < abs(yaw_error_2):
        yaw_error = yaw_error_1
    elif abs(yaw_error_2) < abs(yaw_error_0) and abs(yaw_error_2) < abs(yaw_error_1):
        yaw_error = yaw_error_2
    else:
        yaw_error = yaw_error_0

    yaw_rate = - round(yaw_kp * yaw_error * 180 / numpy.pi)        # (°/s)

    max_yaw_rate = 180  # (°/s)
    if yaw_rate > max_yaw_rate:
        yaw_rate = max_yaw_rate
    elif yaw_rate < -max_yaw_rate:
        yaw_rate = -max_yaw_rate
    return yaw_rate


def distance(position_1_xyz_list: List[float], position_2_xyz_list: List[float]):
    d = numpy.sqrt((position_1_xyz_list[0] - position_2_xyz_list[0]) ** 2
                   + (position_1_xyz_list[1] - position_2_xyz_list[1]) ** 2
                   + (position_1_xyz_list[2] - position_2_xyz_list[2]) ** 2)
    return d


def vertical_distance(position_1_z: float, position_2_z: float):
    vd = numpy.sqrt((position_1_z - position_2_z) ** 2)
    return vd


def horizontal_distance(position_1_xy: List[float], position_2_xy: List[float]):
    hd = numpy.sqrt((position_1_xy[0] - position_2_xy[0]) ** 2
                    + (position_1_xy[1] - position_2_xy[1]) ** 2)
    return hd
