import array
import joystick_map
import logging
import os
import struct

from fcntl import ioctl
from threading import Thread
from typing import Union

from swarm_object_class import SwarmObject
from flight_state_class import FlightState


logger = logging.getLogger(__name__)


class Joystick:
    def __init__(self, swarm_object: Union[SwarmObject, None] = None):
        self.swarm = swarm_object
        self.stopped = False

        logger.debug('Detected input devices : ' + str([('/dev/input/%s' % fn)
                                                       for fn in os.listdir('/dev/input') if fn.startswith('js')]))

        # Buttons and axes states initialization
        self.axis_states = {}
        self.button_states = {}

        self.axis_map = []
        self.button_map = []

        # Opens the Joystick device
        self.fn = '/dev/input/js0'
        logger.debug('Opening %s:' % self.fn)
        try:
            self.js_device = open(self.fn, 'rb')
        except FileNotFoundError as e:
            logger.error('No controller / joystick found')
            raise e

        # Gets the device name
        buf = array.array('B', [0] * 64)
        ioctl(self.js_device, 0x80006a13 + (0x10000 * len(buf)), buf)
        self.js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
        logger.debug('Device name: %s' % self.js_name)

        # Gets the number of axes and buttons
        buf = array.array('B', [0])
        ioctl(self.js_device, 0x80016a11, buf)
        self.num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self.js_device, 0x80016a12, buf)
        self.num_buttons = buf[0]

        # Gets the buttons and axes names from the corresponding Joystick map data
        self.button_names, self.axis_names = joystick_map.joystick_map(self.js_name)

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.js_device, 0x80406a32, buf)
        for axis in buf[:self.num_axes]:
            axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.js_device, 0x80406a34, buf)
        for btn in buf[:self.num_buttons]:
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0.0

        logger.debug('%d axes found: %s' % (self.num_axes, ', '.join(self.axis_map)))
        logger.debug('%d buttons found: %s' % (self.num_buttons, ', '.join(self.button_map)))

        if self.swarm is not None:
            self.jsl = Thread(target=self.joystick_inputs)
            self.jsl.start()

    def joystick_inputs(self):
        while not self.stopped:
            ev_buf = self.js_device.read(8)
            time, value, buf_type, number = struct.unpack('IhBB', ev_buf)

            if buf_type & 0x01:
                button = self.button_map[number]
                if button:
                    self.buttons(button, value)

            if buf_type & 0x02:
                axis = self.axis_map[number]
                if axis:
                    self.axis(axis, value)

    def buttons(self, button, value):
        if button == 'Stop' and value:
            logger.info('Stop button triggered, joystick disconnected')
            for agt in self.swarm.swarm_agent_list:
                agt.cf.commander.send_stop_setpoint()
                agt.stop()
            self.stopped = True

        if button == 'Takeoff/Land' and value:
            logger.info('Takeoff / Land button triggered')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and agt.state == FlightState.NOT_FLYING:
                    agt.takeoff()
                elif agt.enabled:
                    agt.land()

        if button == 'Standby' and value:
            logger.info('Standby button triggered')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and not agt.state == FlightState.NOT_FLYING:
                    agt.standby()

        if button == 'Manual_flight' and value:
            logger.info('Manual flight button triggered')
            if any([agt.state == FlightState.TAKEOFF
                    or agt.state == FlightState.LAND for agt in self.swarm.swarm_agent_list]):
                logger.warning('Manual control mode is prohibited while any UAV is in '
                               'Takeoff or Landing flight mode')
                logger.error('Manual control command access denied')
            else:
                for agt in self.swarm.swarm_agent_list:
                    if agt.enabled and \
                            not agt.state == FlightState.NOT_FLYING and \
                            any([agt == manual for manual in
                                 self.swarm.manual_flight_agents_list]):
                        self.swarm.manual_z = agt.position.z
                        agt.manual_flight()

        if button == 'Yaw-' and value:
            self.swarm.manual_yaw = self.swarm.manual_yaw + 22.5

        if button == 'Yaw+' and value:
            self.swarm.manual_yaw = self.swarm.manual_yaw - 22.5

        if button == 'Initial_position' and value:
            logger.info('Initial position button triggered')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and not agt.state == FlightState.NOT_FLYING:
                    agt.back_to_initial_position()

        if button == 'z_consensus' and value:
            logger.info('z consensus button triggered')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and not agt.state == FlightState.NOT_FLYING:
                    agt.z_consensus()

        if button == 'xy_consensus' and value:
            logger.info('xy consensus button triggered')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and not agt.state == FlightState.NOT_FLYING:
                    agt.xy_consensus()

    def axis(self, axis, value):
        fvalue = value / 32767.0

        if axis == 'pitch':
            if -0.01 < fvalue < 0.01:
                self.swarm.manual_x = 0.0
            else:
                self.swarm.manual_x = fvalue ** 3

        if axis == 'roll':
            if -0.01 < fvalue < 0.01:
                self.swarm.manual_y = 0.0
            else:
                self.swarm.manual_y = fvalue ** 3

        if axis == 'yaw':
            self.swarm.manual_yaw = self.swarm.manual_yaw - 2 * fvalue ** 3

        if axis == 'yaw-':
            fvalue = (1 + fvalue) / 2
            self.swarm.manual_yaw = self.swarm.manual_yaw + 2 * fvalue ** 3

        if axis == 'yaw+':
            fvalue = (1 + fvalue) / 2
            self.swarm.manual_yaw = self.swarm.manual_yaw - 2 * fvalue ** 3

        if axis == 'height':
            self.swarm.manual_z = (1 - fvalue) / 2

        if axis == 'height2':
            self.swarm.manual_z = self.swarm.manual_z - 0.01 * fvalue

    def print_inputs(self):
        """
        Test method that prints any input received from the Joystick
        """
        while True:
            ev_buf = self.js_device.read(8)
            time, value, buf_type, number = struct.unpack('IhBB', ev_buf)

            if buf_type & 0x01:
                button = self.button_map[number]
                if button:
                    print([button, value])

            if buf_type & 0x02:
                axis = self.axis_map[number]
                if axis:
                    print([axis, value])


if __name__ == '__main__':
    js = Joystick()
    js.print_inputs()
