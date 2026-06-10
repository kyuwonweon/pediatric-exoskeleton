import pyb                                          # type: ignore

import micropython                                   # type: ignore
from micropython import const                       # type: ignore
from lib.ll_common.const_dict import CONST_DICT as const_dict
from lib.ll_common.novanta_constants import NovantaConstants as NOV
from lib.ll_common import upy_spi
from lib.ll_common import nvic
from lib.ll_common import novanta


class ServoInterface: # servo constructor/decorator
    def __init__(self, device_parameters: DeviceParameters):
        self.device_parameters = device_parameters

        self.profiler = "P" # default servo behavior is position profiler

        self.pos =0
        self.pos_des =0
        self.vel =0
        self.tau_des= 0
        self.tau_actual = 0
        self.current =0
        self.temperature =0
        self.counter_for_temp =0 # reading temp each instance can be heavy so one for sec
        self.run = False
        self._busy = False

        self.reference_position = 0

        # pre-allocate bound method to avoid heap allocation inside the ISR
        self._servo_work_ref = self._servo_work

        # computation of the accelleration
        self.prev_vel = 0.0
        self.acc = 0.0
        self.dt = 1 / self.device_parameters.TIMER_SERVO['FREQ']

    def _init_spi(self):
        self.spi = upy_spi.upySPI(
        # self.spi = pyb.SPI(
            self._NOVANTA_SPI_BUS
        )
        self.spi.init(
            mode=pyb.SPI.CONTROLLER,
            baudrate=self._NOVANTA_SPI_BAUDRATE,
            # prescaler=self._NOVANTA_SPI_PRESCALER,
            polarity=0,
            phase=1,
            bits=8,
            #   crc=0x1021,
            firstbit=pyb.SPI.MSB
        )
        print(self.spi)

    def _init_everest(self):
        self._servo = novanta.Everest(
            self.spi,
            self._NOVANTA_RESET_PIN,
            self._NOVANTA_BOOT_SYNC1_PIN,
            self._NOVANTA_SPI_SELECT_PIN,
            self._NOVANTA_INTERRUPT_PIN,
            cyclic_pico_registers=self.cyclic_pico_registers,
            cyclic_poci_registers=self.cyclic_poci_registers,
            config_dict=self.config_dict,
            use_hardware_crc=True,
            # use_hardware_crc=False,
            verbose=True
        )

        nvic.nvic_set_prio(self._servo.interrupt_pin.line() + 6, 5 - 1)

        if self._servo.read_int(NOV.ACTUAL_POSITION) > 2048:
            print('=============================')
            print('WARNING: JOINT ENCODER > 2048')
            print('=============================')

    def _init_timer(self):
        self.timer_servo = pyb.Timer(
            self.device_parameters.TIMER_SERVO['ID'],
            freq=self.device_parameters.TIMER_SERVO['FREQ'],
            callback=self._servo_callback
        )

    @property
    def position(self, static=False):
        if static:
            return self._servo.read_uint(NOV.ACTUAL_POSITION)
        else:
            return self.pos
    @property
    def position_commanded(self):
        return self.pos_des
    @property
    def torque_commanded(self):
        return self.tau_des
    @property
    def actual_torque(self):
        return self.tau_actual
    @property
    def velocity(self):
        return self.vel
    @property
    def acceleration(self):
        return self.acc
    
    def get_current(self):
        return self.current
    def get_temperature(self):
        return self.temperature

    def get_profiler(self):
        return self.profiler

    # this method is specific by which encoders are used so should be overwritten
    def enable(self, active):
        pass

    # this method is specific by which encoders are used so should be overwritten
    def zero(self):
        pass

    def change_PVT_profile(self):
        """
        Function to change to PVT profile.
        Position velocity time (PVT) behaves in a similar fashion to IP.
        In this mode, the Set-point manager and interpolation buffer allow the user to provide the drive with sets of
        final position, velocity, and movement duration. Interpolation time mantissa and magnitude order will
        not be necessary for this mode because interpolation time will be extracted from the time introduced into the
        buffer of the set-point manager directly. These sets of points are then fed to the set-point manager,
        which will use the PVT profiler to generate 5th-degree polynomial trajectories that reach each position
        set-point at the specified velocity in the given time duration.
        Returns:
            None
        """
        self.profiler = "PVT"

        # clear the buffer
        # TODO: check why stucked
        # try:
        #     self._servo.write_uint(NOV.INTERPOLATION_DATA_RECORD_FORCE_CLEAR, 1)
        #     pyb.delay(10)
        # except Exception as e:
        #     print("Warning: could not clear buffer:", e)
        self._servo.channel_pico.control_word = 0
        pyb.delay(30)
        self._servo.channel_pico.control_word = 6  # check exa
        pyb.delay(20)
        self._servo.channel_pico.control_word = 7  # check exa
        pyb.delay(20)
        #Set the Operation mode to PROFILER_BUFFERED_PVT Profile
        self._servo.channel_pico.operation_mode = NOV.PROFILER_BUFFERED_PVT #0xB4
        pyb.delay(20)
        self._servo.channel_pico.control_word = 15  # check exa
        pyb.delay(20)
        print("Changed OPERATION_MODE to PVT profiler")

    def change_to_Position_profile(self):
        """
        Function to profiler to position profile.
        """
        self._servo.channel_pico.control_word = 0
        pyb.delay(30)
        self._servo.channel_pico.control_word = 6
        pyb.delay(20)
        self._servo.channel_pico.control_word = 7
        pyb.delay(20)
        self._servo.channel_pico.operation_mode = NOV.OPERATION_MODE_POSITION  # Position Profile
        pyb.delay(20)
        self._servo.channel_pico.control_word = 15

        self.profiler = "P"
        print("Drive is now in POSITION_PROFILE mode.")

    def clear_buffer(self):

        self._servo.channel_pico.control_word = 0
        pyb.delay(30)
        self._servo.channel_pico.control_word = 6  # check exa
        pyb.delay(20)
        self._servo.channel_pico.control_word = 7  # check exa
        pyb.delay(20)
        # clear the buffer
        try:
            self._servo.write_uint(NOV.INTERPOLATION_DATA_RECORD_FORCE_CLEAR, 1)
            pyb.delay(10)
        except Exception as e:
            print("Warning: could not clear buffer:", e)

        self._servo.channel_pico.control_word = 15  # check exa
        pyb.delay(20)

    def set_pos_loop_set_point(self, position : int):
        """This is uded only in full position mode.
        Double check what it is the current servo profiler"""
        self.reference_position = position

    def append_execute_buffer(self, angle: int, vel= 0.0, dt = 1) -> None:
        """
        Append element to the buffer and update the control word to execute it
        Args:
            angle(int): The angle in counts.
            vel (float): The velocity in revs/s. This is the arriving velocity. By default is zero.
            dt (float): The time in seconds. For interpolation profiler this is not used.
                        The one used for this mode is the product between the exp and the mantissa one.
        Returns:
            None
        """
        # Write the required input:
        # 0x20C1 - Interpolation data record - Position input
        # 0x20C2 - Interpolation data record - Velocity input
        # 0x20C3 - Interpolation data record - Time input  --> this is not used in the interpolated position mode
        #                                                   and replaced by the one defined by the time mantissa and exponent
        self._servo.channel_pico.interpolation_data_record_position_input = angle
        self._servo.channel_pico.interpolation_data_record_velocity_input = vel
        self._servo.channel_pico.interpolation_data_record_time_input = max(1,  int(dt*1000))
        # Increment the integrity check value by 1 (0x20C4 - Interpolation data record integrity check).
        #  When the drive detects an increment of 1 in the integrity check,
        #  it will save the configured values in the position, velocity, and time inputs.
        self._servo.channel_pico.interpolation_data_record_integrity_check += 1
        if not self._servo.channel_pico.control_word == 31:
            pyb.delay(1)
            self._servo.channel_pico.control_word = 31

    def _update_position(self):
        self.pos = self._servo.channel_poci.actual_position
    def _update_pos_desired(self):
        self.pos_des = self._servo.channel_poci.position_loop_control_command
    def _update_torque_commanded(self):
        self.tau_des = self._servo.channel_poci.torque_loop_control_command
    def _update_actual_torque(self):
        self.tau_actual =self._servo.channel_poci.actual_torque
    def _update_velocity(self):
        self.prev_vel = self.vel
        self.vel = self._servo.channel_poci.actual_velocity
    def _update_acceleration(self):
        prev_acc = self.acc
        curr_acc = (self.vel - self.prev_vel) / self.dt
        alpha =0.05
        self.acc = alpha * curr_acc + (1 - alpha) * prev_acc
    def _update_current_temp(self):
        self.current =  self._servo.channel_poci.current_quadrature_value
        self.counter_for_temp += 1
        if self.counter_for_temp==300:
            self.temperature = self._servo.channel_poci.motor_temperature_value
            self.counter_for_temp =0
    def _set_pos_loop_set_point(self):
        self._servo.channel_pico.position_set_point = self.reference_position

    def _servo_callback(self, _):
        if not self._busy:
            self._busy = True
            micropython.schedule(self._servo_work_ref, 0)

    def _servo_work(self, _):
        self._busy = False
        if self.run:
            self._servo.send_receive_cyclic()
            self._update_position()
            self._update_pos_desired()
            self._update_actual_torque()
            self._update_torque_commanded()
            self._update_velocity()
            self._update_acceleration()
            self._update_current_temp()
            self._set_pos_loop_set_point()
            self._servo.send_receive_cyclic()


    def set_tau_offset(self, tau=0.0):
        self._servo.channel_pico.torque_loop_input_offset = tau

    def set_Kpp(self, Kpp):
        # position loop K
        self._servo.channel_pico.position_loop_kp = Kpp
    
    def set_Kpd(self, Kpd):
        # position loop derivative component
        self._servo.channel_pico.position_loop_kd = Kpd

    def set_Kpv(self, Kpv):
        # velocity loop K
        self._servo.channel_pico.velocity_loop_kp = Kpv

    def set_Kpt(self, Kpt):
        # torque loop K
        self._servo.channel_pico.torque_loop_kp = Kpt