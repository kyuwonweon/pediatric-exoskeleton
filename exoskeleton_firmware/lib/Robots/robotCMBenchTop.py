import pyb                                          # type: ignore
import json

# from lib.Hardware.imu_streamer import ImuStreamer
from lib.Hardware.servoMotorCM import ServoCM
import micropython                                  # type: ignore

from micropython import const                       # type: ignore

from lib.Robots.RobotInterface import Robot
import time

from lib.ll_common.error_flag import print_error

### const for Robot
_GRAVITY= const(9.81)



class CMBenchTop(Robot):
    def __init__(self, robot_file,
                 robot_name,
                 device_parameters
                 ):
        super().__init__(robot_file, robot_name,
                         device_parameters)

        # Servo params specifics of Novanta (all other params are defined in the super class)
        self.Kpp = 0.0
        self.Kpv = 0.0
        self.Kpt = 0.0

        self.KNEE_JOINT_ENC_COEF = 360 / 2**12
        self.TORQUE_CONSTANT_LARGE_MOTOR = 0.13  # Nm/A
        self.KE_LARGE_MOTOR = 0.75    # V/(rev/s), from Ke=12.5 V/krpm
        self.MOTOR_RESISTANCE = 0.577             # Ohm
        # Feedforward gain: I_ff = BACKEMF_FF_GAIN * velocity [A / (rev/s)]
        # TORQUE_LOOP_INPUT_OFFSET takes Amps. Theoretical Ke/R = 1.3 but empirically
        # tuned to 1.0 — above ~1.3 the motor becomes self-propelling (negative damping).
        self.BACKEMF_FF_GAIN = 1.0

        self.K =0.0
        self.B =0.0

        self.position_limits =[-90, 90]

        self.pos_incr_encoder = 0
        self.offset_incr_encoder = 0

        self.transmission = 8

        self.reference_position = 0.0
        self._backemf_ff_enabled = False
        self._tau_backemf_ff = 0.0
        self._tau_external = 0.0
        self._tau_commanded = 0.0
        self._q_des_test = 0.0

        # servo
        current_limit = self.params["servo"]["CURRENT_LIMIT"]  # TODO: move in servo
        self._initialize_servo(current_limit)
        # update timer
        self._start_timer()



    ###########################
    ### PUBLIC METHODS
    ### GETTER FUNCTIONS ###
    def get_theta_des(self):
        return self.servo.position_commanded * self.KNEE_JOINT_ENC_COEF
    def get_position_incr_encoder(self):
        return  self.pos_incr_encoder + self.offset_incr_encoder
    def get_backemf(self):
        return self.KE_LARGE_MOTOR * self.servo.velocity
    def get_torque_des(self):
        return self.servo.torque_commanded
    def get_torque_act(self):
        return self.servo.actual_torque
    def get_current(self):
        return self.servo.get_current()
    def get_tempetature(self):
        return self.servo.get_temperature()


    def create_report(self, out, index) -> int:
        new_index = super().create_report(out, index)

        # all robot report has dimension defined in params file
        # first three are filled in the interface and dedicated to:
        # 0 - position
        # 1 - velocity
        # 2 - acceleration
        # ---- locally defined ----
        # 3 - loadcell Fz
        # 4 - loadcell My
        # 5 - thigh positionll
        out[new_index]     = self._tau_commanded              # total offset commanded to drive [Nm]
        out[new_index + 1] = self.get_torque_act()           # actual torque from drive
        out[new_index + 2] = self.get_position_incr_encoder()
        out[new_index + 3] = self._tau_backemf_ff            # feedforward contribution
        # current and temperature
        out[new_index +4] = self.get_current()
        out[new_index +5] = self.get_tempetature()
        out[new_index + 6] = self.get_backemf()
        out[new_index + 7] = self._q_des_test               # commanded position (set by test script)
        return index + self.rep_robot_msg_dim

    #### INITIALIZE FUNCTIONS

    def enable(self, active):
        self.servo.enable(active)
        pyb.delay(1000)
        # self.servo.change_to_Position_profile()
        pyb.delay(200)
        self.set_position_set_point(self.get_position())
        pyb.delay(200)

        # default set the servo in PVT mode
        # FOR the moment default is PVT profile
        self.servo.change_PVT_profile()
        self.run = True

        self.offset_incr_encoder = (self.servo.position * self.KNEE_JOINT_ENC_COEF + 180) % 360 - 180

    #########################
    #### setter functions
    def change_to_Position_profile(self):
        if self.servo.get_profiler() == "PVT":
            self.servo.change_to_Position_profile()
            self.set_position_set_point(self.get_position())
        elif self.servo.get_profiler() == "P":
            print("Robot already in position profiler")
        else:
            raise ValueError("Profiler not recognized")

    def change_to_PVT_profiler(self):
        if self.servo.get_profiler() == "P":
            self.servo.change_PVT_profile()
        elif self.servo.get_profiler() == "PVT":
            print("Robot already in PVT profiler")
        else:
            raise ValueError("Profiler not recognized")

    def set_position_set_point(self, position):
        if self.servo.get_profiler() == "P":
            # bound positions
            position = max(position, self.position_limits[0])
            position = min(position, self.position_limits[1])

            # position should be in degs
            position = int(position / self.KNEE_JOINT_ENC_COEF)
            self.servo.set_pos_loop_set_point(position)
        else:
            print("The current profiler doesn't support this method!")

    def set_transition_point(self, pos, vel = 0.0, dt = 1.0):
        """
        THis function set a position in the servo (specific for PVT profiler)
        Args:
            pos (float): The angle in degrees.
            vel (float): The velocity in degs/s. This is the arriving velocity. By default is zero.
            dt (float): The time in seconds. For interpolation profiler this is not used.
                        The one used for this mode is the product between the exp and the mantissa one.
        Returns:
            None
        """
        pos = max(pos, self.position_limits[0])
        pos = min(pos, self.position_limits[1])
        if self.servo.get_profiler() == "PVT":
            pos = int(pos / self.KNEE_JOINT_ENC_COEF)
            # Convert degs/s to rev/s
            vel = vel/360
            self.servo.append_execute_buffer(pos, vel, dt)
        else:
            print("The current profiler doesn't support this method!")


    def set_tau_offset(self, tau=0.0):
        self._tau_external = tau

    def set_backemf_feedforward(self, enabled: bool):
        self._backemf_ff_enabled = enabled

    def set_KD_parameter(self, K: float, B: float) -> None:
        """
        Function to set the stiffness and damping parameter.
            :param K: Stiffness coefficient. The value must be in the range [0, 2]. If value exceeds the range the bound value will be applied.
            :param B: Damping coefficient. The value must be in the range [0, 0.1]. If value exceeds the range the bound value will be applied.
            :return: None
        """
        if K < self.K_bounds[0] or K > self.K_bounds[1]:
            print(f'KD parameter must be in the range [{self.K_bounds[0]}, {self.K_bounds[1]}]. Buonded value applied')
            K = min(max(self.K_bounds[0], K), self.K_bounds[1])
        if B < self.B_bounds[0] or B > self.B_bounds[1]:
            print(f'Damping parameter must be in the range [{self.B_bounds[0]}, {self.B_bounds[1]}]. Buonded value applied')
            B = min(max(self.B_bounds[0], B), self.B_bounds[1])
        print('K = ', K)
        print('D = ', B)
        self.K = K
        self.B = B


    #####################
    ### PRIVATE

    ## Initialize functions
    def _initialize_servo(self, current_limit):
        self.servo = ServoCM(CURRENT_LIMIT=current_limit,
                             device_parameters=self.device_parameters)
    # update functions
    def _update_position(self, static =False):
        # function that return and update the value of position
        if static:
            pos = self.servo.position(static) * self.KNEE_JOINT_ENC_COEF
            # TODO: this might not work in config mode
            pos_incr_enc = self.servo.position_incremental_encoder* self.KNEE_JOINT_ENC_COEF / self.transmission
        else:
            pos = self.servo.position * self.KNEE_JOINT_ENC_COEF
            pos_incr_enc = self.servo.position_incremental_encoder* self.KNEE_JOINT_ENC_COEF / self.transmission
        self.pos = (pos + 180) % 360 - 180
        self.pos_incr_encoder = (pos_incr_enc + 180) % 360 - 180
    def _update_velocity(self):
        # function that update and return the value of vel
        self.vel = self.servo.velocity * 360.0
    def _update_acceleration(self):
        self.acc = self.servo.acceleration * 360.0
    def _update_K(self):
        if self.B > 0.3:
            self.B = 0.3
        # self.Kpv = 360.0 * self.B / self.n  # motor velocity
        self.Kpv = 360.0 * self.B  # joint velocity
        # Avoid division by zero when Kpv is zero
        if self.Kpv == 0.0:
            self.Kpp = 0.0
        # The normal controller case:
        else:
            self.Kpp = self.K * self.KNEE_JOINT_ENC_COEF / self.Kpv
        self.Kpt = (
                1.0 / (self.transmission * self.TORQUE_CONSTANT_LARGE_MOTOR)
        )
        self.servo.set_Kpp(self.Kpp)
        self.servo.set_Kpv(self.Kpv)
        self.servo.set_Kpt(self.Kpt)


    def _compute_backemf_feedforward(self):
        if self._backemf_ff_enabled and self.BACKEMF_FF_GAIN > 0.0:
            self._tau_backemf_ff = self.BACKEMF_FF_GAIN * self.servo.velocity  # [Nm] at motor shaft
        else:
            self._tau_backemf_ff = 0.0
        self._tau_commanded = self._tau_external + self._tau_backemf_ff
        self.servo.set_tau_offset(self._tau_commanded)

    def _update_callback(self, timer_object):
        if self.run:
            # update kin
            self._update_position()
            self._update_velocity()
            self._update_acceleration()
            self._compute_transmission()
            self._update_K()
            self._compute_backemf_feedforward()




