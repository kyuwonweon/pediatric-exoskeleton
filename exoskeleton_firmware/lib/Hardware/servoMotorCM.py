import pyb                                          # type: ignore

from micropython import const                       # type: ignore
from lib.ll_common.const_dict import CONST_DICT as const_dict
from lib.ll_common.novanta_constants import NovantaConstants as NOV
from lib.Hardware.servoMotorInterface import ServoInterface
from lib.ll_common import upy_spi
from lib.ll_common import nvic
from lib.ll_common import novanta

from lib import config
from lib.ll_common import mode_switch


### const for Servo
BOARD_EVEREST_BREAKOUT = 0
BOARD_LL_CONTROL_BOARD_VB_1_1 = 1   # Pyboard SF6
BOARD_PORTENTA_ADAPTER = 2          # Portenta H7 adapter for VB_1_1
BOARD_LL_CONTROL_BOARD_VB_2_0 = 3   # Portenta H7
BOARD_LL_CONTROL_BOARD_VB_3_0 = 4   # Portenta H7 Fox Knee

DEVICE_HYBRID_KNEE = 0
DEVICE_FOX_KNEE = 1
DEVICE_CUBE_MARS_BENCHTOP = 2

class ServoCM(ServoInterface): # servo constructor/decorator
    def __init__(self, CURRENT_LIMIT, device_parameters: DeviceParameters):
        
        super().__init__(device_parameters = device_parameters)
        print("Using SERVO CM")
        # -----------------------------------------------------------------------------
        #   DEFINE BOARD-SPECIFIC CONSTANTS
        # -----------------------------------------------------------------------------
        #   For control board vB_2_0:
        print('Using ll-control-board vB_2_0...')
        self._NOVANTA_SPI_BUS = 2
        # Everest max is 50 MHz. 50 MHz is prescaler of 8 for Portenta H7
        self._NOVANTA_SPI_BAUDRATE = 50000000
        # self._NOVANTA_SPI_PRESCALER      = 8

        self._NOVANTA_SPI_SELECT_PIN = 'J2_36'  # SPI1-SS1 I0
        self._NOVANTA_INTERRUPT_PIN = 'J2_54'  # E3

        self._NOVANTA_BOOT_SYNC1_PIN = 'J2_22'  # A4
        self._NOVANTA_RESET_PIN = 'J2_52'

        sto_1 = pyb.Pin('J2_68', mode=pyb.Pin.OUT, pull=pyb.Pin.PULL_UP)
        dbb_enable = pyb.Pin('J2_65', mode=pyb.Pin.OUT, pull=pyb.Pin.PULL_UP)
        sto_1.value(1)
        dbb_enable.value(0)

        #
        #   INITIALIZE SPI
        #
        self._init_spi()

        #
        #   INITIALIZE EVEREST
        #
        self.config_dict = {
            NOV.POSITION_LOOP_KP: 0.0,                          # Control parameter
            NOV.POSITION_LOOP_KI: 0.0,
            NOV.POSITION_LOOP_KD: 0.0,
            NOV.POSITION_LOOP_KFFA: 0.0,
            NOV.POSITION_LOOP_KFFV: 0.0,
            NOV.POSITION_LOOP_KD_FILTER: 0.0,
            NOV.POSITION_FOLLOWING_ERROR_OPTION_CODE: 1,
            NOV.POSITION_FEEDBACK_OUT_OF_LIMITS_ERROR_OPTION_CODE: 1,     # Needed?
            NOV.POSITION_LOOP_MIN_OUTPUT: -200.0,             # Default is -200.0
            NOV.POSITION_LOOP_MAX_OUTPUT: 200.0,              # Default is 200.0

            NOV.VELOCITY_LOOP_KP: 0.0,                          # Control parameter
            NOV.VELOCITY_LOOP_KI: 0.0,
            NOV.VELOCITY_LOOP_KD: 0.0,
            NOV.VELOCITY_LOOP_KFFA: 0.0,
            NOV.VELOCITY_LOOP_KD_FILTER: 0.0,
            NOV.VELOCITY_FOLLOWING_ERROR_OPTION_CODE: 1,
            NOV.VELOCITY_FEEDBACK_OUT_OF_LIMITS_ERROR_OPTION_CODE: 1,     # Needed?
            NOV.VELOCITY_LOOP_MIN_OUTPUT: -200.0,
            NOV.VELOCITY_LOOP_MAX_OUTPUT: 200.0,

            NOV.TORQUE_LOOP_KP: 0.0,                            # Control parameter
            NOV.TORQUE_LOOP_KI: 0.0,

            # Controls the frequency of the power stage, as well as position
            # and velocity control loops. The latter may be read via
            # NOV.POSITION_AND_VELOCITY_CONTROL_LOOP_RATE, which work out as:
            # 0 = 10 kHz
            # 1 = 20 kHz
            # 2 = 25 kHz
            NOV.POWER_STAGE_FREQUENCY: 1,

            # NOV.CONTROL_LOOPS_FEEDBACK_OPTIONS: 19,        # normally 27. vel feedback disabled = 19

            NOV.OPERATION_MODE: NOV.OPERATION_MODE_POSITION,
            # NOV.OPERATION_MODE: NOV.PROFILER_CYCLIC_SYNCHRONOUS_POSITION,
            # NOV.INTERPOLATION_TIME_MANTISSA: 3.0,
            # NOV.INTERPOLATION_TIME_EXPONENT: 1.0,

            NOV.POSITIONING_OPTION_CODE: 3 << 6,            # 0 is default, 3 is shortest to target
            # NOV.POSITIONING_OPTION_CODE: 0,

            #
            #   SET ERROR OPTION CODE FOR STO
            #
            # This allows for the servo to tolerate the sto lines toggling off
            # when in the SWITCHED_ON or OPERATION_ENABLED state. If STO is
            # toggled low in these modes, the servo will go to the FAULT state.
            # However, by setting this register to 1, performing transition #15
            # will work to recover from the fault state. If this option code is
            # left at it's default value of 0, recovering from this type of
            # fault requires power-cycling the servo.
            NOV.STO_ERROR_OPTION_CODE: 1,

            #
            #   SET FEEDBACK RUNAWAY ERROR OPTION CODE
            #
            #   This masks faults arising from missed encoder or hall-effect
            #   counts. Uncomment the next line line if this error begins
            #   occurring due to excessive signal noise or motor speed. Default
            #   is 0
            # NOV.FEEDBACK_RUNAWAY_ERROR_OPTION_CODE: 1,            # Default is 0

            #
            #   LIMIT CURRENT COMMANDS WHEN DEBUGGING
            #
            NOV.CURRENT_QUADRATURE_LOOP_MIN_OUT: -1.0 * CURRENT_LIMIT,
            NOV.CURRENT_QUADRATURE_LOOP_MAX_OUT: CURRENT_LIMIT,
            NOV.MAX_CURRENT: CURRENT_LIMIT,

            #
            #   I2T CURRENT PROTECTION
            #

            # 0=disable power stage, 1=do nothing, 2=slow down ramp, 3=quick stop ramp
            NOV.USER_I2T_ERROR_OPTION_CODE: 1,
            NOV.PEAK_CURRENT: CURRENT_LIMIT,  # Amps
            NOV.PEAK_CURRENT_TIME: 1_000_000,  # ms
        }


        self.config_dict.update({
                NOV.POSITION_FEEDBACK_SENSOR: NOV.FEEDBACK_PRIMARY_ABSOLUTE_SLAVE_1,
                NOV.PRIMARY_ABSOLUTE_SLAVE_1_POLARITY: 1, # TODO check current direction
                NOV.MIN_POSITION_RANGE_LIMIT: 0,
                NOV.MAX_POSITION_RANGE_LIMIT: 2 ** 12,
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_PROTOCOL: 0x0,  # 0 = Biss-C # default 1 --> SSI
                NOV.PRIMARY_ABSOLUTE_SLAVE_1_FRAME_SIZE: 25,
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_BAUDRATE: 1000,  # kHz
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_CRC_POLYNOMIAL: 0x43,
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_CRC_NUMBER_OF_BITS: 6,
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_CRC_SEED: 0x0,
                NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION_BITS: 12,
                NOV.PRIMARY_ABSOLUTE_SLAVE_1_SINGLE_TURN_BITS: 12,
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION_START_BIT: 1,
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_FRAME_TYPE: 0,  # 0=raw, 3=BiSS-C BP3, 4=BiSS-C BP3 gray
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_ERROR_TOLERANCE: 0,  # Errors are ignored
                NOV.POSITION_WINDOW: 13,  # counts (default is 0)
                NOV.POSITION_WINDOW_TIME: 0,  # ms (int) - default 0

                NOV.VELOCITY_FEEDBACK_SENSOR: NOV.FEEDBACK_PRIMARY_ABSOLUTE_SLAVE_1,
                NOV.VELOCITY_THRESHOLD: 1.0 / 360.0,  # rev/s default=1.0
                NOV.VELOCITY_THRESHOLD_TIME: 0,

                NOV.VELOCITY_FEEDBACK_FILTER_1_TYPE: 1,  # Configured 1
                NOV.VELOCITY_FEEDBACK_FILTER_1_FREQUENCY: 120, #200,  # Configured 200
                NOV.VELOCITY_FEEDBACK_FILTER_1_Q_FACTOR: 0.7071,
                NOV.VELOCITY_FEEDBACK_FILTER_1_GAIN: 0.0,

                NOV.VELOCITY_REFERENCE_FILTER_1_TYPE: 1,
                NOV.VELOCITY_REFERENCE_FILTER_1_FREQUENCY: 200,
                NOV.VELOCITY_REFERENCE_FILTER_1_Q_FACTOR: 0.7071,
                NOV.VELOCITY_REFERENCE_FILTER_1_GAIN: 0.0,

                NOV.AUXILIAR_FEEDBACK_SENSOR: NOV.FEEDBACK_INCREMENTAL_ENCODER_1,
                NOV.INCREMENTAL_ENCODER_1_RESOLUTION: 4096,
            })
        self.cyclic_poci_registers=[ # ATTENTION: Max number of registers supported is 15
                NOV.ACTUAL_VELOCITY,                                # (4 bytes)
                NOV.ACTUAL_POSITION,
                NOV.CURRENT_QUADRATURE_A_CONTROL_LOOP_COMMAND,      # (4 bytes)
                NOV.CURRENT_QUADRATURE_VALUE,                       # (4 bytes)
                NOV.MOTOR_TEMPERATURE_VALUE,    # motor temperature (float, C)
                # NOV.STATUS_WORD,                                    # (2 bytes)
                # NOV.LAST_ERROR,                                     # (4 bytes)
                # NOV.POSITION_CONTROL_LOOP_ERROR,                    # (4 bytes)
                NOV.POSITION_LOOP_CONTROL_COMMAND,
                # NOV.VELOCITY_LOOP_CONTROL_COMMAND,
                # NOV.BUS_VOLTAGE_VALUE,                               # (4 bytes)
                NOV.POSITION_DEMAND,
                NOV.TORQUE_LOOP_CONTROL_COMMAND,
                # NOV.INTERPOLATION_DATA_RECORD_STATUS,
                # NOV.COMMUTATION_ANGLE_VALUE,    # doesn't work in cyclic mode
                NOV.INCREMENTAL_ENCODER_1_VALUE,
                NOV.ACTUAL_TORQUE,
            ]
        self.current_name = 'current_quadrature_a_control_loop_command'

        self.cyclic_pico_registers=[
                NOV.POSITION_SET_POINT,                             # (4 bytes)
                NOV.INTERPOLATION_DATA_RECORD_POSITION_INPUT,       # INT32 [counts] --> different from [encoder counts]
                NOV.INTERPOLATION_DATA_RECORD_VELOCITY_INPUT,       # FLOAT [rev/sec]
                NOV.INTERPOLATION_DATA_RECORD_TIME_INPUT,           # UINT16 [millisec]
                NOV.INTERPOLATION_DATA_RECORD_INTEGRITY_CHECK,      # UINT16
                NOV.POSITION_LOOP_KP,                               # (4 bytes)
                NOV.POSITION_LOOP_KD,
                NOV.VELOCITY_LOOP_KP,                               # (4 bytes)
                NOV.TORQUE_LOOP_KP,                                 # (4 bytes)
                NOV.TORQUE_LOOP_INPUT_OFFSET,                       # (4 bytes)
                # NOV.CURRENT_QUADRATURE_SET_POINT,
                NOV.CONTROL_WORD,
                NOV.OPERATION_MODE
            ]

        self._init_everest()
        ### init timer
        self._init_timer()


        # servo specific variables
        self.pos_incremental_encoder =0



    @property
    def position_incremental_encoder(self):
        return self.pos_incremental_encoder



    def enable(self, active):
        self.zero()
        self.reference_position = self._servo.read_uint(NOV.ACTUAL_POSITION)

        # in the current setup valid config are within [3072, 4096] and [0, 1024]
        # so not feasible ones are withing [1024 , 3072]
        if  3072 > self.reference_position > 1024:
            print("Current position: ", self.reference_position)
            raise ValueError("Robot out of range: move manually and then restart!")

        self._servo.channel_pico.set_idle()
        self._servo.state_transition(2)
        pyb.delay(100)
        self._servo.state_transition(3)
        pyb.delay(300)

        if active:
            self._servo.state_transition(4)
            pyb.delay(100)

        self._servo.enable_cyclic_mode()
        print('COMMUNICATION MODE =', self._servo.communication_mode)
        self.run = True

    def zero(self):
        self._servo.set_register(
            NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION_OFFSET,
            0
        )
        self._servo.set_register(
            NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION_OFFSET,
            -1 *712
        )


    def _update_position(self):
        self.pos = self._servo.channel_poci.actual_position

        # in the current setup valid config are within [3072, 4096] and [0, 1024]
        # so not feasible ones are withing [1024 , 3072]
        if 3072 > self.pos > 1024:
            print("Current position: ", self.pos)
            self._servo.operation_disable_cyclic()
            raise ValueError("Robot out of range: move manually and then restart!")


        self.pos_incremental_encoder = self._servo.channel_poci.incremental_encoder_1_value

    def _update_current_temp(self):
        self.current =  self._servo.channel_poci.current_quadrature_value
        self.counter_for_temp += 1
        # currently no temp sensor so keep constant to zero
        # if self.counter_for_temp==300:
        #     self.temperature = self._servo.channel_poci.motor_temperature_value
        #     self.counter_for_temp =0

    def _update_actual_torque(self):
        self.tau_actual = self._servo.channel_poci.actual_torque