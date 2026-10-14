# Generated from Lorenzo Vianello's micropython_logger devel/CMbenchTop
# (ef24346) cfg/*.cfg by parsing, not executing, each file. Order matches
# dynamic_reconfigure: top-level parameters, then groups in creation order.
from micropython_logger.reconfigure import ParamSpec

GuiTeleop = [
    ParamSpec('enable_controller', 'bool', False, None, None, 'Enable controller'),
    ParamSpec('eq_angle', 'double', 0.0, 0.0, 80.0, 'A double parameter representing pos'),
    ParamSpec('eq_vel', 'double', 0.0, 0.0, 1.0, 'A double parameter representing vel'),
    ParamSpec('K_stiffness', 'double', 0.5, 0.0, 2.0, 'A double parameter representing stiffness controller'),
    ParamSpec('B_damping', 'double', 0.1, 0.0, 0.2, 'A double parameter representing damping controller'),
    ParamSpec('dt', 'double', 1.0, 0.02, 2.0, 'A double parameter representing dt'),
]

GuiSMWalking2States = [
    ParamSpec('enable_controller', 'bool', False, None, None, 'Enable controller'),
    ParamSpec('extension_q_eq', 'double', 60.0, 30.0, 80.0, 'extension equilibrium angle'),
    ParamSpec('extension_q_dot_eq', 'double', 0.0, 0.0, 2.0, 'extension equilibrium velocity'),
    ParamSpec('extension_dt', 'double', 2.0, 0.0, 2.0, 'extension time to go'),
    ParamSpec('extension_K', 'double', 0.1, 0.0, 3.0, 'Extension stiffness'),
    ParamSpec('extension_B', 'double', 0.01, 0.0, 0.3, 'Extension damping'),
    ParamSpec('flexion_q_eq', 'double', 120.0, 80.0, 150.0, 'flexion equilibrium angle, must be greater than extension_q_eq'),
    ParamSpec('flexion_q_dot_eq', 'double', 0.0, 0.0, 2.0, 'flexion equilibrium velocity'),
    ParamSpec('flexion_dt', 'double', 2.0, 0.0, 2.0, 'flexion time to go'),
    ParamSpec('flexion_K', 'double', 0.1, 0.0, 3.0, 'Flexion stiffness'),
    ParamSpec('flexion_B', 'double', 0.01, 0.0, 0.3, 'Flexion damping'),
    ParamSpec('vel_tresh', 'double', 10.0, 5.0, 30.0, 'vel tresh'),
    ParamSpec('Fz_tresh', 'double', 10.0, 0.0, 50.0, 'Fz tresh'),
]

GuiSMWalking = [
    ParamSpec('enable_controller', 'bool', False, None, None, 'Enable controller'),
    ParamSpec('earlyStance_q_eq', 'double', 0.0, 0.0, 90.0, 'equilibrium angle'),
    ParamSpec('earlyStance_q_dot_eq', 'double', 0.0, 0.0, 2.0, 'equilibrium velocity'),
    ParamSpec('earlyStance_dt', 'double', 1.0, 0.0, 1.0, 'time to go'),
    ParamSpec('earlyStance_K', 'double', 0.1, 0.0, 3.0, 'Early Stance Stiffness '),
    ParamSpec('earlyStance_B', 'double', 0.01, 0.0, 0.3, 'Early Stance Damping'),
    ParamSpec('lateStance_q_eq', 'double', 0.0, 0.0, 90.0, 'equilibrium angle'),
    ParamSpec('lateStance_q_dot_eq', 'double', 0.0, 0.0, 2.0, 'equilibrium velocity'),
    ParamSpec('lateStance_dt', 'double', 1.0, 0.0, 1.0, 'time to go'),
    ParamSpec('lateStance_K', 'double', 0.1, 0.0, 3.0, 'Late Stance Stiffness'),
    ParamSpec('lateStance_B', 'double', 0.01, 0.0, 0.3, 'Late Stance Damping'),
    ParamSpec('earlySwing_q_eq', 'double', 50.0, 0.0, 90.0, 'equilibrium angle'),
    ParamSpec('earlySwing_q_dot_eq', 'double', 0.0, 0.0, 2.0, 'equilibrium velocity'),
    ParamSpec('earlySwing_dt', 'double', 2.0, 0.0, 2.0, 'time to go'),
    ParamSpec('earlySwing_K', 'double', 0.1, 0.0, 3.0, 'Early swing Stiffness'),
    ParamSpec('earlySwing_B', 'double', 0.01, 0.0, 0.3, 'Early Swing Damping'),
    ParamSpec('lateSwing_q_eq', 'double', 0.0, 0.0, 90.0, 'equilibrium angle'),
    ParamSpec('lateSwing_q_dot_eq', 'double', 0.0, 0.0, 2.0, 'equilibrium velocity'),
    ParamSpec('lateSwing_dt', 'double', 2.0, 0.0, 2.0, 'time to go'),
    ParamSpec('lateSwing_K', 'double', 0.1, 0.0, 3.0, 'Late Swing Stiffnes'),
    ParamSpec('lateSwing_B', 'double', 0.01, 0.0, 0.3, 'Late Swing Damping'),
    ParamSpec('vel_tresh', 'double', 10.0, 5.0, 30.0, 'vel tresh'),
    ParamSpec('Fz_tresh', 'double', 10.0, 0.0, 50.0, 'Fz tresh'),
]

GuiSMWalking3States = [
    ParamSpec('enable_controller', 'bool', False, None, None, 'Enable controller'),
    ParamSpec('stance_q_eq', 'double', 0.0, 0.0, 90.0, 'equilibrium angle'),
    ParamSpec('stance_q_dot_eq', 'double', 0.0, 0.0, 2.0, 'equilibrium velocity'),
    ParamSpec('stance_dt', 'double', 1.0, 0.0, 1.0, 'time to go'),
    ParamSpec('stance_K', 'double', 0.1, 0.0, 3.0, 'Stance Stiffness '),
    ParamSpec('stance_B', 'double', 0.01, 0.0, 0.3, 'Stance Damping'),
    ParamSpec('earlySwing_q_eq', 'double', 50.0, 0.0, 90.0, 'equilibrium angle'),
    ParamSpec('earlySwing_q_dot_eq', 'double', 0.0, 0.0, 2.0, 'equilibrium velocity'),
    ParamSpec('earlySwing_dt', 'double', 2.0, 0.0, 2.0, 'time to go'),
    ParamSpec('earlySwing_K', 'double', 0.1, 0.0, 3.0, 'Early swing Stiffness'),
    ParamSpec('earlySwing_B', 'double', 0.01, 0.0, 0.3, 'Early Swing Damping'),
    ParamSpec('lateSwing_q_eq', 'double', 0.0, 0.0, 90.0, 'equilibrium angle'),
    ParamSpec('lateSwing_q_dot_eq', 'double', 0.0, 0.0, 2.0, 'equilibrium velocity'),
    ParamSpec('lateSwing_dt', 'double', 2.0, 0.0, 2.0, 'time to go'),
    ParamSpec('lateSwing_K', 'double', 0.1, 0.0, 3.0, 'Late Swing Stiffnes'),
    ParamSpec('lateSwing_B', 'double', 0.01, 0.0, 0.3, 'Late Swing Damping'),
    ParamSpec('vel_tresh', 'double', 10.0, 5.0, 30.0, 'vel tresh'),
    ParamSpec('Fz_tresh', 'double', 10.0, 0.0, 50.0, 'Fz tresh'),
]

GuiCubeMarsSMMultiAct = [
    ParamSpec('enable_controller', 'bool', False, None, None, 'Enable this motor (use can_motor_group for a group start)'),
    ParamSpec('activity', 'int', 0, 0, 5, 'Activity profile to edit/view (does not request motion)', enum={0: 'OW', 1: 'StairsAsc', 2: 'StairsDesc', 3: 'RampAsc', 4: 'RampDesc', 5: 'StS'}),
    ParamSpec('requested_activity', 'str', 'OW', None, None, 'Status only: most recent commanded activity; use can_motor_group to change it'),
    ParamSpec('phase', 'int', 0, 0, 1, 'Phase to configure/view', enum={0: 'Extension', 1: 'Flexion'}),
    ParamSpec('q_eq', 'double', 60.0, 30.0, 150.0, 'Angle: Extension [30,90], Flexion [90,150]; respect board limits'),
    ParamSpec('qdot_eq', 'double', 0.0, 0.0, 2.0, 'Equilibrium velocity'),
    ParamSpec('dt', 'double', 2.0, 0.01, 10.0, 'Transition duration (seconds)'),
    ParamSpec('K', 'double', 0.1, 0.0, 3.0, 'Stiffness'),
    ParamSpec('B', 'double', 0.01, 0.0, 0.3, 'Damping'),
    ParamSpec('vel_tresh', 'double', 10.0, 5.0, 30.0, 'Velocity threshold for this activity'),
    ParamSpec('Fz_tresh', 'double', 10.0, 0.0, 50.0, 'Stored force threshold (current two-state guards do not use force)'),
    ParamSpec('save_name', 'str', '', None, None, 'Name for a snapshot of all six activities for this motor'),
    ParamSpec('save_now', 'bool', False, None, None, 'Save snapshot; resets after the operation'),
    ParamSpec('load_name', 'str', '', None, None, 'Saved snapshot to load (controller must be disabled)'),
    ParamSpec('load_now', 'bool', False, None, None, 'Load snapshot; resets after the operation'),
]

GuiSMMultiAct = [
    ParamSpec('enable_controller', 'bool', False, None, None, 'Enable controller'),
    ParamSpec('activity_source', 'int', 0, 0, 1, 'Where the active activity comes from', enum={0: 'Manual', 1: 'FromTopic_ComputerVision'}),
    ParamSpec('activity', 'int', 0, 0, 4, 'Activity being configured/requested', enum={0: 'OW', 1: 'StairsAsc', 2: 'StairsDesc', 3: 'RampAsc', 4: 'RampDesc'}),
    ParamSpec('phase', 'int', 0, 0, 3, 'Phase being configured', enum={0: 'EarlyStance', 1: 'LateStance', 2: 'EarlySwing', 3: 'LateSwing'}),
    ParamSpec('q_eq', 'double', 0.0, 0.0, 90.0, 'equilibrium angle'),
    ParamSpec('qdot_eq', 'double', 0.0, 0.0, 2.0, 'equilibrium velocity'),
    ParamSpec('dt', 'double', 1.0, 0.0, 2.0, 'time to go'),
    ParamSpec('K', 'double', 0.1, 0.0, 3.0, 'Stiffness'),
    ParamSpec('B', 'double', 0.01, 0.0, 0.3, 'Damping'),
    ParamSpec('vel_tresh', 'double', 10.0, 5.0, 30.0, 'vel tresh'),
    ParamSpec('Fz_tresh', 'double', 10.0, 0.0, 50.0, 'Fz tresh'),
    ParamSpec('thigh_tresh', 'double', 20.0, -30.0, 90.0, 'thigh tresh (EarlyStance exit)'),
    ParamSpec('save_name', 'str', '', None, None, 'Name to save current (all-activity, all-phase) params under'),
    ParamSpec('save_now', 'bool', False, None, None, 'Check to save current params under save_name'),
    ParamSpec('load_name', 'str', '', None, None, 'Name to load params from'),
    ParamSpec('load_now', 'bool', False, None, None, 'Check to load params from load_name'),
]

GuiOscillatorForce = [
    ParamSpec('enable_controller', 'bool', False, None, None, 'Enable controller'),
    ParamSpec('use_FF', 'bool', False, None, None, 'Use FF forces'),
    ParamSpec('use_Energy_var_freq', 'bool', False, None, None, 'Use Energy to modulate freq'),
    ParamSpec('freq_value', 'double', 0.25, 0.1, 0.5, 'A double parameter representing freq adaptive oscillator'),
    ParamSpec('min_angle', 'double', -50.0, -90.0, 80.0, 'Lower bound of the oscillation'),
    ParamSpec('max_angle', 'double', 60.0, 10.0, 90.0, 'Upper bound of the oscillation'),
    ParamSpec('K_stiffness', 'double', 2.0, 0.0, 3.0, 'A double parameter representing stiffness controller'),
    ParamSpec('B_damping', 'double', 0.1, 0.0, 0.5, 'A double parameter representing damping controller'),
    ParamSpec('lb', 'double', -2.0, -10.0, 0.0, 'Hysteresis lower bound'),
    ParamSpec('ub', 'double', 3.0, 0.0, 10.0, 'Hysteresis upper bound'),
    ParamSpec('K_1', 'double', 0.3, 0.01, 1.0, 'Residual work scaling factor 1'),
    ParamSpec('K_2', 'double', 200.0, 150.0, 250.0, 'Residual work scaling factor 2'),
    ParamSpec('forget_factor', 'double', 0.99, 0.1, 1.0, 'Forget factor res work: closer to 1 the more it is remember from the past'),
    ParamSpec('filt_loadcell', 'double', 0.2, 0.0, 1.0, 'Load cell filtering closer to 0 the more filtered the signal'),
    ParamSpec('mass_foot', 'double', 1.0, 0.0, 2.0, 'mass of the foot attached to the loadcell'),
    ParamSpec('l_foot', 'double', 0.15, 0.0, 0.5, 'distance from the loadcell and the weight'),
]

GuiOscillatorPos = [
    ParamSpec('enable_controller', 'bool', False, None, None, 'Enable controller'),
    ParamSpec('use_FF', 'bool', False, None, None, 'Use FF forces'),
    ParamSpec('enable_AFO', 'bool', False, None, None, 'Enable Adaptive freq oscillator'),
    ParamSpec('freq', 'double', 0.25, 0.1, 0.5, 'A double parameter representing freq (not used if AFO is active)'),
    ParamSpec('offset', 'double', 20.0, 5.0, 80.0, 'A double parameter representing offset'),
    ParamSpec('K_stiffness', 'double', 1.0, 0.0, 3.0, 'A double parameter representing stiffness controller'),
    ParamSpec('B_damping', 'double', 0.1, 0.0, 0.5, 'A double parameter representing damping controller'),
    ParamSpec('K_freq', 'double', 0.0, 0.0, 0.5, 'A double parameter representing gain freq adapt osc'),
    ParamSpec('K_phase', 'double', 0.0, 0.0, 0.1, 'A double parameter representing gain phase adapt osc'),
    ParamSpec('K_amp', 'double', 0.0, 0.0, 0.1, 'A double parameter representing gain ampl adapt osc'),
]

GuiFFCal = [
    ParamSpec('enable_FF', 'bool', False, None, None, 'Enable FF controller'),
    ParamSpec('robot_state_Swing', 'bool', True, None, None, 'True is default and it is the swing'),
    ParamSpec('mass_thigh', 'double', 0.2, 0.0, 10.0, 'A double parameter representing the thigh mass'),
    ParamSpec('mass_shank', 'double', 3.0, 0.0, 5.0, 'A double parameter representing the shank mass'),
    ParamSpec('l_com_thigh', 'double', 0.1, 0.1, 0.75, 'A double parameter representing the com pos'),
    ParamSpec('l_com_shank', 'double', 0.15, 0.1, 0.75, 'A double parameter representing the com pos'),
    ParamSpec('k_spring', 'double', 0.0124, 0.0, 0.1, 'A double parameter representing the spring stiffness'),
    ParamSpec('eq_angle', 'double', -29.85, -30.0, 10.0, 'A double parameter representing the eq angle'),
    ParamSpec('b', 'double', 0.02, 0.0, 0.1, 'A double parameter representing the viscousity coeff'),
    ParamSpec('v_th', 'double', 3.0, 1.0, 5.0, 'A double parameter representing the v_th '),
    ParamSpec('stic', 'double', 0.003, -0.005, 0.005, 'A double parameter representing the stic'),
    ParamSpec('coul', 'double', 0.25, -0.25, 0.5, 'A double parameter representing the coul'),
    ParamSpec('I_thigh', 'double', 0.002, 0.0, 0.005, 'A double parameter representing the Inertia'),
    ParamSpec('I_shank', 'double', 0.002, 0.0, 0.005, 'A double parameter representing the Inertia'),
]

GuiFBTransp = [
    ParamSpec('enable_Controller', 'bool', False, None, None, 'Enable FB controller'),
    ParamSpec('enable_FF', 'bool', False, None, None, 'Use FF forces'),
    ParamSpec('tau_int_des', 'double', 0.0, -5.0, 5.0, 'Desired int torque profile'),
    ParamSpec('P_fb', 'double', 0.5, 0.0, 1.5, 'Prop PID param'),
    ParamSpec('I_fb', 'double', 0.0, 0.0, 0.1, 'Integral'),
    ParamSpec('D_fb', 'double', 0.02, 0.0, 0.1, 'Derivative'),
    ParamSpec('alpha_test', 'double', 0.0, 0.0, 1.0, 'rescaling factor for test, closer to zero lower torque'),
    ParamSpec('filter_loadcell', 'double', 0.2, 0.0, 1.0, 'Load cell filtering closer to 0 the more filtered the signal'),
]

GuiTeleopCAN = [
    ParamSpec('enable_controller', 'bool', False, None, None, 'Enable controller'),
    ParamSpec('K_stiffness', 'double', 0.5, 0.0, 2.0, 'A double parameter representing stiffness controller'),
    ParamSpec('B_damping', 'double', 0.1, 0.0, 0.2, 'A double parameter representing damping controller'),
]

GuiCANMotorGroup = [
    ParamSpec('enable_controller', 'bool', False, None, None, 'Enable all selected motor controllers after readiness checks; uncheck to disable'),
]

GuiCANMotorGroupActivity = [
    ParamSpec('activity', 'int', 0, 0, 5, 'Shared activity; stop before switching into/out of StS', enum={0: 'OW', 1: 'StairsAsc', 2: 'StairsDesc', 3: 'RampAsc', 4: 'RampDesc', 5: 'StS'}),
    ParamSpec('starting_phase', 'int', 0, 0, 1, 'Starting phase of first selected motor; change only while stopped', enum={0: 'Extension', 1: 'Flexion'}),
    ParamSpec('enable_controller', 'bool', False, None, None, 'Enable selected motors after activity/ACK checks; uncheck to disable'),
]

