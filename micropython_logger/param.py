CONTROLLER_CONFIGS = {
    "Teleop": {
        "params": {
            "enable_controller": False,
            "eq_angle": 0.0,
            "eq_vel": 0.0,
            "K_stiffness": 0.5,
            "B_damping": 0.1,
            "dt": 1.0
        },
        "map": {
            "enable_controller": 0, "eq_angle": 1, "eq_vel": 2,
            "K_stiffness": 3, "B_damping": 4, "dt": 5
        }
    },

    "SMWalking": {
        "params": {
            "enable_controller": False,
            "stance_q_eq": 0.0,
            "stance_q_dot_eq": 0.0,
            "stance_dt": 1.0,
            "stance_K": 0.1,
            "stance_B": 0.01,
            "earlySwing_q_eq": 50.0,
            "earlySwing_q_dot_eq": 0.0,
            "earlySwing_dt": 2.0,
            "earlySwing_K": 0.1,
            "earlySwing_B": 0.01,
            "lateSwing_q_eq": 0.0,
            "lateSwing_q_dot_eq": 0.0,
            "lateSwing_dt": 2.0,
            "lateSwing_K": 0.1,
            "lateSwing_B": 0.01,
            "vel_tresh": 10.0,
            "Fz_tresh": 10.0
        },
        "map": {
            "enable_controller": 0, "stance_q_eq": 1, "stance_q_dot_eq": 2, "stance_dt": 3,
            "stance_K": 4, "stance_B": 5, "earlySwing_q_eq": 6, "earlySwing_q_dot_eq": 7,
            "earlySwing_dt": 8, "earlySwing_K": 9, "earlySwing_B": 10, "lateSwing_q_eq": 11,
            "lateSwing_q_dot_eq": 12, "lateSwing_dt": 13, "lateSwing_K": 14, "lateSwing_B": 15,
            "vel_tresh": 16, "Fz_tresh": 17
        }
    },

    "FF": {
        "params": {
            "enable_FF": False,
            "robot_state_Swing": True,
            "mass_thigh": 0.2,
            "mass_shank": 3.0,
            "l_com_thigh": 0.1,
            "l_com_shank": 0.15,
            "k_spring": 0.0124,
            "eq_angle": -29.85,
            "b": 0.02,
            "v_th": 3.0,
            "stic": 0.003,
            "coul": 0.25,
            "I_thigh": 0.002,
            "I_shank": 0.002
        },
        "map": {
            "enable_FF": 0, "robot_state_Swing": 1, "mass_thigh": 2, "mass_shank": 3,
            "l_com_thigh": 4, "l_com_shank": 5, "k_spring": 6, "eq_angle": 7, "b": 8,
            "v_th": 9, "stic": 10, "coul": 11, "I_thigh": 12, "I_shank": 13
        }
    },

    "FB": {
        "params": {
            "enable_Controller": False,
            "enable_FF": False,
            "tau_int_des": 0.0,
            "P_fb": 0.5,
            "I_fb": 0.0,
            "D_fb": 0.02,
            "alpha_test": 0.0,
            "filter_loadcell": 0.2
        },
        "map": {
            "enable_Controller": 0, "enable_FF": 1, "tau_int_des": 2, "P_fb": 3,
            "I_fb": 4, "D_fb": 5, "alpha_test": 6, "filter_loadcell": 7
        }
    },

    "OscillatorPos": {
        "params": {
            "enable_controller": False,
            "use_FF": False,
            "enable_AFO": False,
            "freq": 0.25,
            "offset": 20.0,
            "K_stiffness": 1.0,
            "B_damping": 0.1,
            "K_freq": 0.0,
            "K_phase": 0.0,
            "K_amp": 0.0
        },
        "map": {
            "enable_controller": 0, "use_FF": 1, "enable_AFO": 2, "freq": 3, "offset": 4,
            "K_stiffness": 5, "B_damping": 6, "K_freq": 7, "K_phase": 8, "K_amp": 9
        }
    },

    "OscillatorForce": {
        "params": {
            "enable_controller": False,
            "use_FF": False,
            "use_Energy_var_freq": False,
            "freq_value": 0.25,
            "min_angle": -50.0,
            "max_angle": 60.0,
            "K_stiffness": 2.0,
            "B_damping": 0.1,
            "lb": -2.0,
            "ub": 3.0,
            "K_1": 0.3,
            "K_2": 200.0,
            "forget_factor": 0.99,
            "filt_loadcell": 0.2,
            "mass_foot": 1.0,
            "l_foot": 0.15
        },
        "map": {
            "enable_controller": 0, "use_FF": 1, "use_Energy_var_freq": 2, "freq_value": 3,
            "min_angle": 4, "max_angle": 5, "K_stiffness": 6, "B_damping": 7, "lb": 8, "ub": 9,
            "K_1": 10, "K_2": 11, "forget_factor": 12, "filt_loadcell": 13, "mass_foot": 14, "l_foot": 15
        }
    }
}


def load_controller_params(node, ctrl_mode):
    if ctrl_mode not in CONTROLLER_CONFIGS:
        node.get_logger().error(f"Unknown controller mode: {ctrl_mode}")
        return {}

    config = CONTROLLER_CONFIGS[ctrl_mode]
    
    for param_name, default_value in config["params"].items():
        node.declare_parameter(param_name, default_value)
        
    return config["map"]