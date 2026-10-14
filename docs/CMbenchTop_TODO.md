# CubeMars CAN Tasks

## Today: 2026-10-08

- [x] Add one launch setup for CM1 only, CM2 only, or both, with a separate group start/stop action. Rebuilt `micropython_logger` and validated configuration ACKs, concurrent enable requests, single-motor mode, and failed-start disable attempts using isolated ROS nodes and simulated boards, including the real `canIP2ROS` relay.
- [x] Expose group enable/disable in `rqt_reconfigure` under `/can_motor_group`, with the same readiness checks and ACK handling as the services. Verified GUI requests, service/checkbox consistency, startup with a stored enable value, missing ACK recovery, and single-motor mode using simulated boards. Added no-hardware regression tests in `tests/test_can_motor_group.py`.
- [ ] Bench-test the new launch and start/stop actions on the physical motors.
- [x] Add optional CubeMars `SMMultipleAct` with two-state Extension/Flexion profiles for OW, stairs ascent/descent, and ramps ascent/descent; per-motor rqt editing and YAML save/load; controller header 9 and existing 25-frame CAN/ACK format. Preserve the Fox four-phase implementation and the existing CubeMars walking controllers.
- [ ] Deploy and bench-test multi-activity switching and snapshot save/load on the physical motors. Tune activity profiles; shipped defaults are identical bench placeholders, not validated locomotion parameters.
- [x] Separate the shared requested activity in `/can_motor_group` from each motor's profile editor. Keep independent profiles, re-confirm common activity before enable, and attempt group disable on activity ACK failures. Validate with simulated boards.
- [x] Add StS as activity 5 with independent per-motor Extension/Flexion profiles and a repeating cycle. Group starts choose explicit same phases for StS, opposite phases for the other five activities; canIP applies that relation for AUTO late joins. Require disable before changing phase policy. Preserve version-1 snapshots with a warned StS bench-default migration; save all six activities as version 2.
- [ ] Deploy StS board/canIP updates and bench-test start phases, motor-specific profiles and stopping. No physical hardware was used for development; StS defaults remain untuned placeholders.

## Coordination Alternatives To Keep Open

- Initialization only: current canIP start-state synchronization, followed by autonomous local transitions.
- A shared CAN start release after the selected motors have acknowledged their configuration.
- Local transition readiness with high-level release at selected cycle boundaries.
- Gradual phase correction through bounded adjustments to future motion durations.
- Full high-level transition coordination, retaining local trajectory execution and local limits.

Keep one-motor operation available. Shared plotted canIP timestamps do not by themselves synchronize the Portenta execution clocks.
