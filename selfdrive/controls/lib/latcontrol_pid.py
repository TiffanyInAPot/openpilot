from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from selfdrive.kegman_conf import kegman_conf
from common.numpy_fast import interp
import numpy as np
from cereal import car
from cereal import log
from common.realtime import sec_since_boot

class LatControlPID(object):
  def __init__(self, CP):
    kegman = kegman_conf(CP)
    self.frame = 0
    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=1.0)
    self.angle_steers_des = 0.
    self.damp_angle_steers = 0.
    self.damp_time = 0.1
    self.react_mpc = -0.05

  def live_tune(self, CP):
    self.frame += 1
    if self.frame % 300 == 0:
      # live tuning through /data/openpilot/tune.py overrides interface.py settings
      kegman = kegman_conf()
      self.pid._k_i = ([0.], [float(kegman.conf['Ki'])])
      self.pid._k_p = ([0.], [float(kegman.conf['Kp'])])
      self.pid.k_f = (float(kegman.conf['Kf']))
      self.damp_time = (float(kegman.conf['dampTime']))
      self.react_mpc = (float(kegman.conf['reactMPC']))

  def reset(self):
    self.pid.reset()
 
  def update(self, active, v_ego, angle_steers, angle_steers_rate, steer_override, CP, VM, path_plan):
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steerAngle = float(angle_steers)
    pid_log.steerRate = float(angle_steers_rate)

    self.live_tune(CP)

    if v_ego < 0.3 or not active:
      output_steer = 0.0
      self.damp_angle_steers= 0.0
      self.damp_angle_steers_des = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      self.angle_steers_des = path_plan.angleSteers
      self.damp_angle_steers_des += (interp(sec_since_boot() + 0.25 + self.react_mpc, path_plan.mpcTimes, path_plan.mpcAngles) - self.damp_angle_steers_des) / 25.0
      self.damp_angle_steers += (angle_steers + self.damp_time * angle_steers_rate - self.damp_angle_steers) / max(1.0, self.damp_time * 100.)
      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      steer_feedforward = self.damp_angle_steers_des   # feedforward desired angle
      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        # TODO: feedforward something based on path_plan.rateSteers
        steer_feedforward -= path_plan.angleOffset   # subtract the offset, since it does not contribute to resistive torque
        steer_feedforward *= v_ego**2  # proportional to realigning tire momentum (~ lateral accel)
      deadzone = 0.0
      output_steer = self.pid.update(self.damp_angle_steers_des, self.damp_angle_steers, check_saturation=(v_ego > 10), override=steer_override,
                                     feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)
      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(output_steer)
      pid_log.saturated = bool(self.pid.saturated)

    self.sat_flag = self.pid.saturated
    return output_steer, float(self.angle_steers_des), pid_log
