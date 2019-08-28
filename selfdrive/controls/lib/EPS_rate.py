import numpy as np

class eps_rate_gain(object):
  def __init__(self, CP, rate=100):
    self.torque_count = int(CP.steerActuatorDelay * float(rate)) - 1
    self.torque_samples = np.zeros(self.torque_count)
    self.angle_samples = np.zeros(self.torque_count)
    self.rate_samples = np.zeros(self.torque_count)

    self.frame = 0
    self.outer_angle = 0.
    self.inner_angle = 0.
    self.prev_angle = 0.
    self.torque_sum = 0.
    self.torque_rate_factor = 0.
    self.deadzone = 0.5
    self.spring_factor = 0.0
    self.prev_override = False

  def update(self, v_ego, angle_steers, rate_steers_des, eps_torque, steer_override, saturated):

    if abs(angle_steers) > abs(self.prev_angle):
        self.outer_angle = angle_steers
    elif abs(angle_steers) < abs(self.prev_angle):
        self.inner_angle = angle_steers

    notDeadzone = v_ego > 10.0 and ((abs(angle_steers - self.inner_angle) > self.deadzone) or (abs(angle_steers - self.outer_angle) > self.deadzone)) and \
        ((abs(self.angle_samples[self.frame % self.torque_count]) > self.inner_angle) or (abs(self.angle_samples[self.frame % self.torque_count]) < self.outer_angle))

    if not (steer_override or self.prev_override):
        self.torque_sum += eps_torque
        if notDeadzone and abs(self.torque_sum) > 0 and abs(angle_steers) < 30 and abs(rate_steers_des) < 20:
            self.torque_rate_factor += 0.001 * (((angle_steers - self.angle_samples[self.frame % self.torque_count]) / self.torque_sum) - self.torque_rate_factor)
    self.torque_sum -= self.torque_samples[self.frame % self.torque_count]

    if notDeadzone and not (steer_override or self.prev_override):
        self.advance_angle = self.spring_factor * self.torque_sum * self.torque_rate_factor
    else:
        self.advance_angle = 0.
    eps_rate = self.rate_samples[self.frame % self.torque_count]
    self.rate_samples[self.frame % self.torque_count] = 100.0 * eps_torque * self.torque_rate_factor
    self.torque_samples[self.frame % self.torque_count] = eps_torque
    self.angle_samples[self.frame % self.torque_count] = angle_steers
    self.prev_angle = angle_steers
    self.prev_override = steer_override
    self.frame += 1

    if self.frame % 10 == 0: print(self.advance_angle, eps_rate)
    return float(self.advance_angle), float(eps_rate)
