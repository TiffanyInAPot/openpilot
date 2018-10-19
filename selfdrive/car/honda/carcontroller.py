import zmq
import time
from cereal import car
from collections import namedtuple
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.controls.lib.drive_helpers import rate_limit
from common.numpy_fast import clip
from selfdrive.car.honda import hondacan
from selfdrive.car.honda.values import AH, CruiseButtons, CAR, CruiseSettings
from selfdrive.can.packer import CANPacker

def actuator_hystereses(brake, braking, brake_steady, v_ego, car_fingerprint):
  # hyst params... TODO: move these to VehicleParams
  brake_hyst_on = 0.02     # to activate brakes exceed this value
  brake_hyst_off = 0.005                     # to deactivate brakes below this value
  brake_hyst_gap = 0.01                      # don't change brake command for small ocilalitons within this value

  #*** histeresis logic to avoid brake blinking. go above 0.1 to trigger
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  brake = brake_steady

  if (car_fingerprint in (CAR.ACURA_ILX, CAR.CRV)) and brake > 0.0:
    brake += 0.15

  return brake, braking, brake_steady


def process_hud_alert(hud_alert):
  # initialize to no alert
  fcw_display = 0
  steer_required = 0
  acc_alert = 0
  if hud_alert == AH.NONE:          # no alert
    pass
  elif hud_alert == AH.FCW:         # FCW
    fcw_display = hud_alert[1]
  elif hud_alert == AH.STEER:       # STEER
    steer_required = hud_alert[1]
  else:                             # any other ACC alert
    acc_alert = hud_alert[1]

  return fcw_display, steer_required, acc_alert


HUDData = namedtuple("HUDData",
                     ["pcm_accel", "v_cruise", "mini_car", "car", "X4",
                      "lanes", "beep", "chime", "fcw", "acc_alert", "steer_required", 
                      "r_hud_speed", "gernby1", "gernby2", "gernby3", "gernby4", "LKAS_OFF","LDW_RIGHT", "dashed_lanes",
                      "LDW_ON","LDW_OFF"])


class CarController(object):
  def __init__(self, dbc_name, enable_camera=True):
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.enable_camera = enable_camera
    self.packer = CANPacker(dbc_name)
    self.new_radar_config = False
    self.auto_Steer = True
    self.stock_lane_limit = 1.
    self.context = zmq.Context()
    self.steerpub = self.context.socket(zmq.PUB)
    self.steerpub.bind("tcp://*:8593")
    self.avg_lane_limit = 1.
    self.sample_count = 1.
    self.steerData = ""
    self.avg_lane_center = 0.
    self.stock_torque_offset = 0
    self.stop_protecting_frame = 0
    self.stock_online = False
    self.stock_lane_center = 0.
    self.avg_apply_steer = 0.
    self.max_stock_steer = 0
    self.min_stock_steer = 0

  def update(self, sendcan, enabled, CS, frame, actuators, \
             pcm_speed, pcm_override, pcm_cancel_cmd, pcm_accel, \
             radar_error, hud_v_cruise, hud_show_lanes, hud_show_car, \
             hud_alert, snd_beep, snd_chime):

    """ Controls thread """

    if not self.enable_camera:
      return
    
    # *** apply brake hysteresis ***
    brake, self.braking, self.brake_steady = actuator_hystereses(actuators.brake, self.braking, self.brake_steady, CS.v_ego, CS.CP.carFingerprint)

    # *** no output if not enabled ***
    if not enabled and CS.pcm_acc_status:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
      pcm_cancel_cmd = True

    # *** rate limit after the enable check ***
    self.brake_last = rate_limit(brake, self.brake_last, -2., 1./100)

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    if hud_show_lanes:
      hud_lanes = 1
    else:
      hud_lanes = 0

    if enabled:
      if hud_show_car:
        hud_car = 2
      else:
        hud_car = 1
    else:
      hud_car = 0

    # For lateral control-only, send chimes as a beep since we don't send 0x1fa
    if CS.CP.radarOffCan:
      snd_beep = not CS.brake_pressed and not CS.steer_override and (snd_beep if snd_beep is not 0 else snd_chime)

    #print chime, alert_id, hud_alert
    fcw_display, steer_required, acc_alert = process_hud_alert(hud_alert)

    stock_lkas_enabled = CS.lkas_hud_dashed_lanes + CS.lkas_hud_solid_lanes > 0
    if not enabled:
      CS.lkas_hud_dashed_lanes = 0 #CS.lkas_hud_dashed_lanes
      CS.lkas_hud_solid_lanes = 0
    elif CS.lkas_hud_solid_lanes == 0:
      CS.lkas_hud_dashed_lanes = hud_lanes
      hud_lanes = 0  

    hud = HUDData(int(pcm_accel), int(round(hud_v_cruise)), 1, hud_car, 0xc1, 
                  max(CS.lkas_hud_solid_lanes, hud_lanes), int(snd_beep), snd_chime, 
                  fcw_display, acc_alert, steer_required,
                  int(min(1023, CS.v_ego_raw * 600)), CS.lkas_hud_GERNBY1, 
                  CS.lkas_hud_GERNBY2, CS.lkas_hud_GERNBY3, CS.lkas_hud_GERNBY4,
                  CS.lkas_hud_LKAS_OFF, CS.lkas_hud_LDW_RIGHT, CS.lkas_hud_dashed_lanes, 
                  CS.lkas_hud_LDW_ON, CS.lkas_hud_LDW_OFF)

#    if not all(isinstance(x, int) and 0 <= x < 256 for x in hud):
#      hud = HUDData(0xc6, 255, 64, 0xc0, 209, 0x40, 0, 0, 0, 0, 0, int(max(1023, CS.v_ego_raw *20)))

    # **** process the car messages ****

    # *** compute control surfaces ***
    BRAKE_MAX = 1024/4
    if CS.CP.carFingerprint in (CAR.ACURA_ILX):
      STEER_MAX = 0xF00
    elif CS.CP.carFingerprint in (CAR.CRV, CAR.ACURA_RDX):
      STEER_MAX = 0x3e8  # CR-V only uses 12-bits and requires a lower value (max value from energee)
    else:
      STEER_MAX = 0x1000

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_gas = clip(actuators.gas, 0., 1.)
    apply_brake = int(clip(self.brake_last * BRAKE_MAX, 0, BRAKE_MAX - 1))

    # any other cp.vl[0x18F]['STEER_STATUS'] is common and can happen during user override. sending 0 torque to avoid EPS sending error 5
    lkas_active = enabled and not CS.steer_not_allowed and self.auto_Steer

    orig_apply_steer = int(clip(-actuators.steer * STEER_MAX, -STEER_MAX, STEER_MAX))      
    protect_hard = 0   
    lane_diff_1 = 0
    lane_diff_2 = 0
    cross_diff = 0
    protect_hard = 0
    min_steer_limit = 0
    OP_STEER_AT_STOCK_LANE_CENTER = 0.333
    STOCK_FILTER_WIDTH = 15.
    force_zmq_send = False
    MAX_STEERING_SAMPLES = int(2000. / (CS.v_ego_raw + 1))
    speed_factored_average = int(500 / (CS.v_ego_raw + 1))
    
    if False == False:

      if (CS.lane13 + CS.lane33 + CS.lane53 + CS.lane73) > 0:
            
        self.stock_online = True
        
        self.sample_count = min(MAX_STEERING_SAMPLES, self.sample_count + 1.)

        total_lane_confidence = (CS.lane13 + CS.lane33 + CS.lane53 + CS.lane73) 
        
        self.stock_lane_center = ((CS.lane11 * CS.lane13) + (CS.lane31 * CS.lane33) + (CS.lane51 * CS.lane53) + (CS.lane71 * CS.lane73)) / total_lane_confidence

        if (CS.lane13 + CS.lane53) > (CS.lane33 + CS.lane73) and actuators.steer < 0:
          #OP steer direction favors strong lane confidence
          min_steer_limit = 0.2
        else:
          min_steer_limit = 0.0

        if actuators.steer > 0 and self.avg_lane_center < 0:
          # OP agrees with stock lane orientation
          self.stock_lane_limit = min(1., OP_STEER_AT_STOCK_LANE_CENTER + min(STOCK_FILTER_WIDTH, abs(self.avg_lane_center)) / STOCK_FILTER_WIDTH)
        else:
          # OP disagrees with stock lane orientation
          self.stock_lane_limit = max(min_steer_limit, OP_STEER_AT_STOCK_LANE_CENTER - 1. + ((STOCK_FILTER_WIDTH - min(STOCK_FILTER_WIDTH, abs(self.avg_lane_center))) / STOCK_FILTER_WIDTH))
                    
      else:
        self.sample_count = max(0., self.sample_count - 1.)
        self.stock_lane_limit = 1.

      self.avg_lane_limit = ((self.sample_count * self.avg_lane_limit) + self.stock_lane_limit) / (self.sample_count + 1.)
      
      if not self.stock_online:
        self.stock_lane_limit = self.avg_lane_limit
      
      apply_steer = int(clip(-actuators.steer * STEER_MAX, -STEER_MAX * self.stock_lane_limit, STEER_MAX * self.stock_lane_limit))

    else:
      apply_steer = orig_apply_steer
      
    if CS.stock_steer_steer_torque != 0:          
      apply_steer = int(clip(1.15 * CS.stock_steer_steer_torque, -STEER_MAX, STEER_MAX))
      lkas_active = int(CS.stock_steer_request)

    if CS.blinker_on or not self.auto_Steer or (CS.steer_override and (apply_steer < 0) == (CS.steer_torque_driver < 0)):
      apply_steer = 0

    # Send CAN commands.
    can_sends = []

    # Send steering command.
    idx = frame % 4
      
    can_sends.extend(hondacan.create_steering_control(self.packer, int(apply_steer), lkas_active, CS.CP.carFingerprint, idx))

    if (frame % 10) == 0:
      self.steerData += ('%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d|' \
              % (CS.lkas_hud_GERNBY1, CS.lkas_hud_GERNBY2, CS.lkas_hud_LKAS_PROBLEM, CS.lkas_hud_LKAS_OFF, CS.lkas_hud_LDW_RIGHT, CS.lkas_hud_BEEP, 
              CS.lkas_hud_LDW_ON, CS.lkas_hud_LDW_OFF, CS.lkas_hud_CLEAN_WINDSHIELD, CS.lkas_hud_DTC, CS.lkas_hud_CAM_TEMP_HIGH, CS.radar_hud_gernby1, 
              CS.radar_hud_gernby2, CS.radar_hud_gernby3, CS.radar_hud_gernby4, CS.radar_hud_gernby5, CS.radar_hud_gernby6, CS.radar_hud_CMBS_OFF, 
              CS.radar_hud_RESUME_INSTRUCTION, CS.stock_steer_request, CS.stock_steer_set_me_x00, CS.stock_steer_set_me_x00_2, CS.stock_steer_steer_torque, 
              CS.lkas_hud_solid_lanes, CS.lkas_hud_steering_required, CS.lkas_hud_GERNBY3, CS.lkas_hud_GERNBY4, CS.lkas_hud_dashed_lanes, CS.v_ego_raw,
              CS.lane11,CS.lane12,CS.lane13,CS.lane14,CS.lane15,CS.lane16,CS.lane17,CS.lane31,CS.lane32,CS.lane33,CS.lane34,CS.lane35,CS.lane36,CS.lane37,
              CS.lane51,CS.lane52,CS.lane53,CS.lane54,CS.lane55,CS.lane56,CS.lane57,CS.lane71,CS.lane72,CS.lane73,CS.lane74,CS.lane75,CS.lane76,CS.lane77,
              self.stock_lane_center,protect_hard,100*min_steer_limit,100*OP_STEER_AT_STOCK_LANE_CENTER,lane_diff_1,lane_diff_2,cross_diff,orig_apply_steer, 
              CS.angle_steers, CS.angle_steers_rate, self.avg_lane_limit, frame, self.avg_lane_center, self.sample_count, apply_steer, CS.steer_torque_driver, 
              self.stock_lane_limit * 100, int(time.time() * 1000000000)))
              
    elif len(self.steerData) > 10 and (frame % 10) == 5:
      self.steerpub.send(self.steerData)
      self.steerData = ""

    # Send dashboard UI commands.
    if (frame % 10) == 0:
      idx = (frame/10) % 4
      can_sends.extend(hondacan.create_ui_commands(self.packer, pcm_speed, hud, CS.CP.carFingerprint, idx))

    if CS.CP.radarOffCan:
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.CANCEL, idx))
      elif CS.stopped:
        can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RES_ACCEL, idx))
      elif enabled != stock_lkas_enabled:
        if frame % 1000 < 330:
          can_sends.append(hondacan.spam_lkas_command(self.packer, CruiseSettings.LKAS, idx))
      elif CS.new_cruise_target_speed > 0 and (frame % 50) <= 20:
        if enabled and CS.new_cruise_target_speed < CS.v_cruise_pcm:
          can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.DECEL_SET, idx))
        elif enabled and CS.new_cruise_target_speed > CS.v_cruise_pcm:
          can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RES_ACCEL, idx))
        elif CS.new_cruise_target_speed == CS.v_cruise_pcm:
          CS.new_cruise_target_speed = 0
    else:
      # Send gas and brake commands.
      if (frame % 2) == 0:
        idx = (frame / 2) % 4
        can_sends.append(
          hondacan.create_brake_command(self.packer, apply_brake, pcm_override,
                                      pcm_cancel_cmd, hud.chime, hud.fcw, idx))
        if CS.CP.enableGasInterceptor:
          # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
          # This prevents unexpected pedal range rescaling
          can_sends.append(hondacan.create_gas_command(self.packer, apply_gas, idx))

      # radar at 20Hz, but these msgs need to be sent at 50Hz on ilx (seems like an Acura bug)
      if CS.CP.carFingerprint == CAR.ACURA_ILX:
        radar_send_step = 2
      else:
        radar_send_step = 5

      if (frame % radar_send_step) == 0:
        idx = (frame/radar_send_step) % 4
        if not self.new_radar_config:  # only change state once
          self.new_radar_config = car.RadarState.Error.wrongConfig in radar_error
        can_sends.extend(hondacan.create_radar_commands(CS.v_ego, CS.CP.carFingerprint, self.new_radar_config, idx))

    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
