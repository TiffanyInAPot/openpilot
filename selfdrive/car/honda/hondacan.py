import struct
from selfdrive.config import Conversions as CV
from selfdrive.car.honda.values import CAR, HONDA_BOSCH

# *** Honda specific ***
def can_cksum(mm):
  s = 0
  for c in mm:
    c = ord(c)
    s += (c>>4)
    s += c & 0xF
  s = 8-s
  s %= 0x10
  return s


def fix(msg, addr):
  msg2 = msg[0:-1] + chr(ord(msg[-1]) | can_cksum(struct.pack("I", addr)+msg))
  return msg2


def create_brake_command(packer, apply_brake, pump_on, pcm_override, pcm_cancel_cmd, chime, fcw, idx):
  # TODO: do we loose pressure if we keep pump off for long?
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False

  values = {
    "COMPUTER_BRAKE": apply_brake,
    "BRAKE_PUMP_REQUEST": pump_on,
    "CRUISE_OVERRIDE": pcm_override,
    "CRUISE_FAULT_CMD": pcm_fault_cmd,
    "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    "COMPUTER_BRAKE_REQUEST": brake_rq,
    "SET_ME_0X80": 0x80,
    "BRAKE_LIGHTS": brakelights,
    "CHIME": chime,
    # TODO: Why are there two bits for fcw? According to dbc file the first bit should also work
    "FCW": fcw << 1,
  }
  return packer.make_can_msg("BRAKE_COMMAND", 0, values, idx)


def create_gas_command(packer, gas_amount, idx):
  enable = gas_amount > 0.001

  values = {"ENABLE": enable}

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  return packer.make_can_msg("GAS_COMMAND", 0, values, idx)


def create_steering_control(packer, apply_steer, CS, lkas_active, car_fingerprint, idx):
  if lkas_active:
    values = {
      "STEER_TORQUE": apply_steer,
      "STEER_TORQUE_REQUEST": lkas_active,
    }
  else:
    values = {
      "STEER_TORQUE": CS.steer_stock_torque,
      "STEER_TORQUE_REQUEST": CS.steer_stock_torque_request,
      "NEW_SIGNAL_1": CS.steer_parameter5,
      "NEW_SIGNAL_2": CS.steer_parameter6,
    }
        
  # Set bus 2 for accord and new crv.
  bus = 2 if car_fingerprint in HONDA_BOSCH else 0
  return packer.make_can_msg("STEERING_CONTROL", bus, values, idx)

def create_steering_control2(packer, CS, lkas_active, car_fingerprint, idx):
  values = {
    "NEW_SIGNAL_1": CS.steer_parameter1,
    "NEW_SIGNAL_2": int(CS.steer_parameter2) | 32,
    "NEW_SIGNAL_3": CS.steer_parameter3,
    "NEW_SIGNAL_4": CS.steer_parameter4,
  }
  bus = 2 if car_fingerprint in HONDA_BOSCH else 0
  return packer.make_can_msg("STEERING_CONTROL2", bus, values, idx)

def create_ui_commands(packer, pcm_speed, lkas_active, CS, hud, car_fingerprint, idx):
  commands = []
  bus = 0

  # Bosch sends commands to bus 2.
  if car_fingerprint in HONDA_BOSCH:
    bus = 2
  else:
    acc_hud_values = {
      'PCM_SPEED': pcm_speed * CV.MS_TO_KPH,
      'PCM_GAS': hud.pcm_accel,
      'CRUISE_SPEED': hud.v_cruise,
      'ENABLE_MINI_CAR': hud.mini_car,
      'HUD_LEAD': hud.car,
      'SET_ME_X03': 0x03,
      'SET_ME_X03_2': 0x03,
      'SET_ME_X01': 0x01,
    }
    commands.append(packer.make_can_msg("ACC_HUD", 0, acc_hud_values, idx))

  if lkas_active:
    lkas_hud_values = {
      'SET_ME_X41': 0x41,
      'SET_ME_X48': 0x48,
      'STEERING_REQUIRED': hud.steer_required,
      'SOLID_LANES': hud.lanes,
      'BEEP': hud.beep,
    }
  else:
    lkas_hud_values = {
      'SET_ME_X41': CS.SET_ME_X41,
      'BOH': CS.BOH,
      'DASHED_LANES': CS.DASHED_LANES,
      'DTC': CS.DTC,
      'LKAS_PROBLEM': CS.LKAS_PROBLEM,
      'LKAS_OFF': CS.LKAS_OFF,
      'SOLID_LANES': CS.SOLID_LANES,
      'LDW_RIGHT': CS.LDW_RIGHT,
      'STEERING_REQUIRED': CS.STEERING_REQUIRED,
      'BOH2': CS.BOH2,
      'LDW_PROBLEM': CS.LDW_PROBLEM,
      'BEEP': CS.BEEP,
      'LDW_ON': CS.LDW_ON,
      'LDW_OFF': CS.LDW_OFF,
      'CLEAN_WINDSHIELD': CS.CLEAN_WINDSHIELD,
      'SET_ME_X48': CS.SET_ME_X48,
    }
        
  commands.append(packer.make_can_msg('LKAS_HUD', bus, lkas_hud_values, idx))

  if car_fingerprint in (CAR.CIVIC, CAR.ODYSSEY):

    radar_hud_values = {
      'ACC_ALERTS': hud.acc_alert,
      'LEAD_SPEED': 0x1fe,  # What are these magic values
      'LEAD_STATE': 0x7,
      'LEAD_DISTANCE': 0x1e,
    }
    commands.append(packer.make_can_msg('RADAR_HUD', 0, radar_hud_values, idx))
  return commands


def spam_buttons_command(packer, button_val, idx):
  values = {
    'CRUISE_BUTTONS': button_val,
    'CRUISE_SETTING': 0,
  }
  return packer.make_can_msg("SCM_BUTTONS", 0, values, idx)
