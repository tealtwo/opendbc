def create_steering_control(packer, bus, apply_torque, lkas_enabled):
  values = {
    "LM_Offset": abs(apply_torque),
    "LM_OffSign": 1 if apply_torque < 0 else 0,
    "HCA_Status": 7 if (lkas_enabled and apply_torque != 0) else 3,
    "Vib_Freq": 16,
  }

  return packer.make_can_msg("HCA_1", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control):
  values = {}
  if len(ldw_stock_values):
    values = {s: ldw_stock_values[s] for s in [
      "LDW_SW_Warnung_links",   # Blind spot in warning mode on left side due to lane departure
      "LDW_SW_Warnung_rechts",  # Blind spot in warning mode on right side due to lane departure
      "LDW_Seite_DLCTLC",       # Direction of most likely lane departure (left or right)
      "LDW_DLC",                # Lane departure, distance to line crossing
      "LDW_TLC",                # Lane departure, time to line crossing
    ]}

  values.update({
    "LDW_Lampe_gelb": 1 if lat_active and steering_pressed else 0,
    "LDW_Lampe_gruen": 1 if lat_active and not steering_pressed else 0,
    "LDW_Lernmodus_links": 2 if lat_active else 1,
    "LDW_Lernmodus_rechts": 2 if lat_active else 1,
    "LDW_Kameratyp": 1,
    "LDW_Textbits": hud_alert,
  })

  return packer.make_can_msg("LDW_Status", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, longitudinalControl, frame=0, buttons=0, cancel=False, resume=False, custom_stock_long=False):
  values = {s: gra_stock_values[s] for s in [
    "GRA_Hauptschalt",      # ACC button, on/off
    "GRA_Typ_Hauptschalt",  # ACC button, momentary vs latching
    "GRA_Kodierinfo",       # ACC button, configuration
    "GRA_Sender",           # ACC button, CAN message originator
  ]}

  accel_cruise = 1 if buttons == 1 else 0
  decel_cruise = 1 if buttons == 2 else 0
  resume_cruise = 1 if buttons == 3 else 0
  set_cruise = 1 if buttons == 4 else 0

  values.update({
    "COUNTER": (frame + 1) % 0x10 if custom_stock_long else (gra_stock_values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen": cancel if not longitudinalControl else 0,
    "GRA_Recall": resume or resume_cruise if not longitudinalControl else 0,
    "GRA_Neu_Setzen": set_cruise if not longitudinalControl else 0,
    "GRA_Down_kurz": decel_cruise if not longitudinalControl else 0,
    "GRA_Up_kurz": accel_cruise if not longitudinalControl else 0,
  })

  return packer.make_can_msg("GRA_Neu", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active, cruiseOverride):
  if long_active or cruiseOverride:
    acc_control = 1
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, acc_control, cruiseOverride):
  if acc_faulted:
    hud_status = 6
  elif acc_control == 1:
    hud_status = 4 if cruiseOverride else 3
  elif main_switch_on:
    hud_status = 2
  else:
    hud_status = 0

  return hud_status


def create_acc_accel_control(packer, bus, acc_type, accel, acc_control, stopping, starting, esp_hold, comfortBand, jerkLimit):
  commands = []
  acc_enabled = 1 if acc_control == 1 else 0

  values = {
    "ACS_Sta_ADR": acc_control,
    "ACS_StSt_Info": acc_enabled,
    "ACS_Typ_ACC": acc_type,
    "ACS_Anhaltewunsch": acc_type == 1 and stopping,
    "ACS_FreigSollB": acc_enabled,
    "ACS_Sollbeschl": accel if acc_enabled else 3.01,
    "ACS_zul_Regelabw": comfortBand if acc_enabled else 1.27,
    "ACS_max_AendGrad": jerkLimit if acc_enabled else 5.08,
  }

  commands.append(packer.make_can_msg("ACC_System", bus, values))

  return commands

def create_epb_control(packer, bus, apply_brake, epb_enabled):

  values = {
    "EP1_Fehler_Sta": 0,
    "EP1_Sta_EPB": 0,
    "EP1_Spannkraft": 0,
    "EP1_Schalterinfo": 0,
    "EP1_Fkt_Lampe": 0,
    "EP1_Verzoegerung": apply_brake,                        #Brake request in m/s2
    "EP1_Freigabe_Ver": 1 if epb_enabled else 0,            #Allow braking pressure to build.
    "EP1_Bremslicht": 1 if apply_brake != 0 else 0,         #Enable brake lights
    "EP1_HydrHalten": 1 if epb_enabled else 0,              #Disengage DSG
    "EP1_AutoHold_aktiv": 1,                                #Signal indicating EPB is available
  }

  return packer.make_can_msg("EPB_1", bus, values)

def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance):
  values = {
    "ACA_StaACC": acc_hud_status,
    "ACA_Zeitluecke": distance + 2,
    "ACA_V_Wunsch": set_speed,
    "ACA_gemZeitl": lead_distance,
    "ACA_PrioDisp": 3,
    # TODO: restore dynamic pop-to-foreground/highlight behavior with ACA_PrioDisp and ACA_AnzDisplay
    # TODO: ACA_kmh_mph handling probably needed to resolve rounding errors in displayed setpoint
  }

  return packer.make_can_msg("ACC_GRA_Anzeige", bus, values)

def create_motor2_control(packer, bus, motor2_stock):
  values = motor2_stock
  values.update({
    "GRA_Status": 0,
  })
  return packer.make_can_msg("Motor_2", bus, values)
