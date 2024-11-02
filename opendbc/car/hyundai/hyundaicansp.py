from opendbc.car.hyundai import hyundaican
from opendbc.car.hyundai.hyundaican import get_scc11_values, calculate_scc12_checksum, \
    get_scc14_values, get_fca11_values, calculate_fca11_checksum
from sunnypilot.car.hyundai import escc


def get_scc12_values(enabled, accel, idx, stopping, long_override, use_fca, CS):
    scc12_values = hyundaican.get_scc12_values(enabled, accel, idx, stopping, long_override, use_fca)
    scc12_values = escc.update_escc_values(scc12_values, CS)
    return scc12_values


def create_acc_commands(packer, enabled, accel, upper_jerk, idx, hud_control, set_speed, stopping, long_override, use_fca, CS):
    commands = []

    scc11_values = get_scc11_values(enabled, set_speed, idx, hud_control)
    commands.append(packer.make_can_msg("SCC11", 0, scc11_values))

    scc12_values = get_scc12_values(enabled, accel, idx, stopping, long_override, use_fca, CS)
    scc12_values = calculate_scc12_checksum(packer, scc12_values)
    commands.append(packer.make_can_msg("SCC12", 0, scc12_values))

    scc14_values = get_scc14_values(enabled, upper_jerk, hud_control, long_override)
    commands.append(packer.make_can_msg("SCC14", 0, scc14_values))

    if use_fca:
        fca11_values = get_fca11_values(idx)
        fca11_values = calculate_fca11_checksum(packer, fca11_values)
        commands.append(packer.make_can_msg("FCA11", 0, fca11_values))

    return commands
