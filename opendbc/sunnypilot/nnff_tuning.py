import json
from typing import Any
from opendbc.car import structs

def apply_nnff_tuning(car_params: structs.CarParams, car_params_sp: structs.CarParamsSP) -> None:
    """
    Apply NNFF tuning parameters from the live tuner attached to car_params.

    This helper function checks if 'car_params' has a '_liveTuner' attribute.
    If present, it retrieves tuned parameters via get_tuned_params() and:
      - Updates car_params attributes such as vEgoStopping, vEgoStarting, and stoppingDecelRate.
      - Updates longitudinalTuning fields (kf, kpBP, kpV, kiBP, kiV) when the lengths match.
      - Sets car_params_sp.nnffLongTuning to a JSON-encoded string of the tuned parameters.

    Args:
      car_params: The CarParams object (or similar) that will have an attached live tune.
      car_params_sp: The CarParamsSP object stores the NNFF tuning data.
    """

    if hasattr(car_params, "_liveTuner"):
        tuned: dict[str, Any] = car_params._liveTuner.get_tuned_params()
        car_params.vEgoStopping = tuned['vego_stopping']
        car_params.vEgoStarting = tuned['vego_starting']
        car_params.stoppingDecelRate = tuned['stopping_decel_rate']
        if hasattr(car_params.longitudinalTuning, "kf"):
            car_params.longitudinalTuning.kf = tuned['kf']
        if len(car_params.longitudinalTuning.kpBP) == len(tuned['kpBP']):
            car_params.longitudinalTuning.kpBP = tuned['kpBP']
        if len(car_params.longitudinalTuning.kpV) == len(tuned['kpV']):
            car_params.longitudinalTuning.kpV = tuned['kpV']
        if len(car_params.longitudinalTuning.kiBP) == len(tuned['kiBP']):
            car_params.longitudinalTuning.kiBP = tuned['kiBP']
        if len(car_params.longitudinalTuning.kiV) == len(tuned['kiV']):
            car_params.longitudinalTuning.kiV = tuned['kiV']
        car_params_sp.nnffLongTuning = json.dumps(tuned)