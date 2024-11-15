from enum import IntFlag


class HyundaiFlagsSP(IntFlag):
  """
  Represents special flags for Hyundai specific features within sunnypilot.
  These flags will go into the `sunnyParams.flags` field of the `CarParams` (car.capnp)
  Max 32 flags can be defined in this class.
  """
  SP_ENHANCED_SCC = 1 << 0
  HAS_LFA_BUTTON = 1 << 1
