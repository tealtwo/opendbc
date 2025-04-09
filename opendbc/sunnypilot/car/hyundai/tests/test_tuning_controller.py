import unittest
import math
import numpy as np
from unittest.mock import Mock, patch
from opendbc.sunnypilot.car.hyundai.longitudinal.tuning_controller import LongitudinalTuningController, LongitudinalTuningState
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase

LongCtrlState = structs.CarControl.Actuators.LongControlState


class TestLongitudinalTuningController(unittest.TestCase):
  def setUp(self):
    self.mock_CP = Mock(carFingerprint="KIA_NIRO_EV")
    self.mock_CP_SP = Mock(flags=0)

    # Mock car_config
    with patch('opendbc.sunnypilot.car.hyundai.longitudinal.helpers.get_car_config') as mock_get_config:
      mock_get_config.return_value = Mock(jerk_limits=[0.53, 3.0, 2.0])  # min, decel, accel
      self.controller = LongitudinalTuningController(self.mock_CP, self.mock_CP_SP)
    print(f"\n[SETUP] Controller initialized with jerk limits: {self.controller.car_config.jerk_limits}")

  def test_init(self):
    """Test controller initialization"""
    self.assertIsInstance(self.controller.state, LongitudinalTuningState)
    self.assertEqual(self.controller.accel_raw, 0.0)
    self.assertEqual(self.controller.accel_value, 0.0)
    self.assertEqual(self.controller.jerk_upper, 0.0)
    self.assertEqual(self.controller.jerk_lower, 0.0)

  def test_reset(self):
    """Test reset functionality"""
    # Set non-zero values and verify reset
    attrs = ['accel_raw', 'accel_value', 'jerk_upper', 'jerk_lower']
    state_attrs = ['accel_last', 'accel_last_jerk']

    for attr in attrs:
      setattr(self.controller, attr, 1.0)
    for attr in state_attrs:
      setattr(self.controller.state, attr, 1.0)

    self.controller.reset()

    # Verify all reset to 0.0
    for attr in attrs:
      self.assertEqual(getattr(self.controller, attr), 0.0)
    for attr in state_attrs:
      self.assertEqual(getattr(self.controller.state, attr), 0.0)

  def test_make_jerk_flag_off(self):
    """Test when LONG_TUNING_BRAKING flag is off"""
    mock_CC, mock_CS = Mock(spec=structs.CarControl), Mock(spec=CarStateBase)

    # Test with PID state
    self.controller.make_jerk(mock_CC, mock_CS, LongCtrlState.pid)
    print(f"[PID state] jerk_upper={self.controller.jerk_upper:.2f}, jerk_lower={self.controller.jerk_lower:.2f}")
    self.assertEqual(self.controller.jerk_upper, 3.0)
    self.assertEqual(self.controller.jerk_lower, 3.0)

    # Test with non-PID state
    self.controller.make_jerk(mock_CC, mock_CS, LongCtrlState.stopping)
    print(f"[Non-PID state] jerk_upper={self.controller.jerk_upper:.2f}, jerk_lower={self.controller.jerk_lower:.2f}")
    self.assertEqual(self.controller.jerk_upper, 1.0)
    self.assertEqual(self.controller.jerk_lower, 1.0)

  def test_make_jerk_flag_on(self):
    """Test when LONG_TUNING_BRAKING flag is on"""
    self.controller.CP_SP.flags = HyundaiFlagsSP.LONG_TUNING_BRAKING
    print(f"\n[test_make_jerk_flag_on] LONG_TUNING_BRAKING flag set: {self.controller.CP_SP.flags}")

    # Setup test mocks
    mock_CC = Mock()
    mock_CC.actuators = Mock(accel=1.0)
    mock_CS = Mock()
    mock_CS.out = Mock(aEgo=0.8, vEgo=3.0)

    self.controller.reset()
    print(f"After reset: accel_last_jerk={self.controller.state.accel_last_jerk}")
    print(f"First call with planned_accel={mock_CC.actuators.accel}, current_accel={mock_CS.out.aEgo}")

    self.controller.make_jerk(mock_CC, mock_CS, LongCtrlState.pid)

    blended_value = 0.80 * 1.0 + 0.20 * 3.0
    delta = blended_value
    expected_jerk = math.copysign(delta * delta, delta)

    print(f"Blended: {blended_value}, Delta: {delta}")
    print(f"Expected jerk: {expected_jerk}, Actual jerk: {self.controller.state.jerk}")
    print(f"Jerk limits (upper/lower): {self.controller.jerk_upper:.4f}/{self.controller.jerk_lower:.4f}")

    self.assertAlmostEqual(self.controller.state.jerk, expected_jerk)
    self.assertGreaterEqual(self.controller.jerk_upper, 0.6)  # Min at low velocity

    # Test with high velocity
    mock_CS.out.vEgo = 25.0
    self.controller.reset()
    print(f"Second call with high velocity={mock_CS.out.vEgo}")

    self.controller.make_jerk(mock_CC, mock_CS, LongCtrlState.pid)
    print(f"High velocity jerk limits: upper={self.controller.jerk_upper:.4f}, lower={self.controller.jerk_lower:.4f}")

    self.assertGreaterEqual(self.controller.jerk_upper, 0.53)  # Should use min jerk limit

  def test_jerk_calculation(self):
    """Test jerk calculation with various inputs"""
    self.controller.CP_SP.flags = HyundaiFlagsSP.LONG_TUNING_BRAKING
    print("\n[test_jerk_calculation] Testing with various acceleration values")

    mock_CC, mock_CS = Mock(), Mock()
    mock_CC.actuators = Mock()
    mock_CS.out = Mock(aEgo=0.0, vEgo=10.0) # change this to whatever you want

    test_deltas = [-2.0, -1.0, -0.5, -0.1, -0.01, 0.0, 0.01, 0.1, 0.5, 1.0, 2.0]

    for planned_accel in test_deltas:
      self.controller.reset()
      mock_CC.actuators.accel = planned_accel

      # Force a deterministic state for testing
      self.controller.make_jerk(mock_CC, mock_CS, LongCtrlState.pid)
      self.controller.state.accel_last_jerk = 0.0
      self.controller.make_jerk(mock_CC, mock_CS, LongCtrlState.pid)

      # Calculate expected jerk
      blended_value = 0.80 * planned_accel + 0.20 * 10.0
      delta = blended_value
      expected_jerk = math.copysign(delta * delta, delta)

      print(f"\nTesting planned_accel={planned_accel}")
      print(f"  Delta: {blended_value}, expected_jerk={expected_jerk}, actual_jerk={self.controller.state.jerk}")
      print(f"  Original jerk would be: {math.copysign(blended_value * blended_value, blended_value)}")
      print(f"  Jerk limits (upper/lower): {self.controller.jerk_upper:.4f}/{self.controller.jerk_lower:.4f}")

      self.assertAlmostEqual(self.controller.state.jerk, expected_jerk)

      # Check minimum jerk limits for small deltas
      if abs(blended_value) < 1.0:
        expected_min = self.controller.car_config.jerk_limits[0]
        if expected_jerk > 0:
          self.assertGreaterEqual(self.controller.jerk_upper, expected_min)
        else:
          self.assertGreaterEqual(self.controller.jerk_lower, expected_min)

  def test_a_value_jerk_scaling(self):
    """Test a_value"""
    mock_CC = Mock(enabled=True)
    mock_CC.actuators = Mock(accel=1.0)

    # Start
    self.controller.reset()

    result = self.controller.calculate_a_value(mock_CC)

    # Expected jerk number is always 5/50 = 0.1
    self.assertAlmostEqual(float(result), 0.1)

    # Second call with known state
    mock_CC.actuators.accel = 0.7
    second_result = self.controller.calculate_a_value(mock_CC)
    self.assertAlmostEqual(float(second_result), 0.2)  # Should be 0.1 + 0.1 = 0.2

  def test_calculate_accel(self):
    """Test calculate_accel method"""
    # Test disabled
    mock_CC = Mock(enabled=False)
    self.assertEqual(self.controller.calculate_accel(mock_CC), 0.0)

    # Test enabled
    mock_CC.enabled = True
    mock_CC.actuators = Mock(accel=2.0)

    with patch('opendbc.car.hyundai.values.CarControllerParams') as mock_params:
      mock_params.ACCEL_MIN, mock_params.ACCEL_MAX = -3.5, 2.0
      self.assertEqual(self.controller.calculate_accel(mock_CC), 2.0)

  def test_make_jerk_realistic_profile(self):
    """Test make_jerk with realistic velocity and acceleration profile"""
    self.controller.CP_SP.flags = HyundaiFlagsSP.LONG_TUNING_BRAKING

    # Generate test data
    np.random.seed(42)
    num_points = 20
    segments = [
      np.random.uniform(0.3, 0.8, num_points//4),     # Mild acceleration
      np.random.uniform(0.8, 1.6, num_points//4),     # Moderate acceleration
      np.random.uniform(-0.2, 0.2, num_points//4),    # Cruise
      np.random.uniform(-1.2, -0.5, num_points//8),   # Moderate deceleration
      np.random.uniform(-2.2, -1.2, num_points//8)    # Hard deceleration
    ]
    accelerations = np.concatenate(segments)[:num_points]

    # Calculate velocities
    velocities = np.zeros(len(accelerations))
    velocities[0] = 1.0  # Starting velocity
    for i in range(1, len(accelerations)):
        velocities[i] = velocities[i-1] + accelerations[i-1] * 0.2
    velocities = np.clip(velocities, 0.0, 30.0)

    # Setup mocks
    mock_CC, mock_CS = Mock(), Mock()
    mock_CC.actuators, mock_CS.out = Mock(), Mock()

    print("\n[test_make_jerk_realistic_profile] Testing velocity profile:")
    for i, (v, a) in enumerate(zip(velocities, accelerations, strict=True)):
      mock_CS.out.vEgo = float(v)
      mock_CS.out.aEgo = float(a)
      mock_CC.actuators.accel = float(a)

      self.controller.make_jerk(mock_CC, mock_CS, LongCtrlState.pid)

      print(f"  Step {i:2d}: v={v:5.2f} m/s, a={a:5.2f} m/s², jerk={self.controller.state.jerk:5.2f}, \
            jerk_upper={self.controller.jerk_upper:5.2f}, jerk_lower={self.controller.jerk_lower:5.2f}")

      # Verify minimum jerk limits based on velocity
      min_jerk = self.controller.car_config.jerk_limits[0]
      if v > 3.611:  # Above walking speed
        self.assertGreaterEqual(self.controller.jerk_upper, min_jerk)
        self.assertGreaterEqual(self.controller.jerk_lower, min_jerk)
      else:  # Low speed
        self.assertGreaterEqual(self.controller.jerk_upper, 0.6)

if __name__ == "__main__":
  unittest.main()
