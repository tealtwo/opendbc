import numpy as np


def catmull_rom_interp(x, xp, fp, alpha=0.5):
  """1D Catmull-Rom interpolation for vehicle dynamics.

  Args:
    x: The input value to interpolate at
    xp: Array of x-coordinates of points
    fp: Array of y-coordinates of points
    alpha: Parameter affecting tension (0.5=centripetal recommended)

  Returns:
    Interpolated value at x
  """
  # Handle boundary conditions
  if x <= xp[0]:
    return fp[0]
  elif x >= xp[-1]:
    return fp[-1]

  # Find the segment containing x
  i = np.searchsorted(xp, x) - 1
  i = max(0, min(i, len(xp)-2))

  # Get the 4 control points needed
  if i == 0:
    p0 = 2*fp[0] - fp[1]  # Extrapolate
  else:
    p0 = fp[i-1]

  p1 = fp[i]
  p2 = fp[i+1]

  if i+2 >= len(fp):
    p3 = 2*fp[-1] - fp[-2]  # Extrapolate
  else:
    p3 = fp[i+2]

  # Calculate parameter values using the chosen alpha
  t0 = 0.0
  t1 = t0 + abs(p1 - p0)**alpha
  t2 = t1 + abs(p2 - p1)**alpha
  t3 = t2 + abs(p3 - p2)**alpha
  t = t1 + (t2 - t1) * (x - xp[i]) / (xp[i+1] - xp[i])

  # Catmull-Rom basis functions
  s = (t - t1) / (t2 - t1)

  # Hermite basis functions
  h00 = 2*s**3 - 3*s**2 + 1
  h10 = s**3 - 2*s**2 + s
  h01 = -2*s**3 + 3*s**2
  h11 = s**3 - s**2

  # Calculate tangents
  m1 = (p2 - p0) / (t2 - t0) * (t2 - t1)
  m2 = (p3 - p1) / (t3 - t1) * (t2 - t1)

  return h00*p1 + h10*m1 + h01*p2 + h11*m2
