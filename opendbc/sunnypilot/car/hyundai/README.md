
# **How our custom longitudinal tuning works for Hyundai/Kia/Genesis vehicles.**

To start this readme, I would like to first present the safety guidelines followed to create the tune:

Our main safety guideline considered is [ISO 15622:2018](https://www.iso.org/obp/ui/en/#iso:std:iso:15622:ed-3:v1:en)
This provides the groundwork of safety limits this tune must adhere too, and therefore, must be followed.
For example, in our jerk calculations throughout this tune, you will see how jerk is clipped using the equation ISO standards.

In the tuning you will see a set of equations, the first being jerk, **but what exactly is jerk?**
Jerk is calculated by taking current acceleration (in the form of m/s^2), subtracting that by previous acceleration, and
dividing that by time. In our calculations we use 50hz as our timestep when calling the `make_jerk` method. In our tune you will see the following equation:

    current_accel = CS.out.aEgo
    self.state.jerk = (current_accel - self.state.accel_last_jerk) / 0.125
    self.state.accel_last_jerk = current_accel

For time, in this equation we are using 100hz which as a decimal is 0.125 to represent time for our calculations.
For example, lets say our current acceleration is 0.7 m/s^2 and our previous acceleration was 0.2 m/s^2; This would lead to us having 0.5 m/s^2 divided by 
0.125 (our timestep), which leads to a calculated jerk value of 4.0 m/s^3. This then goes through our minimum and maximum clipping which forces a value between our set min and max,
which I discuss later in this readme.

Moving on, the accel_last_jerk, stores current accel after each iteration and uses that in the calculation as previous accel for
our jerk calculations. Now we see the calculation of jerk max and jerk min. **Lets dive into how jerk lower limit max is calculated:**

     velocity = CS.out.vEgo
    if velocity < 5.0:
      decel_jerk_max = self.car_config.jerk_limits[1]
    elif velocity > 20.0:
      decel_jerk_max = 2.5
    else:
      decel_jerk_max = 5.83 - (velocity / 6)

This equation above is set by ISO 15622, and dictates that jerk lower limit can only be five when below 5 m/s. 
Between 5 m/s and 20 m/s jerk is capped using the calculation:

    5.83 - (current velocity / 6 )

This means that if current velocity is say, 15 m/s the final jerk value would be capped at 3.33 m/s^3. 
Anything above 20 m/s is capped to a lower jerk max of 2.5 m/s^3. This allows for a smoother jerk range, while complying to ISO standards to a tee.
The current jerk Lower Limit you will see in openpilot before this tune, is 5.0 m/s^3; Which as you can see from using the above calculation, 
the 5.0 m/s^3 technically does not comply with ISO standards at any speed above 5.0 m/s^3.
Having our jerk max be clipped to these values not only allows for better consistency with ISO standards, but also enables us to have a much smoother braking experience. 

**Getting into our next topic, I would like to explain how our minimum jerk was chosen.** 

Minimum jerk was chosen based off of the following guideline proposed by Handbook of Intellegent Vehicles (2012):
`Ride comfort may be sacrificed only under emergency conditions when vehicle and occupant safety consideration may preclude comfort.`

**The value of 0.6 m/s^3 as the lower limit was chosen based off of**
[Carlowitz et al. (2024).](https://www.researchgate.net/publication/382274551_User_evaluation_of_comfortable_deceleration_profiles_for_highly_automated_driving_Findings_from_a_test_track_study)
This research study identified the average lower jerk used in comfortable driving settings, which is 0.53 m/s^3 respectively. 
In this equation, the minimum has been rounded to 0.6 m/s^3 for maintainability and consistency. 


**Next, we have our acceleration smoothing**. This is limited to above 17 m/s for safety. This smoothing uses a catmull rom interpolation
which helps create smoother trajectory than linear interpolation. The equation for acceleration is set below:

    # Normal operation = above 17 m/s
    if CS.out.vEgo > 17.0 and target_accel < 0.01:
      brake_ratio = np.clip(abs(target_accel / self.car_config.accel_limits[0]), 0.0, 1.0)
      # Array comes from longitudinal_config.py, 1.0 = -3.5 accel, which will never be less than -3.5 EVER
      accel_rate_down = self.DT_CTRL * catmull_rom_interp(brake_ratio,
                                                          np.array([0.25, 0.5, 0.75, 1.0]),
                                                          np.array(self.car_config.brake_response))
      accel = max(target_accel, self.state.accel_last - accel_rate_down)
    else:
      accel = actuators.accel

As we can see, there are checks to ensure the Ego is above 17 m/s, and is currently applying negative acceleration (i.e., braking).
Then we use our accel limits, which vary slightly car by car but generally, the values will be about:

    self.car_config.brake_response=(1.25, 1.85, 2.55, 3.5)

This gets plugged into the equation as follows. Brake_ratio is our x value. This is the input value we interpolate at. Next, we have our
x coordinates which is the first numpy array:

    [0.25, 0.5, 0.75, 1.0]

This represents 25% brake force all the way to 100% brake force (also known as -0.875 m/s^2 and -3.5 m/s^2). Next we input our
y coordinates as the second numpy array. These are the self.car_config.brake_response:

    [1.25, 1.85, 2.55, 3.5]

These values represent our negative acceleration squared. These values are then extrapolated in a catmull rom interpolation 
calcuation using an alpha level of 0.5 centripetal. To read more about how catmull_rom interpolations work view the [interpolation_utils.py](opendbc_repo/opendbc/sunnypilot/interpolation_utils.py),
in the codebase, or click on this [catmull rom splines](https://qroph.github.io/2018/07/30/smooth-paths-using-catmull-rom-splines.html) hyperlink
to learn more.





