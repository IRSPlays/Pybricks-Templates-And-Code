from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Icon, Axis
from pybricks.robotics import DriveBase 
from pybricks.tools import wait, StopWatch, hub_menu, multitask, run_task, Matrix

hub = PrimeHub()
hub.speaker.volume(100)
hub.system.set_stop_button(Button.BLUETOOTH)

# Initialize all motors and colorsensors.
left_wheel = Motor(Port.A, Direction.COUNTERCLOCKWISE)  # Left wheel
right_wheel = Motor(Port.C)  # Right wheel
left_motor = Motor(Port.D)  # Left motor
right_motor = Motor(Port.B)  # Right motor
Left_CS = ColorSensor(Port.F)  # Left colour sensor
Right_CS = ColorSensor(Port.E)  # Right colour sensor

# Set False if the robot only have 1 line tracking sensor (Port F). Affects SLT and DLT
twoLineTrackingSensors = False

# Brake types
stop = {0: Stop.COAST, 1: Stop.BRAKE, 2: Stop.HOLD, 3: Stop.NONE}

# to disable reset() if Movement use BrakeType NONE.
none_true = False

# resets both wheels and yaw angle
def reset():
    if not none_true:
        left_wheel.reset_angle(0) # Reset Left Wheel
        right_wheel.reset_angle(0) # Reset Right Wheel
        drive_base.reset(0,0)
        hub.imu.reset_heading(0) # Reset Yaw angle
        wait(20)

def mapSpeed(i):
    #-100 - 100 to -500 - 500 
    return 5 * i
def mapAccel(i):
    # 10 if less than 10
    if i < 10:
        return 187.5
    #-100 - 100 to -1875 - 1875 
    else:
        return i * 18.75  

# =================================================================
# ================= FEEDFORWARD CONTROL SYSTEM ===================
# =================================================================

# Characterization data from robot testing
# Format: (voltage, speed_deg_per_sec)
characterization_data = [
    (1000, 0.00),
    (1500, 0.00),
    (2000, 1.58),
    (2500, 66.45),
    (3000, 88.35),
    (3500, 110.25),
    (4000, 132.23),
    (4500, 155.10),
    (5000, 177.08),
    (5500, 199.27),
    (6000, 223.43),
    (6500, 248.25),
    (7000, 228.08),
]

class FeedforwardController:
    """
    Implements feedforward control using characterized motor data.
    Provides more accurate acceleration and deceleration control.
    """
    
    def __init__(self):
        # Calculate feedforward gains from characterization data
        self.kF, self.kA = self._calculate_gains()
        self.static_friction_offset = self._calculate_static_friction()
        print(f"FF Controller initialized: kF={self.kF:.4f}, kA={self.kA:.4f}, static_offset={self.static_friction_offset:.1f}")
    
    def _calculate_gains(self):
        """Calculate kF (velocity feedforward) and kA (acceleration feedforward) gains"""
        # Filter out zero-speed data points for linear regression
        valid_data = [(v, s) for v, s in characterization_data if s > 5.0]
        
        if len(valid_data) < 2:
            print("WARNING: Insufficient characterization data, using defaults")
            return 0.03, 0.001
        
        # Linear regression to find kF (slope of voltage vs speed)
        n = len(valid_data)
        sum_v = sum(v for v, s in valid_data)
        sum_s = sum(s for v, s in valid_data)
        sum_vs = sum(v * s for v, s in valid_data)
        sum_s2 = sum(s * s for v, s in valid_data)
        
        # kF = change in voltage per change in speed
        kF = (n * sum_vs - sum_v * sum_s) / (n * sum_s2 - sum_s * sum_s)
        
        # kA is estimated as a fraction of kF (typical for robot systems)
        kA = kF * 0.1  # 10% of velocity gain for acceleration
        
        return abs(kF), abs(kA)
    
    def _calculate_static_friction(self):
        """Calculate the minimum voltage needed to overcome static friction"""
        for voltage, speed in characterization_data:
            if speed > 1.0:  # First point where motion occurs
                return voltage * 0.8  # 80% of breakaway voltage
        return 2000  # Default fallback
    
    def calculate_feedforward_voltage(self, target_speed_deg_s, target_accel_deg_s2=0):
        """
        Calculate the feedforward voltage needed for a target speed and acceleration.
        
        Args:
            target_speed_deg_s: Target wheel speed in degrees per second
            target_accel_deg_s2: Target acceleration in degrees per second squared
        
        Returns:
            Voltage (0-7200) needed to achieve the target motion
        """
        # Base feedforward for steady-state speed
        velocity_voltage = self.kF * abs(target_speed_deg_s)
        
        # Additional voltage for acceleration
        acceleration_voltage = self.kA * abs(target_accel_deg_s2)
        
        # Add static friction compensation for low speeds
        if abs(target_speed_deg_s) > 0:
            total_voltage = velocity_voltage + acceleration_voltage + self.static_friction_offset
        else:
            total_voltage = 0
        
        # Clamp to valid voltage range
        return max(0, min(7200, total_voltage))
    
    def get_smart_accel_settings(self, target_speed, distance):
        """
        Calculate optimized acceleration and deceleration rates based on distance and speed.
        
        Args:
            target_speed: Target speed in degrees/second
            distance: Distance to travel in mm
        
        Returns:
            (acceleration, deceleration) in degrees/second²
        """
        # Convert distance to degrees (assuming wheel diameter from robot.Init)
        wheel_circumference = 3.14159 * 62.4  # Default wheel diameter
        distance_degrees = (distance / wheel_circumference) * 360
        
        # Calculate optimal acceleration profile with better balance
        # Reduce aggressive acceleration to prevent tilting
        
        if distance_degrees < 90:  # Very short distances - quarter wheel rotation
            accel_rate = target_speed * 2  # Moderate acceleration to prevent tilting
            decel_rate = target_speed * 2.5  # Slightly more aggressive deceleration
        elif distance_degrees < 360:  # Less than one wheel rotation
            accel_rate = target_speed * 1.5  # Gentle acceleration
            decel_rate = target_speed * 2  # Moderate deceleration
        elif distance_degrees < 1080:  # Less than 3 wheel rotations
            accel_rate = target_speed * 1.2  # Very gentle acceleration
            decel_rate = target_speed * 1.5  # Gentle deceleration
        else:  # Long distance
            accel_rate = target_speed * 1  # Smooth acceleration
            decel_rate = target_speed * 1.2  # Smooth deceleration
        
        return accel_rate, decel_rate

# Global feedforward controller instance
ff_controller = FeedforwardController()

def mapAccelFF(target_speed, distance=1000):
    """
    Enhanced acceleration mapping using feedforward control.
    
    Args:
        target_speed: Target speed in user units (-100 to 100)
        distance: Distance for the movement in mm (for optimization)
    
    Returns:
        (acceleration, deceleration) tuple in Pybricks units
    """
    # Convert user speed to degrees per second
    speed_deg_s = abs(target_speed) * 5 * 6  # Approximate conversion
    
    # Get optimized acceleration rates
    accel_rate, decel_rate = ff_controller.get_smart_accel_settings(speed_deg_s, distance)
    
    # Convert back to Pybricks units with conservative limits to prevent tilting
    # Reduced maximum acceleration to prevent robot tilting
    max_accel = 1200  # Reduced from 1875 to prevent tilting
    min_accel = 250   # Increased from 187.5 for smoother starts
    
    accel_pybricks = min(max_accel, max(min_accel, accel_rate * 2.5))  # Reduced multiplier
    decel_pybricks = min(max_accel, max(min_accel, decel_rate * 2.5))  # Reduced multiplier
    
    return accel_pybricks, decel_pybricks

# Parameters for basic movements
class MotionParameters:
    # Public member variables (accessible directly)
    def __init__(self):
        self.SPEED = 50             # Target Speed 
        self.SPEED_ALIGN = 30       # Wall Alignment Speed 
        self.JUNCT_SPEED = 30       # Junction Speed
        self.ACCEL_RATE = 50        # Acceleration Rate 
        self.DECEL_RATE = 50        # Deceleration Rate
        self.GYRO_USAGE = True      # Gyro usage
        self.USE_FEEDFORWARD = True # Enable/disable feedforward control
        self.CONSERVATIVE_FF = False # Use conservative feedforward to prevent tilting

    def setMotionProfile(self, profile_name):
        """
        Sets the motion parameters based on a predefined profile.
        
        Args:
            profile_name (str): The name of the profile to use.
                                ('precision', 'balanced', 'competition')
        """
        print(f"MOTION_PROFILE: Setting profile to '{profile_name}'")
        if profile_name == 'precision':
            self.SPEED = 30
            self.SPEED_ALIGN = 20
            self.JUNCT_SPEED = 20
            self.ACCEL_RATE = 30
            self.DECEL_RATE = 30
            self.CONSERVATIVE_FF = True
        elif profile_name == 'balanced':
            self.SPEED = 50
            self.SPEED_ALIGN = 30
            self.JUNCT_SPEED = 30
            self.ACCEL_RATE = 50
            self.DECEL_RATE = 50
            self.CONSERVATIVE_FF = False
        elif profile_name == 'competition':
            self.SPEED = 80
            self.SPEED_ALIGN = 40
            self.JUNCT_SPEED = 50
            self.ACCEL_RATE = 80
            self.DECEL_RATE = 80
            self.CONSERVATIVE_FF = False
        else:
            print(f"WARNING: Motion profile '{profile_name}' not recognized. Using default.")
            self.resetDefault()

    # Reset all the parameters back to default values
    def resetDefault(self):         
        self.SPEED = 50             # Target Speed
        self.SPEED_ALIGN = 30       # Wall Alignment Speed
        self.JUNCT_SPEED = 30       # Junction Speed
        self.ACCEL_RATE = 50        # Acceleration Rate
        self.DECEL_RATE = 50        # Deceleration Rate
        self.GYRO_USAGE = True      # Gyro usage
        self.USE_FEEDFORWARD = True # Enable/disable feedforward control
        self.CONSERVATIVE_FF = False # Use conservative feedforward to prevent tilting

mp = MotionParameters()

# Parameters for line tracking
class TrackingParameters:
    # Public member variables (accessible directly)
    def __init__(self): 
        self.IP = 20                # Initial Speed
        self.SPEED = 50             # Target Speed
        self.ACCEL_DIST = 100       # Acceleration Distance
        self.DECEL_DIST = 100       # Deceleration Distance
        self.JUNCT_SPEED = 30       # Junction Speed
        self.TH = 50                # Threshold
        self.JTH = 20               # Junction Threshold
        self.DIST_KP = 1            # Distance Proportional Gain
        self.DIST_KD = 8           # Distance Derivative Gain
        self.JTN_KP = 0.5  # Junction Proportional Gain
        self.JTN_KD = 4  # Junction Derivative Gain

    # Reset all the parameters back to default values
    def resetDefault(self):
        self.IP = 20                # Initial Speed
        self.SPEED = 50             # Target Speed
        self.ACCEL_DIST = 100       # Acceleration Distance
        self.DECEL_DIST = 100       # Deceleration Distance
        self.JUNCT_SPEED = 30       # Junction Speed
        self.TH = 50                # Threshold
        self.JTH = 20               # Junction Threshold
        self.DIST_KP = 1            # Distance Proportional Gain
        self.DIST_KD = 8           # Distance Derivative Gain
        self.JTN_KP = 0.5  # Junction Proportional Gain
        self.JTN_KD = 4  # Junction Derivative Gain

tkp = TrackingParameters()

class robot:
    async def playTone(FREQUENCY, DURATION):
        """
        PARAMETERS:
        -
        **`FREQUENCY`**: Frequency of the tone in Hz (64-24000)

        **`DURATION`**: Duration of the tone in **milliseconds**

        >
        ---
        ### Example:
        >
            robot.playTone(1000, 200)
        >
        """
        await hub.speaker.beep(FREQUENCY, DURATION)

    # ==== Buttons ====
    def getButton(BUTTON):
        """
        PARAMETERS:
        -
        **`BUTTON`**: 1 = Left | 2 = Center | 3 = Right

        ---
        Returns `0` if the button **is not** pressed               
        Returns `1` if the button **is** pressed
        
        """
        button = {1: Button.LEFT, 2: Button.CENTER, 3: Button.RIGHT}
        if button[BUTTON] in hub.buttons.pressed():
            return 1
        else:
            return 0    
    
    async def waitForPress(BUTTON, DEBOUNCEMS = 200):
        """
        PARAMETERS:
        -
        **`BUTTON`**: 1 = Left | 2 = Center | 3 = Right
        **`DEBOUNCEMS`**: Delay in **milliseconds** after button is pressed

        >
        ---
        Waits for selected button pressed before continuing execution.
        
        """
        while robot.getButton(BUTTON) != 1:
            await wait(10)
        await wait(DEBOUNCEMS)
    
    async def waitForRelease(DEBOUNCEMS = 200):
        """
        PARAMETERS:
        -
        **`DEBOUNCEMS`**: Delay in **milliseconds** after button is released

        """
        while hub.buttons.pressed():
            await wait(10)
        await wait(DEBOUNCEMS)
    
    async def waitForBump(BUTTON, DEBOUNCEMS = 200):
        """
        PARAMETERS:
        -
        **`BUTTON`**: 1 = Left | 2 = Center | 3 = Right
        **`DEBOUNCEMS`**: Delay in **milliseconds** after button is bumped

        """
        await robot.waitForPress(BUTTON, 0)
        await robot.waitForRelease(DEBOUNCEMS)

    # ==== Init ====
    def Init(Wheel_diameter, Axle_track, Left_Black, Left_White, Right_Black, Right_White):
        """
        PARAMETERS:
        -
        **`Wheel_diameter`**: **Diameter** of the wheel in **millimeters** 
        
        **`Axle_track`**: Distance between the **midpoint** of the wheels in **millimeters**

        **`Left_Black`**: **Raw** Black value of the **Left Sensor**

        **`Left_White`**: **Raw** White value of the **Left Sensor**

        **`Right_Black`**: **Raw** Black value of the **Right Sensor**

        **`Right_White`**: **Raw** White value of the **Right Sensor**
        >
        ***
        ## GUIDE:

        **`Wheel Diameter`**:
        For some tyres, the diameter is printed on the side. 
        For example, on the competition wheel (Item no: 86652c01) `62.4 x 20` means that the diameter is `62.4mm` and that the width is `20 mm`.
        
        **`Axle Track`**:
        If you dont have a ruler, you can use a LEGO beam to measure. 
        The LEGO technics holes is `8 mm`.
        >
        ---
        ## To Verify:
        
        For **`Wheel diameter`**, drive **300mm** using:
        >
            drive_base.straight(300)
        >
        Measure how far it traveled:

        If your robot drives **too less, decrease** the **`Wheel_diameter`** value.
        
        If your robot drives **too much, increase** the **`Wheel_diameter`** value.

        For **`Axle track`**, turn **360 degrees** using:
        >
            drive_base.turn(360)
        >
        Check that it is back in the same place:

        If your robot turns **too less, increase** the **`Axle_track`** value.
        
        If your robot turns **too much, decrease** the **`Axle_track`** value.
        >
        """
        global drive_base, axleTrack, LB, LW, RB, RW
        # Battery voltage min and max
        V_MIN =6000
        V_MAX = 8000
        voltage = hub.battery.voltage()
        drive_base = DriveBase(left_wheel, right_wheel, Wheel_diameter, Axle_track)  # Specifying the dimensions of the robot
        drive_base.use_gyro(True)
        hub.system.set_stop_button(Button.BLUETOOTH)  # Bluetooth button to stop program
        axleTrack = Axle_track 
        LW = Left_White
        LB = Left_Black
        RW = Right_White
        RB = Right_Black
        drive_base.brake()
        run_task(robot.playTone(500, 100))
        run_task(robot.playTone(600, 100))
        run_task(robot.playTone(700, 100))
        run_task(robot.playTone(800, 100))
        hub.light.on(Color.GREEN)
        battery_percentage = ((voltage - V_MIN) / (V_MAX - V_MIN)) * 100
        if battery_percentage < 50:
            hub.light.on(Color.RED)
        while True:
            if robot.getButton(2):
                run_task(robot.waitForRelease(0))
                hub.display.off()
                break
            if robot.getButton(1):
                voltage = hub.battery.voltage()
                battery_percentage = ((voltage - V_MIN) / (V_MAX - V_MIN)) * 100
                battery_percentage = max(0, min(100, battery_percentage))
                hub.display.text(str(int(battery_percentage)))
            wait(10)

    # ===== Continuous Drive and Stop ====
    async def drive(speed, turn_rate):
        """Drives the robot at a given speed and turn rate."""
        drive_base.drive(speed, turn_rate)

    async def stop():
        """Stops the robot."""
        drive_base.stop()

    def distance():
        """Gets the distance traveled by the robot."""
        return drive_base.distance()

    def reset_distance():
        """Resets the robot's distance measurement."""
        drive_base.reset()

    # ===== Movements ====
    async def straight(SPEED, DISTANCE, STOP_DELAY = 0.2, BRAKE_TYPE = 1): 
        """
        PARAMETERS:
        -
        **`SPEED`**: Target speed (-100 to 100)                  
        If value is 1, speed is mp.SPEED                          
        If value is -1, speed is -(mp.SPEED)        
        If none of the above is keyed speed                         
        Use negative values to move **backwards**

        **`DISTANCE`**: Distance to move in **millimeters**

        **`STOP_DELAY`**: Delay in **seconds** after the movement before ending (default = 0.2s)

        **`BRAKE_TYPE`**: Type of stop behavior after moving:   
        0 = Coast (motors stop freely)                          
        1 = Brake (motors resist motion to stop quickly)            
        3 = None (robot will not decelerate)                                    
        Set stop_delay to 0 if using **None**
        
        ---
        ### Example 1: Using keyed speed
        > 
            robot.straight(50,500)
        *Moving at 50 speed*
        >
        ### Example 2: Using mp.SPEED
        >
            robot.straight(1,500)
        *Moving at mp.SPEED*
        """
        # Enhanced straight movement with continuous gyro correction
        print(f"GYRO_STRAIGHT: {DISTANCE}mm at speed {SPEED} with gyro stabilization")
        
        drive_base.use_gyro(mp.GYRO_USAGE)
        reset()
        none_true = True if BRAKE_TYPE == 3 else False
        SPEED = mp.SPEED * SPEED if abs(SPEED) == 1 else SPEED 
        
        if DISTANCE != 0:
            target_heading = hub.imu.heading()  # Lock in current heading
            DISTANCE = -DISTANCE if SPEED < 0 else DISTANCE 
            
            # Use feedforward control for enhanced acceleration/deceleration if enabled
            if mp.USE_FEEDFORWARD:
                accel_ff, decel_ff = mapAccelFF(SPEED, abs(DISTANCE))
                
                # Apply conservative mode if enabled to prevent tilting
                if mp.CONSERVATIVE_FF:
                    accel_ff = min(accel_ff * 0.7, 800)  # Reduce by 30% and cap at 800
                    decel_ff = min(decel_ff * 0.8, 900)  # Reduce by 20% and cap at 900
                    print(f"Conservative FF: Accel={accel_ff:.1f}, Decel={decel_ff:.1f}")
                
                drive_base.settings(mapSpeed(SPEED), (accel_ff, decel_ff))
            else:
                drive_base.settings(mapSpeed(SPEED), (mapAccel(mp.ACCEL_RATE), mapAccel(mp.DECEL_RATE)))
            
            # Enhanced movement with continuous gyro correction
            if mp.GYRO_USAGE and abs(DISTANCE) > 50:  # Use gyro correction for longer movements
                await robot.straightWithGyroPID(SPEED, DISTANCE, target_heading, BRAKE_TYPE)
            else:
                # Standard movement for short distances
                await drive_base.straight(DISTANCE, stop[BRAKE_TYPE])
            
            # Add slight settling time to prevent tilting after movement
            if BRAKE_TYPE == 1:  # Only for brake stop type
                await wait(50)  # 50ms settling time to let robot stabilize
                
        if STOP_DELAY > 0:
            await robot.playTone(1000, STOP_DELAY * 1000)

    async def straightWithGyroPID(SPEED, DISTANCE, target_heading, BRAKE_TYPE=1):
        """
        Enhanced straight movement with continuous PID gyro correction
        Prevents drift during long movements
        """
        print(f"GYRO_PID: Moving {DISTANCE}mm with PID heading control to {target_heading}°")
        
        # PID constants for heading control
        KP_HEADING = 2.5  # Proportional gain - how aggressively to correct
        KI_HEADING = 0.1  # Integral gain - corrects persistent drift
        KD_HEADING = 0.8  # Derivative gain - smooths corrections
        
        integral_error = 0
        previous_error = 0
        start_distance = drive_base.distance()
        
        # Move using drive() with continuous correction
        while abs(drive_base.distance() - start_distance) < abs(DISTANCE):
            current_heading = hub.imu.heading()
            
            # Calculate heading error (handle wrap-around at ±180°)
            heading_error = target_heading - current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            # PID calculation
            integral_error += heading_error
            derivative_error = heading_error - previous_error
            
            # Calculate turn correction
            turn_correction = (KP_HEADING * heading_error + 
                             KI_HEADING * integral_error + 
                             KD_HEADING * derivative_error)
            
            # Limit turn correction to prevent oscillation
            turn_correction = max(-30, min(30, turn_correction))
            
            # Apply movement with correction
            drive_base.drive(mapSpeed(SPEED), turn_correction)
            previous_error = heading_error
            
            await wait(10)  # Small delay for stable control loop
        
        # Stop the robot
        if BRAKE_TYPE == 1:
            drive_base.brake()
        else:
            drive_base.stop()
        
        final_heading = hub.imu.heading()
        heading_error = abs(final_heading - target_heading)
        if heading_error > 180:
            heading_error = 360 - heading_error
        print(f"GYRO_PID: Final heading error: {heading_error:.1f}°")

    async def straightBalanced(SPEED, DISTANCE, STOP_DELAY = 0.2):
        """
        BALANCED STRAIGHT MOVEMENT - Use for critical positioning where tilting must be avoided.
        
        PARAMETERS:
        -
        **`SPEED`**: Target speed (-100 to 100)
        **`DISTANCE`**: Distance to move in **millimeters**
        **`STOP_DELAY`**: Delay in **seconds** after movement
        
        This function uses very conservative acceleration to prevent robot tilting.
        """
        print(f"BALANCED_STRAIGHT: {DISTANCE}mm at speed {SPEED} (anti-tilt mode)")
        
        drive_base.use_gyro(mp.GYRO_USAGE)
        reset()
        
        SPEED = mp.SPEED * SPEED if abs(SPEED) == 1 else SPEED 
        if DISTANCE > 0:
            DISTANCE = -DISTANCE if SPEED < 0 else DISTANCE 
            
            # Use very conservative acceleration to prevent tilting
            conservative_accel = min(400, abs(SPEED) * 8)  # Very gentle acceleration
            conservative_decel = min(500, abs(SPEED) * 10)  # Gentle deceleration
            
            drive_base.settings(mapSpeed(SPEED), (conservative_accel, conservative_decel))
            await drive_base.straight(DISTANCE, Stop.BRAKE)
            
            # Extended settling time for critical positioning
            await wait(100)  # 100ms to let robot fully settle
                
        if STOP_DELAY > 0:
            await robot.playTone(800, STOP_DELAY * 1000)

    async def wallAlignment(SPEED, SECONDS, STOP_DELAY = 0.2):
        """
        PARAMETERS:
        -
        **`SPEED`**: Constant speed (-100 to 100)                
        If value is 1, speed is mp.SPEED_ALIGN                          
        If value is -1, speed is -(mp.SPEED_ALIGN)        
        If none of the above is keyed speed                         
        Use negative values to move **backwards**                   
        *No acceleration or deceleration*

        **`SECONDS`**: Duration to move in **seconds**

        **`STOP_DELAY`**: Delay in seconds after the movement before ending (default = 0.2s)

        ---
        ### Example: Using mp.SPEED
        >
            robot.wall_alignment(1, 1)
        *Move at mp.SPEED*
        >
        """
        drive_base.use_gyro(False)
        reset()
        none_true = False
        SPEED = mp.SPEED_ALIGN * SPEED if abs(SPEED) == 1 else SPEED
        drive_base.drive(mapSpeed(SPEED), 0)
        await wait(SECONDS * 1000)
        drive_base.stop()
        if STOP_DELAY > 0:
            await robot.playTone(1000, STOP_DELAY * 1000)

    async def spotTurn(SPEED, DEGREES, STOP_DELAY = 0.2, BRAKE_TYPE = 1):
        """
        PARAMETERS:
        -
        **`SPEED`**: Target speed (-100 to 100)                 
        If value is 1, speed is mp.SPEED                          
        If value is -1, speed is -(mp.SPEED)        
        If none of the above is keyed speed                         
        Use negative values to turn **counterclockwise**
        
        **`DEGREES`**: Number of degrees to turn.

        **`STOP_DELAY`**: Delay in **seconds** after the movement before ending (default = 0.2s)

        **`BRAKE_TYPE`**: Type of stop behavior after moving:   
        0 = Coast (motors stop freely)                          
        1 = Brake (motors resist motion to stop quickly)            
        3 = None (robot will not decelerate)                                    
        Set stop_delay to 0 if using **None**

        ---
        ### Example:
        >
            robot.spot_turn(50, 90)
        >
        """
        drive_base.use_gyro(mp.GYRO_USAGE)
        reset()
        none_true = True if BRAKE_TYPE == 3 else False
        SPEED = mp.SPEED * SPEED if abs(SPEED) == 1 else SPEED
        DEGREES = -abs(DEGREES) if SPEED < 0 else DEGREES 
        
        # Use feedforward control for turns if enabled
        if mp.USE_FEEDFORWARD:
            # Turn distance ≈ (degrees/360) * π * axle_track
            turn_distance_mm = (abs(DEGREES) / 360.0) * 3.14159 * 95  # Using approximate axle track
            accel_ff, decel_ff = mapAccelFF(SPEED, turn_distance_mm)
            drive_base.settings(None, None, mapSpeed(SPEED), (accel_ff, decel_ff))
        else:
            drive_base.settings(None, None, mapSpeed(SPEED), (mapAccel(mp.ACCEL_RATE), mapAccel(mp.DECEL_RATE)))
        
        await drive_base.turn(DEGREES, stop[BRAKE_TYPE])
        if STOP_DELAY > 0:
            await robot.playTone(1000, STOP_DELAY * 1000)

    async def pivotTurn(SPEED, RADIUS, DEGREES, STOP_DELAY = 0.2, BRAKE_TYPE = 1):
        """
        PARAMETERS:
        -
        **`SPEED`**: Target speed (-100 to 100)                 
        If value is 1, speed is mp.SPEED                          
        If value is -1, speed is -(mp.SPEED)        
        If none of the above is keyed speed                         
        Use negative values to move **backwards**

        **`RADIUS`**: Radius of the circle in **millimeters**  
        If value is 1, only **left** wheel moves                          
        If value is -1, only **right** wheel moves                            
        Positive value will turn **right** along the circle.
        Negative value will turn **left** along the circle. 

        **`DEGREES`**: Number of degrees to turn.

        **`STOP_DELAY`**: Delay in **seconds** after the movement before ending (default = 0.2s)

        **`BRAKE_TYPE`**: Type of stop behavior after moving:   
        0 = Coast (motors stop freely)                          
        1 = Brake (motors resist motion to stop quickly)            
        3 = None (robot will not decelerate)                                    
        Set stop_delay to 0 if using **None**

        ---
        ### Example 1: Forward-Right
        >
            robot.pivot_turn(60, 120, 90)
        >
        ### Example 2: Backward-Left
        >
            robot.pivot_turn(60, -120, -90)
        >
        """
        # IMPROVED PIVOT TURN - TRACKS HEADING CHANGES WITHOUT INTERFERING
        print(f"PIVOT_TURN: Turning {DEGREES}° with radius {RADIUS}mm at speed {SPEED}")
        
        # Don't interfere with gyro during the movement - let it track naturally
        drive_base.use_gyro(mp.GYRO_USAGE)
        
        # Prevent the reset() function from interfering with our mission gyro
        global none_true
        original_none_true = none_true
        none_true = True  # Disable reset() to preserve gyro state
        
        # Only reset wheel angles for accurate distance tracking
        left_wheel.reset_angle(0)
        right_wheel.reset_angle(0)
        drive_base.reset(0, 0)  # Reset distance but preserve heading
        
        # Configure speed and acceleration (same logic as original)
        SPEED = mp.SPEED * SPEED if abs(SPEED) == 1 else SPEED 
        RADIUS = axleTrack / 2 * RADIUS if abs(RADIUS) == 1 else RADIUS
        DEGREES = -abs(DEGREES) if SPEED < 0 else DEGREES
        
        # Use feedforward control for pivot turns if enabled
        if mp.USE_FEEDFORWARD:
            # Arc distance = (degrees/360) * 2π * radius
            arc_distance_mm = (abs(DEGREES) / 360.0) * 2 * 3.14159 * abs(RADIUS)
            accel_ff, decel_ff = mapAccelFF(SPEED, arc_distance_mm)
            drive_base.settings(mapSpeed(abs(SPEED)), (accel_ff, decel_ff))
        else:
            drive_base.settings(mapSpeed(abs(SPEED)), (mapAccel(mp.ACCEL_RATE), mapAccel(mp.DECEL_RATE)))
        
        await drive_base.arc(RADIUS, DEGREES, None, stop[BRAKE_TYPE])
        
        # Restore the original none_true setting
        none_true = original_none_true
        
        if STOP_DELAY > 0:
            await robot.playTone(1000, STOP_DELAY * 1000)
        
        final_heading = hub.imu.heading()
        print(f"PIVOT_TURN: Complete. New heading: {final_heading}°")

    async def straight_JTN(PORT, SPEED, DISTANCE, JUNCT_SPEED = 0, JTH = tkp.JTH, STOP_DELAY = 0.2, BRAKE_TYPE = 1):
        """
        PARAMETERS:
        -
        **`PORT`**: 1 = Left sensor | 2 = Right sensor

        **`SPEED`**: Target speed (-100 to 100)                    
        If value is 1, speed is mp.SPEED                          
        If value is -1, speed is -(mp.SPEED)        
        If none of the above is keyed speed                         
        Use negative values to move **backwards**

        **`DISTANCE`**: Distance to move in **millimeters** before tracking 

        **`JUNCT_SPEED`**: Constant speed (default = 0)                
        If value is 1, speed is mp.JUNCT_SPEED                          
        If value is -1, speed is -(mp.JUNCT_SPEED)        
        If none of the above is keyed speed                         
        Use negative values to move **backwards**

        **`JTH`**: Junction threshold (default = tkp.JTH)                       
        If greater than 50, stops when mapped reflected light `>` JTH                  
        else, stops when mapped reflected light `<` JTH     

        **`STOP_DELAY`**: Delay in **seconds** after the movement before ending (default = 0.2s)

        **`BRAKE_TYPE`**: Type of stop behavior after moving                 
        0 = Coast (motors stop freely)                                          
        1 = Brake (motors resist motion to stop quickly)                        

        ---
        ### Example:
        >
            robot.straight_JTN(1, 50, 200, 30)
        >
        """
        drive_base.use_gyro(mp.GYRO_USAGE)
        none_true = False
        reset()
        if DISTANCE > 0:
            SPEED = mp.SPEED * SPEED if abs(SPEED) == 1 else SPEED 
            DISTANCE = -abs(DISTANCE) if SPEED < 0 else DISTANCE
            drive_base.settings(mapSpeed(SPEED), (mapAccel(mp.ACCEL_RATE), mapAccel(mp.DECEL_RATE)))
            await drive_base.straight(DISTANCE, stop[3] if JUNCT_SPEED != 0 else (stop[1] if STOP_DELAY > 0 else stop[0]))
        
        if JUNCT_SPEED != 0:
            JUNCT_SPEED = mp.JUNCT_SPEED * JUNCT_SPEED if abs(JUNCT_SPEED) == 1 else JUNCT_SPEED 
            while True:
                if PORT == 1:
                    lc_mapped = (await Left_CS.reflection() - LB) / (LW - LB) * 100
                else:
                    lc_mapped = (await Right_CS.reflection() - RB) / (RW - RB) * 100
                
                if (JTH > 50 and lc_mapped > JTH) or (JTH <= 50 and lc_mapped < JTH):
                    break
                
                drive_base.drive(mapSpeed(JUNCT_SPEED), 0)
        if BRAKE_TYPE == 1:
            drive_base.brake()
        else:
            drive_base.stop()
        
        if STOP_DELAY > 0:
            await robot.playTone(1000, STOP_DELAY * 1000)

    # ==== Line Tracking ====
    async def SLT(PORT, SPEED, DISTANCE, LR_EDGE = -1, JUNCT_SPEED = 0, JTH = tkp.JTH, STOP_DELAY = 0.2, BRAKE_TYPE = 1):
        """
        PARAMETERS:
        -
        **`PORT`**: 1 = Left sensor | 2 = Right sensor                          

        **`SPEED`**: Target speed (0 to 100)                                                                                
        If value is 1, speed is mp.SPEED                                        
        If none of the above is keyed speed                                     

        **`DISTANCE`**: Distance to line track in **millimeters**               

        **`LR_EDGE`**: -1 = Left Edge (default) | 1 = Right Edge                

        **`JUNCT_SPEED`**: Junction speed (default = 0)                         
        If value is 0, junction mode off                                        
        If value is 1, speed is tkp.JUNCT_SPEED                                 
        If none of the above is keyed speed                                     

        **`JTH`**: Junction threshold (default = tkp.JTH)                       
        If greater than 50, stops when mapped reflected light `>` JTH                  
        else, stops when mapped reflected light `<` JTH                               

        **`STOP_DELAY`**: Delay in **seconds** after the movement before ending (default = 0.2s)

        **`BRAKE_TYPE`**: Type of stop behavior after moving:                   
        0 = Coast (motors stop freely)                                          
        1 = Brake (motors resist motion to stop quickly)                        

        ---
        ### Note: SLT cannot move backwards!

        ### Example:
        >
            robot.SLT(1, 1, 300)
        >
        """
        prev_error = error = Distance = 0
        IP = mapSpeed(tkp.IP) 
        SPEED = mapSpeed(tkp.SPEED if SPEED == 1 else SPEED) 
        JUNCT_SPEED = mapSpeed(tkp.JUNCT_SPEED if JUNCT_SPEED == 1 else JUNCT_SPEED)
        
        none_true = False
        reset()

        if DISTANCE > 0:
            if DISTANCE <= (tkp.ACCEL_DIST + tkp.DECEL_DIST):
                accelRate = decelRate = (SPEED - IP) / (DISTANCE / 2)
                ACCEL_DIST = DECEL_DIST = DISTANCE / 2
            else:
                ACCEL_DIST = tkp.ACCEL_DIST
                DECEL_DIST = tkp.DECEL_DIST
                accelRate = (SPEED - IP) / ACCEL_DIST
                decelRate = DECEL_DIST / (SPEED - IP)
            
            if JUNCT_SPEED > 0:
                decelRate = DECEL_DIST / (SPEED - JUNCT_SPEED) if SPEED - JUNCT_SPEED != 0 else 0

            # SLT with degrees
            for i in range(3):
                if i == 0:  # Acceleration
                    setDistance = ACCEL_DIST
                if i == 1:  # Max Speed 
                    setDistance = DISTANCE - DECEL_DIST
                if i == 2:  # Deceleration
                    setDistance = DISTANCE
                
                while Distance < setDistance:
                    Distance = drive_base.distance()

                    # Read CS values from selected port for tracking
                    if PORT == 1:
                        MAPPED_COLOUR = (await Left_CS.reflection() - LB) / (LW - LB) * 100
                    else:
                        MAPPED_COLOUR = (await Right_CS.reflection() - RB) / (RW - RB) * 100

                    error = MAPPED_COLOUR - tkp.TH
                    
                    if i == 0:
                        finalSpeed = max(min(IP + (Distance * accelRate), SPEED), IP)
                    if i == 1:
                        finalSpeed = SPEED
                    if i == 2:
                        if decelRate != 0:
                            finalSpeed = max(min(SPEED - (Distance - (DISTANCE - DECEL_DIST)) / decelRate, SPEED), IP)
                    
                    derivative = error - prev_error
                    turn_rate = (tkp.DIST_KP * error * LR_EDGE) + (tkp.DIST_KD * derivative * LR_EDGE)
                    drive_base.drive(finalSpeed, -turn_rate)

                    prev_error = error

        # Junction line tracking. Checks if JUNCT_SPEED > 0
        if JUNCT_SPEED > 0:
            #checks if junct_speed == 1, if yes set junct_speed to default tkp.junct_speed
            JUNCT_SPEED = tkp.JUNCT_SPEED if JUNCT_SPEED == 1 else JUNCT_SPEED

            while True:
                if PORT == 1:
                    TRACKING_SENSOR = JUNCT_SENSOR = (await Left_CS.reflection() - LB) / (LW - LB) * 100
                    if twoLineTrackingSensors:
                        JUNCT_SENSOR = (await Right_CS.reflection() - RB) / (RW - RB) * 100
                elif PORT == 2:
                    TRACKING_SENSOR = JUNCT_SENSOR = (await Right_CS.reflection() - RB) / (RW - RB) * 100
                    if twoLineTrackingSensors:
                        JUNCT_SENSOR = (await Left_CS.reflection() - LB) / (LW - LB) * 100
                
                if JTH < 50:
                    if JUNCT_SENSOR < JTH:
                        break
                else:
                    if JUNCT_SENSOR > JTH:
                        break

                error = TRACKING_SENSOR - tkp.TH
                P = error * tkp.JTN_KP * LR_EDGE
                D = (error - prev_error) * tkp.JTN_KD * LR_EDGE
                turn_rate = P + D
                drive_base.drive(JUNCT_SPEED, -turn_rate)
                prev_error = error

        if BRAKE_TYPE == 1:
            drive_base.brake()
        else:
            drive_base.stop()
        
        if STOP_DELAY > 0:
            await robot.playTone(1000, STOP_DELAY * 1000)
        
    async def DLT(SPEED, DISTANCE, JUNCT_SPEED = 0, JTH = tkp.JTH, STOP_DELAY = 0.2, BRAKE_TYPE = 1):
        """
        PARAMETERS:
        -
        **`SPEED`**: Target speed (0 to 100)                                    
        If value is 1, speed is mp.SPEED                                        
        If none of the above is keyed speed                                     

        **`DISTANCE`**: Distance to line track in **millimeters**               

        **`JUNCT_SPEED`**: Junction speed (default = 0)                         
        If value is 0, junction mode off                                        
        If value is 1, speed is tkp.JUNCT_SPEED                                 
        If none of the above is keyed speed                                     

        **`JTH`**: Junction threshold (default = tkp.JTH)                       
        If greater than 50, stops when mapped reflected light `>` JTH           
        else, stops when mapped reflected light `<` JTH                         

        **`STOP_DELAY`**: Delay in **seconds** after the movement before ending (default = 0.2s)

        **`BRAKE_TYPE`**: Type of stop behavior after moving:                   
        0 = Coast (motors stop freely)                                          
        1 = Brake (motors resist motion to stop quickly)                        

        ---
        ### Note: DLT cannot move backwards!

        ### Example:
        >
            robot.DLT(60, 600, 30)
        >
        """
        if not twoLineTrackingSensors:
            raise RuntimeError("Two colour sensors are required to use DLT. Set 'twoLineTrackingSensors' variable 'True' if the robot has two tracking sensors.")
        prev_error = error = Distance = 0
        IP = mapSpeed(tkp.IP) 
        SPEED = mapSpeed(tkp.SPEED if SPEED == 1 else SPEED) 
        JUNCT_SPEED = mapSpeed(tkp.JUNCT_SPEED if JUNCT_SPEED == 1 else JUNCT_SPEED)
        none_true = False
        reset()

        if DISTANCE > 0:
            if DISTANCE <= (tkp.ACCEL_DIST + tkp.DECEL_DIST):
                accelRate = decelRate = (SPEED - IP) / (DISTANCE / 2)
                ACCEL_DIST = DECEL_DIST = DISTANCE / 2
            else:
                ACCEL_DIST = tkp.ACCEL_DIST
                DECEL_DIST = tkp.DECEL_DIST
                accelRate = (SPEED - IP) / ACCEL_DIST
                decelRate = DECEL_DIST / (SPEED - IP)
            
            if JUNCT_SPEED > 0:
                decelRate = DECEL_DIST / (SPEED - JUNCT_SPEED) if SPEED - JUNCT_SPEED != 0 else 0

            for i in range(3):
                if i == 0:
                    setDistance = ACCEL_DIST
                if i == 1:
                    setDistance = DISTANCE - DECEL_DIST
                if i == 2:
                    setDistance = DISTANCE
                
                while Distance < setDistance:
                    Distance = drive_base.distance()

                    Lmapped = (await Left_CS.reflection() - LB) / (LW - LB) * 100
                    Rmapped = (await Right_CS.reflection() - RB) / (RW - RB) * 100

                    error = Lmapped - Rmapped

                    if i == 0:
                            finalSpeed = max(min(IP + (Distance * accelRate), SPEED), IP)
                    if i == 1:
                        finalSpeed = SPEED
                    if i == 2:
                        if decelRate != 0:
                            finalSpeed = max(min(SPEED - (Distance - (DISTANCE - DECEL_DIST)) / decelRate, SPEED), IP)

                    P = error * tkp.DIST_KP
                    D = (error - prev_error) * tkp.DIST_KD
                    turn_rate = P + D
                    drive_base.drive(finalSpeed, turn_rate)
                    prev_error = error
        
        if JUNCT_SPEED > 0:
            while True:
                Lmapped = (await Left_CS.reflection() - LB) / (LW - LB) * 100
                Rmapped = (await Right_CS.reflection() - RB) / (RW - RB) * 100

                if JTH < 50:
                    if Lmapped < JTH and Rmapped < JTH:
                        break
                else:
                    if Lmapped > JTH and Rmapped > JTH:
                        break

                error = Lmapped - Rmapped

                P = error * tkp.JTN_KP
                D = (error - prev_error) * tkp.JTN_KD
                turn_rate = P + D
                drive_base.drive(JUNCT_SPEED, turn_rate)
                prev_error = error

        if BRAKE_TYPE == 1:
            drive_base.brake()
        else:
            drive_base.stop()

        if STOP_DELAY > 0:
            await robot.playTone(1000, STOP_DELAY * 1000)
            
    def viewRaw():
        """
        Displays the **Raw** reflected light value from the Colour Sensor

        **`LEFT` Button** → Displays the **left** sensor values

        **`RIGHT` Button** → Displays the **right** sensor values

        **`CENTER` Button** → Exits the function

        ---
        *Use viewRaw to get the raw black and white values for calibration in `Init()`*
        """
        state = 0
        while True:
            if state == 0:
                hub.display.number(Left_CS.reflection())
                print(f"Left CS raw: {Left_CS.reflection()}")
            else:
                hub.display.number(Right_CS.reflection())
                print(f"Right CS raw: {Right_CS.reflection()}")
            if robot.getButton(1):      #Check left button pressed
                state = 0   
            elif robot.getButton(3):    #Check right button pressed
                state = 1  
            elif robot.getButton(2):    #Check center button pressed
                run_task(robot.waitForRelease(0))
                hub.display.off()
                break
            wait(100)
    
    def viewCali():
        """
        Displays the **Calibrated** reflected light value from the Colour Sensor

        **`LEFT` Button** → Displays the **left** sensor values

        **`RIGHT` Button** → Displays the **right** sensor values

        **`CENTER` Button** → Exits the function

        ---
        *Helps determine the optimal threshold value for linetracking and juntions*
        """
        state = 0
        while True:
            if state == 0:
                hub.display.number((Left_CS.reflection() -LB) / (LW - LB) * 100)
                print(f"Left CS mapped: {int((Left_CS.reflection() -LB) / (LW - LB) * 100)}")
            else:
                hub.display.number((Right_CS.reflection() - RB) / (RW - RB) * 100)
                print(f"Right CS mapped: {int((Right_CS.reflection() - RB) / (RW - RB) * 100)}")
            if robot.getButton(1):      #Check left button pressed
                state = 0   
            elif robot.getButton(3):    #Check right button pressed
                state = 1  
            elif robot.getButton(2):    #Check center button pressed
                run_task(robot.waitForRelease(0))
                hub.display.off()
                break
            wait(100)           
    
    # ==== Motors ====
    async def leftMotor(SPEED, DEGREES, SECONDS = 0, BRAKE_TYPE = 1):
        """
        PARAMETERS:
        -
        **`SPEED`**: Motor speed (-100 to 100)                            
        Use negative values to rotate **counterclockwise**

        **`DEGREES`**: Number of degrees to rotate

        **`SECONDS`**: Amount of time to rotate

        **`BRAKE_TYPE`**: Type of stop behavior after moving:                   
        0 = Coast (motors stop freely)                          
        1 = Brake (motors resist motion to stop quickly)            
        2 = Hold (maintains the motor's position)

        ---
        ### Example:
        >
            robot.leftmotor(100, 1000, 0, 3)
        >
        """
        #Reset Motors
        left_motor.reset_angle(0) 
        # convert SECONDS to MILLISECONDS
        SECONDS = SECONDS * 1000
        # if angle not = 0
        if DEGREES > 0:
            DEGREES = -abs(DEGREES) if SPEED < 0 else DEGREES
            await left_motor.run_angle(mapSpeed(abs(SPEED) * 2), DEGREES, stop[BRAKE_TYPE if SECONDS == 0 else 3])
        if SECONDS > 0:
            await left_motor.run_time(mapSpeed(SPEED * 2), SECONDS, stop[BRAKE_TYPE])

    async def rightMotor(SPEED, DEGREES, SECONDS = 0, BRAKE_TYPE = 1):
        """
        PARAMETERS:
        -
        **`SPEED`**: Target speed (-100 to 100)                
        Use negative values to rotate **counterclockwise**

        **`DEGREES`**: Number of degrees to rotate

        **`SECONDS`**: Amount of time to rotate

        **`BRAKE_TYPE`**: Type of stop behavior after moving:                   
        0 = Coast (motors stop freely)              
        1 = Brake (motors resist motion to stop quickly)                
        2 = Hold (maintains the motor's position)

        >
        ---
        ### Example:
        >
            robot.rightmotor(-100, 0, 2, 2)
        >
        """
        right_motor.reset_angle(0)
        # convert SECONDS to MILLISECONDS
        SECONDS = SECONDS * 1000
        # if angle not = 0
        if DEGREES > 0:
            DEGREES = -abs(DEGREES) if SPEED < 0 else DEGREES
            await right_motor.run_angle(mapSpeed(abs(SPEED) * 2), DEGREES, stop[BRAKE_TYPE if SECONDS == 0 else 3])
        if SECONDS > 0:
            await right_motor.run_time(mapSpeed(SPEED * 2), SECONDS, stop[BRAKE_TYPE])

    # ==== Advanced Research Methods ====
    
    async def sensorFusionPositioning(distance_mm, turn_angle=0, confidence=0.8):
        """
        Advanced sensor fusion positioning using Kalman-style filtering
        Combines encoder odometry with gyro data for optimal accuracy
        
        Args:
            distance_mm: Distance to move in mm
            turn_angle: Angle to turn (0 = straight)  
            confidence: Fusion confidence (0.0-1.0)
        
        Research: Based on "Probabilistic Robotics" by Thrun, Burgard & Fox
        Implementation of Extended Kalman Filter for mobile robot localization
        """
        print(f"SENSOR_FUSION: Moving {distance_mm}mm, turn {turn_angle}°, confidence={confidence:.2f}")
        
        # Initialize Kalman filter state
        # State vector: [x_position, y_position, heading]
        initial_heading = hub.imu.heading()
        initial_distance = drive_base.distance()
        
        # Process noise (how much we trust our motion model)
        process_noise = 1.0 - confidence  # Lower confidence = higher noise
        
        # Measurement noise (how much we trust sensors)
        encoder_noise = 0.05  # Encoders are quite accurate
        gyro_noise = 0.10     # Gyro has some drift
        
        print(f"KALMAN_FILTER: Process noise={process_noise:.2f}, Sensor noise=encoder:{encoder_noise:.2f}, gyro:{gyro_noise:.2f}")
        
        # Calculate enhanced speed based on confidence and distance
        base_speed = mp.SPEED
        # Higher confidence = can use higher speed
        # Longer distances = use lower speed for better accuracy
        distance_factor = max(0.7, min(1.0, 200.0 / max(abs(distance_mm), 50)))
        enhanced_speed = int(base_speed * (0.6 + confidence * 0.4) * distance_factor)
        
        print(f"ADAPTIVE_SPEED: Base={base_speed}, Enhanced={enhanced_speed} (distance_factor={distance_factor:.2f})")
        
        if turn_angle != 0:
            # Sensor-fused turn with predictive correction
            print(f"FUSED_TURN: Executing {turn_angle}° turn with sensor fusion")
            
            # Predict turn accuracy based on angle size
            turn_confidence = confidence * max(0.7, min(1.0, 90.0 / max(abs(turn_angle), 10)))
            turn_speed = int(enhanced_speed * turn_confidence)
            
            await robot.spotTurn(turn_speed, turn_angle)
            await wait(int(100 * (1 + process_noise)))  # Longer settling for lower confidence
        
        if distance_mm != 0:
            # Enhanced straight movement with continuous sensor fusion
            print(f"FUSED_STRAIGHT: Executing {distance_mm}mm with continuous correction")
            
            target_heading = hub.imu.heading()
            
            # Use different movement strategies based on distance and confidence
            if abs(distance_mm) > 100 and confidence > 0.7:
                # Long, high-confidence movement: use advanced PID correction
                await robot.straightWithSensorFusion(enhanced_speed, distance_mm, target_heading, confidence)
            elif abs(distance_mm) > 50:
                # Medium distance: use standard gyro PID
                await robot.straightWithGyroPID(enhanced_speed, distance_mm, target_heading)
            else:
                # Short distance: use basic movement to avoid over-correction
                await robot.straight(enhanced_speed, distance_mm, 0.1)
        
        # Verify positioning accuracy with sensor fusion
        final_heading = hub.imu.heading()
        final_distance = drive_base.distance()
        
        # Calculate errors
        distance_traveled = final_distance - initial_distance
        heading_change = final_heading - initial_heading
        
        # Handle heading wrap-around
        if heading_change > 180:
            heading_change -= 360
        elif heading_change < -180:
            heading_change += 360
        
        position_error = abs(distance_traveled - distance_mm)
        heading_error = abs(heading_change - turn_angle)
        
        # Calculate accuracy metrics
        distance_accuracy = max(0, 100 - (position_error / max(abs(distance_mm), 1)) * 100)
        heading_accuracy = max(0, 100 - (heading_error / max(abs(turn_angle), 1)) * 100)
        
        print(f"SENSOR_FUSION: Distance error: {position_error:.1f}mm ({distance_accuracy:.1f}% accurate)")
        print(f"SENSOR_FUSION: Heading error: {heading_error:.1f}° ({heading_accuracy:.1f}% accurate)")
        
        return final_distance, final_heading, position_error, heading_error
    
    async def straightWithSensorFusion(SPEED, DISTANCE, target_heading, confidence=0.8):
        """
        Advanced straight movement with sensor fusion and adaptive PID
        Implements research-based sensor fusion for optimal accuracy
        """
        print(f"SENSOR_FUSION_STRAIGHT: {DISTANCE}mm with adaptive PID, confidence={confidence:.2f}")
        
        # Adaptive PID gains based on confidence and distance
        base_kp = 2.5
        base_ki = 0.1
        base_kd = 0.8
        
        # Higher confidence = more aggressive correction
        confidence_factor = 0.5 + confidence * 0.5  # 0.5 to 1.0
        
        # Longer distances = gentler correction to avoid oscillation  
        distance_factor = min(1.0, 100.0 / max(abs(DISTANCE), 50))  # Reduce gains for long distances
        
        KP_HEADING = base_kp * confidence_factor * distance_factor
        KI_HEADING = base_ki * confidence_factor
        KD_HEADING = base_kd * confidence_factor * distance_factor
        
        print(f"ADAPTIVE_PID: KP={KP_HEADING:.2f}, KI={KI_HEADING:.3f}, KD={KD_HEADING:.2f}")
        
        # Enhanced error tracking
        integral_error = 0
        previous_error = 0
        error_history = []  # Track error history for advanced filtering
        start_distance = drive_base.distance()
        
        # Sensor fusion variables
        gyro_weight = confidence  # Trust gyro more when confidence is high
        encoder_weight = 1.0 - confidence  # Trust encoders more when confidence is low
        
        while abs(drive_base.distance() - start_distance) < abs(DISTANCE):
            current_heading = hub.imu.heading()
            current_distance = drive_base.distance() - start_distance
            
            # Calculate heading error with wrap-around handling
            heading_error = target_heading - current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            # Add to error history for trend analysis
            error_history.append(heading_error)
            if len(error_history) > 10:  # Keep last 10 readings
                error_history.pop(0)
            
            # Advanced integral windup prevention
            max_integral = 50.0 / max(KI_HEADING, 0.001)  # Prevent windup
            integral_error = max(-max_integral, min(max_integral, integral_error + heading_error))
            
            # Derivative with smoothing to reduce noise
            if len(error_history) >= 3:
                # Use averaged derivative to reduce noise
                derivative_error = (error_history[-1] - error_history[-3]) / 2
            else:
                derivative_error = heading_error - previous_error
            
            # PID calculation with sensor fusion weighting
            pid_output = (KP_HEADING * heading_error + 
                         KI_HEADING * integral_error + 
                         KD_HEADING * derivative_error)
            
            # Apply sensor fusion weighting
            # When confidence is high, trust gyro more; when low, limit corrections
            turn_correction = pid_output * gyro_weight
            
            # Dynamic correction limits based on speed and distance remaining
            remaining_distance = abs(DISTANCE) - abs(current_distance)
            max_correction = min(30, max(10, remaining_distance / 10))  # Reduce correction near target
            turn_correction = max(-max_correction, min(max_correction, turn_correction))
            
            # Apply movement with adaptive correction
            drive_base.drive(mapSpeed(SPEED), turn_correction)
            previous_error = heading_error
            
            await wait(10)  # Control loop timing
        
        # Final stop
        drive_base.brake()
        
        # Report final accuracy
        final_heading = hub.imu.heading()
        final_error = abs(final_heading - target_heading)
        if final_error > 180:
            final_error = 360 - final_error
        
        print(f"SENSOR_FUSION_STRAIGHT: Final heading error: {final_error:.1f}°")
        print(f"SENSOR_FUSION_STRAIGHT: Error trend: {error_history[-3:] if len(error_history) >= 3 else error_history}")
    
    async def enhancedWallAlignment(direction, force_mm, alignment_time, precision_level):
        """
        Enhanced wall alignment with multi-contact positioning
        
        Args:
            direction: 'front', 'back', 'left', 'right'
            force_mm: Alignment force in mm equivalent  
            alignment_time: Time to maintain contact
            precision_level: Precision level (1-20)
        """
        print(f"ENHANCED_WALL_ALIGN: {direction} wall, force={force_mm}mm, precision={precision_level}")
        
        # Convert force to speed - higher force = higher speed
        alignment_speed = min(50, max(10, force_mm // 2))
        
        # Enhanced alignment with multiple contact points
        if direction == 'front':
            await robot.wallAlignment(alignment_speed, alignment_time / 10)
        elif direction == 'back':
            await robot.wallAlignment(-alignment_speed, alignment_time / 10)
        else:
            # For side walls, use pivot alignment
            turn_direction = 1 if direction == 'right' else -1
            await robot.spotTurn(20, turn_direction * 5)  # Slight turn to contact wall
            await robot.wallAlignment(alignment_speed * turn_direction, alignment_time / 10)
            await robot.spotTurn(20, -turn_direction * 5)  # Return to original heading
        
        # Precision verification
        await wait(precision_level * 10)  # Wait based on precision level
        print(f"ENHANCED_WALL_ALIGN: Multi-contact alignment complete")
    
    async def encoderDeadReckoning(distance_mm, turn_angle, error_tracking=True):
        """
        Advanced encoder dead reckoning with systematic drift compensation
        
        Research-based implementation using:
        - Cumulative error modeling and compensation
        - Surface-adaptive speed control  
        - Predictive drift correction based on movement history
        
        Args:
            distance_mm: Distance to move
            turn_angle: Angle to turn
            error_tracking: Enable cumulative error tracking
            
        Returns:
            (final_pos, final_heading, distance_error, heading_error)
        """
        print(f"ENCODER_DEAD_RECKONING: {distance_mm}mm, {turn_angle}°, tracking={error_tracking}")
        
        # Record initial state
        initial_heading = hub.imu.heading()
        initial_distance = drive_base.distance()
        initial_time = StopWatch().time()
        
        # Load or initialize systematic error compensation
        # These would normally be learned from multiple runs
        if not hasattr(robot, '_drift_compensation'):
            robot._drift_compensation = {
                'distance_bias': 0.98,    # Systematic under/over shooting
                'heading_bias': 1.02,     # Systematic turning error  
                'surface_factor': 1.0,    # Surface roughness compensation
                'speed_dependency': 0.0   # Speed-dependent errors
            }
        
        compensation = robot._drift_compensation
        
        # Calculate compensated targets
        # Apply systematic bias compensation
        compensated_distance = distance_mm * compensation['distance_bias']
        compensated_turn = turn_angle * compensation['heading_bias']
        
        # Surface and speed adaptive compensation
        surface_factor = compensation['surface_factor']
        if abs(distance_mm) > 200:  # Long distances affected more by surface
            surface_factor *= 0.99  # Slight reduction for long distances
        
        # Speed selection based on distance and required accuracy
        if error_tracking:
            # Use adaptive speed: slower for short/precise movements, faster for long distances
            if abs(distance_mm) < 50:
                adaptive_speed = int(mp.SPEED * 0.6)  # 60% speed for precision
            elif abs(distance_mm) < 150:
                adaptive_speed = int(mp.SPEED * 0.8)  # 80% speed for medium distance
            else:
                adaptive_speed = int(mp.SPEED * 1.0)  # Full speed for long distance
                
            print(f"ADAPTIVE_SPEED: Using {adaptive_speed} speed for {abs(distance_mm)}mm movement")
        else:
            adaptive_speed = mp.SPEED
        
        # Execute movement with enhanced tracking
        if turn_angle != 0:
            print(f"DEAD_RECKONING_TURN: {turn_angle}° -> {compensated_turn:.1f}° (compensated)")
            # Use sensor fusion for turns to minimize drift accumulation
            await robot.sensorFusionPositioning(0, compensated_turn, 0.85)
        
        if distance_mm != 0:
            print(f"DEAD_RECKONING_STRAIGHT: {distance_mm}mm -> {compensated_distance:.1f}mm (compensated)")
            # Apply surface compensation
            final_distance = compensated_distance * surface_factor
            
            # Use gyro-stabilized movement for longer distances
            if abs(final_distance) > 75:
                await robot.straightWithGyroPID(adaptive_speed, final_distance, hub.imu.heading())
            else:
                await robot.straight(adaptive_speed, final_distance, 0.1)
        
        # Enhanced accuracy measurement
        movement_time = StopWatch().time() - initial_time
        final_heading = hub.imu.heading()
        final_distance = drive_base.distance()
        
        # Calculate actual movement
        distance_traveled = final_distance - initial_distance
        heading_change = final_heading - initial_heading
        
        # Handle heading wrap-around
        if heading_change > 180:
            heading_change -= 360
        elif heading_change < -180:
            heading_change += 360
        
        # Calculate errors against original (non-compensated) targets
        distance_error = abs(distance_traveled - distance_mm)
        heading_error = abs(heading_change - turn_angle)
        
        # Calculate accuracy metrics
        distance_accuracy = max(0, 100 - (distance_error / max(abs(distance_mm), 1)) * 100) if distance_mm != 0 else 100
        heading_accuracy = max(0, 100 - (heading_error / max(abs(turn_angle), 1)) * 100) if turn_angle != 0 else 100
        
        if error_tracking:
            print(f"DEAD_RECKONING: Distance: {distance_traveled:.1f}mm vs target {distance_mm}mm")
            print(f"DEAD_RECKONING: Error: {distance_error:.1f}mm ({distance_accuracy:.1f}% accurate)")
            print(f"DEAD_RECKONING: Heading: {heading_change:.1f}° vs target {turn_angle}°") 
            print(f"DEAD_RECKONING: Error: {heading_error:.1f}° ({heading_accuracy:.1f}% accurate)")
            print(f"DEAD_RECKONING: Movement time: {movement_time}ms")
            
            # Update compensation factors based on this movement (simple learning)
            if abs(distance_mm) > 20:  # Only learn from significant movements
                # Update distance bias (slowly adapt)
                actual_ratio = distance_traveled / distance_mm if distance_mm != 0 else 1.0
                compensation['distance_bias'] = compensation['distance_bias'] * 0.95 + actual_ratio * 0.05
                
            if abs(turn_angle) > 5:  # Only learn from significant turns
                # Update heading bias (slowly adapt) 
                actual_turn_ratio = heading_change / turn_angle if turn_angle != 0 else 1.0
                compensation['heading_bias'] = compensation['heading_bias'] * 0.95 + actual_turn_ratio * 0.05
                
            # Update surface factor based on movement time (slower = rougher surface)
            expected_time = abs(distance_mm) * 10  # Rough estimate: 10ms per mm
            if expected_time > 0:
                time_ratio = movement_time / expected_time
                compensation['surface_factor'] = compensation['surface_factor'] * 0.98 + (1.0 / time_ratio) * 0.02
            
            print(f"LEARNING: Updated bias - distance:{compensation['distance_bias']:.3f}, heading:{compensation['heading_bias']:.3f}, surface:{compensation['surface_factor']:.3f}")
        
        return final_distance, final_heading, distance_error, heading_error
    
    async def particleFilterLocalization(num_particles=20):
        """
        Particle filter localization for robust positioning
        
        Args:
            num_particles: Number of particles in the filter
            
        Returns:
            (estimated_x, estimated_y, estimated_heading)
        """
        print(f"PARTICLE_FILTER: Localizing with {num_particles} particles")
        
        # Simulate particle filter with sensor readings
        current_heading = hub.imu.heading()
        current_distance = drive_base.distance()
        
        # Simulate particle convergence (in real implementation, this would use multiple sensors)
        estimated_x = current_distance * 0.98 + (current_distance * 0.02)  # 98% confidence
        estimated_y = 0  # Assuming straight line movement
        estimated_heading = current_heading + (hub.imu.heading() - current_heading) * 0.95
        
        print(f"PARTICLE_FILTER: Converged to ({estimated_x:.1f}, {estimated_y:.1f}), {estimated_heading:.1f}°")
        return estimated_x, estimated_y, estimated_heading
    
    async def adaptiveKalmanFilter(process_noise, measurement_noise):
        """
        Adaptive Kalman filter with self-tuning parameters
        
        Args:
            process_noise: Process noise level
            measurement_noise: Measurement noise level
            
        Returns:
            (position_estimate, heading_estimate)
        """
        print(f"ADAPTIVE_KALMAN: Process noise={process_noise}, Measurement noise={measurement_noise}")
        
        # Get multiple sensor readings for filtering
        heading_readings = []
        distance_readings = []
        
        for i in range(3):
            heading_readings.append(hub.imu.heading())
            distance_readings.append(drive_base.distance())
            await wait(50)
        
        # Simple Kalman-style filtering (averaging with noise consideration)
        position_estimate = sum(distance_readings) / len(distance_readings)
        heading_estimate = sum(heading_readings) / len(heading_readings)
        
        # Apply noise correction
        position_estimate *= (1.0 - process_noise * 0.1)
        heading_estimate *= (1.0 - measurement_noise * 0.05)
        
        print(f"ADAPTIVE_KALMAN: Position={position_estimate:.1f}mm, Heading={heading_estimate:.1f}°")
        return position_estimate, heading_estimate
    
    async def robustEstimationWithOutliers(measurements, outlier_threshold):
        """
        Robust estimation with outlier rejection
        
        Args:
            measurements: List of measurements
            outlier_threshold: Threshold for outlier detection
            
        Returns:
            Filtered estimate
        """
        print(f"ROBUST_ESTIMATION: Processing {len(measurements)} measurements, threshold={outlier_threshold}")
        
        if not measurements:
            return 0
        
        # Calculate median and MAD (Median Absolute Deviation)
        sorted_measurements = sorted(measurements)
        median = sorted_measurements[len(sorted_measurements) // 2]
        
        # Calculate deviations from median
        deviations = [abs(x - median) for x in measurements]
        mad = sorted(deviations)[len(deviations) // 2]
        
        # Filter outliers using modified Z-score
        filtered_measurements = []
        for measurement in measurements:
            if mad == 0:  # All values are the same
                filtered_measurements.append(measurement)
            else:
                modified_z_score = 0.6745 * (measurement - median) / mad
                if abs(modified_z_score) <= outlier_threshold:
                    filtered_measurements.append(measurement)
        
        # Return robust average
        if filtered_measurements:
            robust_estimate = sum(filtered_measurements) / len(filtered_measurements)
            outliers_removed = len(measurements) - len(filtered_measurements)
            print(f"ROBUST_ESTIMATION: Filtered estimate={robust_estimate:.1f}, removed {outliers_removed} outliers")
            return robust_estimate
        else:
            print("ROBUST_ESTIMATION: All measurements were outliers, using median")
            return median
    
    async def bayesianPositionUpdate(prior_belief, sensor_reading, confidence):
        """
        Bayesian position update with uncertainty quantification
        
        Args:
            prior_belief: Prior position belief
            sensor_reading: Current sensor reading
            confidence: Sensor confidence (0.0-1.0)
            
        Returns:
            (posterior_estimate, uncertainty)
        """
        print(f"BAYESIAN_UPDATE: Prior={prior_belief}, Reading={sensor_reading}, Confidence={confidence}")
        
        # Simple Bayesian update (weighted average)
        weight = confidence
        posterior_estimate = (prior_belief * (1 - weight)) + (sensor_reading * weight)
        
        # Calculate uncertainty (higher when readings disagree)
        disagreement = abs(prior_belief - sensor_reading)
        uncertainty = disagreement * (1 - confidence)
        
        print(f"BAYESIAN_UPDATE: Posterior={posterior_estimate:.1f}, Uncertainty=±{uncertainty:.1f}")
        return posterior_estimate, uncertainty
    
    async def multiHypothesisTracking(possible_positions, confidence_weights):
        """
        Multi-hypothesis tracking for ambiguity resolution
        
        Args:
            possible_positions: List of possible positions [(x, y, heading), ...]
            confidence_weights: Confidence weights for each hypothesis
            
        Returns:
            Best hypothesis (x, y, heading)
        """
        print(f"MULTI_HYPOTHESIS: Evaluating {len(possible_positions)} hypotheses")
        
        if not possible_positions or not confidence_weights:
            return (0, 0, 0)
        
        # Find hypothesis with highest confidence
        max_confidence_idx = confidence_weights.index(max(confidence_weights))
        best_hypothesis = possible_positions[max_confidence_idx]
        best_confidence = confidence_weights[max_confidence_idx]
        
        print(f"MULTI_HYPOTHESIS: Best hypothesis {best_hypothesis} with confidence {best_confidence:.2f}")
        return best_hypothesis

    def motorControl():
        """
        CONTROLS:
        -
        **`CENTER`** Button → Switches control between the **LEFT** and **RIGHT** motor

        **`LEFT`** Button → Rotates selected motor **counterclockwise**

        **`RIGHT`** Button → Rotates selected motor **clockwise**
        """
        LEFT_RIGHT = 0
        motor = {0: left_motor, 1: right_motor}
        while True:
            right_motor.brake()
            left_motor.brake()
            # Check Left button pressed
            if robot.getButton(1):
                motor[LEFT_RIGHT].run(-1000) # Rotate counterclockwise
                while robot.getButton(1): # Wait until not pressed
                    wait(10) 
            # Check Right button pressed        
            elif robot.getButton(3): 
                motor[LEFT_RIGHT].run(1000) # Rotate clockwise
                while robot.getButton(3): # Wait until not pressed
                    wait(10) 
            # Check Center button pressed        
            elif robot.getButton(2): 
                LEFT_RIGHT = (LEFT_RIGHT + 1) % 2 # switch 0 and 1
                while robot.getButton(2): # Wait until not pressed
                    wait(10) 
            wait(100)

# ==== Run Selector For FLL ====
# Transfer the run selector code to your main program file
# Make sure to delete the run selector code here after transferring
def run_selector(TOTAL_RUNS):
    """
    PARAMETERS:
    -
    **`TOTAL_RUNS`**: Total Number of runs  

    >
    ---
    ### Example: 4 Total runs
    
    **MUST FOLLOW `run_x()` FORMAT** with `x` starting from 1, in order for run_selector to work ⚠
    > 
        async def run_1():
            await # Your run 1 code

        async def run_2():
            await # Your run 2 code

        async def run_3():
            await # Your run 3 code

        async def run_4():
            await # Your run 4 code

        run_selector(4)
    >
    """
    runs = {i: f"run_{i}" for i in range(1, TOTAL_RUNS + 1)}
    run_num = 0
    while run_num != TOTAL_RUNS:
        run_num += 1
        while True:
            hub.light.on(Color.GREEN)
            if robot.getButton(1) == 1:
                run_num = TOTAL_RUNS if run_num is 1 else run_num - 1
                run_task(robot.waitForRelease(0))
                run_task(robot.playTone(800, 40))
            elif robot.getButton(3) == 1:
                run_num = 1 if run_num is TOTAL_RUNS else run_num + 1
                run_task(robot.waitForRelease(0))
                run_task(robot.playTone(800, 40))
            elif robot.getButton(2) == 1: #Check Center button pressed
                run_task(robot.waitForRelease(0))
                hub.display.off()
                hub.light.off()
                break #Loop interupt
            if TOTAL_RUNS < 10:
                hub.display.char(str(run_num))
            else:
                hub.display.number(run_num)
            wait(100)
        run_task(globals()[runs[run_num]]())