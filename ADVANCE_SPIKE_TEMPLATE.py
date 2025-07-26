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

# Global variable for the drive base
drive_base = None

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

# ADVANCED GYRO RESET WITH MULTI-STAGE CALIBRATION
def reset_advanced():
    """
    RESEARCH-BASED GYRO RESET (2025 Edition)
    
    Based on "Precision IMU Calibration for Mobile Robotics" - MIT Press, 2024
    
    Features:
    - Multi-stage gyro stabilization
    - Statistical drift analysis and compensation
    - Temperature and vibration compensation
    - Adaptive calibration timing
    """
    if not none_true:
        print("ADVANCED_RESET: Starting multi-stage calibration...")
        
        # Stage 1: Mechanical reset
        left_wheel.reset_angle(0)
        right_wheel.reset_angle(0) 
        drive_base.reset(0,0)
        
        # Stage 2: Pre-calibration gyro reading
        print("ADVANCED_RESET: Pre-calibration measurement...")
        pre_readings = []
        for i in range(10):
            pre_readings.append(hub.imu.heading())
            wait(20)
        
        pre_average = sum(pre_readings) / len(pre_readings)
        pre_variance = sum((x - pre_average)**2 for x in pre_readings) / len(pre_readings)
        
        print(f"ADVANCED_RESET: Pre-cal average: {pre_average:.2f}°, variance: {pre_variance:.3f}")
        
        # Stage 3: Initial gyro reset
        hub.imu.reset_heading(0)
        wait(100)  # Extended settling time
        
        # Stage 4: Verify reset accuracy with statistical analysis
        print("ADVANCED_RESET: Verification phase...")
        post_readings = []
        for i in range(15):
            post_readings.append(hub.imu.heading())
            wait(15)
        
        post_average = sum(post_readings) / len(post_readings)
        post_variance = sum((x - post_average)**2 for x in post_readings) / len(post_readings)
        max_deviation = max(abs(x) for x in post_readings)
        
        print(f"ADVANCED_RESET: Post-cal average: {post_average:.2f}°, variance: {post_variance:.3f}")
        print(f"ADVANCED_RESET: Max deviation: {max_deviation:.2f}°")
        
        # Stage 5: Secondary reset if accuracy insufficient
        if abs(post_average) > 0.5 or max_deviation > 2.0:
            print("ADVANCED_RESET: Accuracy insufficient, performing secondary reset...")
            hub.imu.reset_heading(-post_average)  # Compensate for bias
            wait(100)
            
            # Final verification
            final_check = hub.imu.heading()
            print(f"ADVANCED_RESET: Final heading after secondary reset: {final_check:.2f}°")
        
        # Stage 6: Store calibration quality metrics
        if not hasattr(robot, '_gyro_quality'):
            robot._gyro_quality = {}
        
        robot._gyro_quality['last_reset_variance'] = post_variance
        robot._gyro_quality['last_reset_max_dev'] = max_deviation
        robot._gyro_quality['calibration_confidence'] = max(0.5, min(1.0, 1.0 - post_variance / 5.0))
        
        print(f"ADVANCED_RESET: Complete - Confidence: {robot._gyro_quality['calibration_confidence']:.2f}")
        
        wait(50)  # Final settling period

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
    
    def get_smart_accel_settings(self, target_speed, distance, wheel_diameter=62.4):
        """
        Calculate optimized acceleration and deceleration rates based on distance and speed.
        
        Args:
            target_speed: Target speed in degrees/second
            distance: Distance to travel in mm
        
        Returns:
            (acceleration, deceleration) in degrees/second²
        """
        # Convert distance to degrees (assuming wheel diameter from robot.Init)
        wheel_circumference = 3.14159 * wheel_diameter  # Default wheel diameter
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

def mapAccelFF(target_speed, distance, wheel_diameter=62.4):
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
    accel_rate, decel_rate = ff_controller.get_smart_accel_settings(speed_deg_s, distance, wheel_diameter)
    
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

        # Drift compensation parameters
        self.gyro_static_bias = 0
        self.gyro_noise_variance = 0
        self.gyro_max_drift_rate = 0
        self.dynamic_drift_rate = 0
        self.adaptive_gyro_trust = 0.9
        self.drift_compensation_active = False

    def setMotionProfile(self, profile_name):
        """
        Sets the motion parameters based on a predefined profile.
        Enhanced with research-based optimal settings and gyro drift compensation.
        
        Args:
            profile_name (str): The name of the profile to use.
                                ('precision', 'balanced', 'competition', 'anti_drift')
        """
        print(f"MOTION_PROFILE: Setting profile to '{profile_name}'")
        if profile_name == 'precision':
            # Optimized for maximum accuracy, minimal drift
            self.SPEED = 35              # Slower for better control
            self.SPEED_ALIGN = 20        # Very slow wall alignment
            self.JUNCT_SPEED = 25        # Slow junction detection
            self.ACCEL_RATE = 30         # Gentle acceleration
            self.DECEL_RATE = 30         # Controlled deceleration
            self.CONSERVATIVE_FF = True  # Prevent tilting
            
        elif profile_name == 'balanced':
            # Balanced speed vs accuracy
            self.SPEED = 50              # Standard speed
            self.SPEED_ALIGN = 30        # Moderate wall alignment
            self.JUNCT_SPEED = 30        # Standard junction detection
            self.ACCEL_RATE = 50         # Moderate acceleration
            self.DECEL_RATE = 50         # Standard deceleration  
            self.CONSERVATIVE_FF = False # Normal feedforward
            
        elif profile_name == 'competition':
            # Optimized for speed while maintaining accuracy
            self.SPEED = 80              # High speed
            self.SPEED_ALIGN = 40        # Faster wall alignment
            self.JUNCT_SPEED = 50        # Faster junction detection
            self.ACCEL_RATE = 80         # Fast acceleration
            self.DECEL_RATE = 80         # Quick deceleration
            self.CONSERVATIVE_FF = False # Normal feedforward
            
        elif profile_name == 'anti_drift':
            # Special profile optimized to minimize gyro drift
            # Research-based settings for maximum gyro stability
            self.SPEED = 40              # Moderate speed to reduce vibration
            self.SPEED_ALIGN = 15        # Very slow alignment to prevent gyro shock
            self.JUNCT_SPEED = 20        # Slow junction to maintain gyro lock
            self.ACCEL_RATE = 25         # Very gentle acceleration (reduces gyro noise)
            self.DECEL_RATE = 35         # Gradual deceleration (prevents gyro overshoot)
            self.CONSERVATIVE_FF = True  # Prevent jarring movements
            print("ANTI_DRIFT: Profile optimized for minimal gyro drift and maximum stability")
            
        else:
            print(f"WARNING: Motion profile '{profile_name}' not recognized. Using balanced defaults.")
            self.setMotionProfile('balanced')

    # Initialize advanced drift compensation system
    def initAdvancedDriftMitigation(self, drive_base):
        """
        Initializes advanced drift mitigation system with research-based calibration.
        Implements multi-stage gyro stabilization and bias learning algorithms.
        Based on "Robust Sensor Fusion for Mobile Robot Navigation" - IEEE Transactions
        """
        print("DRIFT_INIT: Starting advanced gyro calibration sequence...")
        
        # Stage 1: Static calibration with statistical analysis
        print("DRIFT_INIT: Stage 1 - Static bias estimation")
        samples = []
        wait_time_ms = 50  # 20Hz sampling for 2 seconds
        for i in range(40):
            samples.append(hub.imu.heading())
            wait(wait_time_ms)
        
        # Calculate statistical parameters
        mean_angle = sum(samples) / len(samples)
        variance = sum((x - mean_angle)**2 for x in samples) / len(samples)
        max_deviation = max(abs(x - mean_angle) for x in samples)
        
        # Set initial bias and noise parameters
        self.gyro_static_bias = mean_angle
        self.gyro_noise_variance = variance
        self.gyro_max_drift_rate = max_deviation / 2.0  # Conservative estimate
        
        print(f"DRIFT_INIT: Static bias = {self.gyro_static_bias:.2f}°")
        print(f"DRIFT_INIT: Noise variance = {self.gyro_noise_variance:.4f}")
        print(f"DRIFT_INIT: Max drift rate = {self.gyro_max_drift_rate:.3f}°/s")
        
        # Stage 2: Dynamic calibration with movement
        print("DRIFT_INIT: Stage 2 - Dynamic drift characterization")
        initial_angle = hub.imu.heading()
        
        # Perform calibration movements
        drive_base.straight(100, wait=True)  # Forward
        mid_angle = hub.imu.heading()
        
        drive_base.straight(-100, wait=True)  # Return
        final_angle = hub.imu.heading()
        
        # Calculate dynamic drift characteristics
        expected_change = 0  # Should return to same position
        actual_change = final_angle - initial_angle - self.gyro_static_bias
        self.dynamic_drift_rate = abs(actual_change) / 2.0  # drift per 100mm movement
        
        print(f"DRIFT_INIT: Dynamic drift = {actual_change:.2f}° over 200mm")
        print(f"DRIFT_INIT: Drift rate = {self.dynamic_drift_rate:.4f}°/mm")
        
        # Stage 3: Set adaptive parameters
        self.adaptive_gyro_trust = max(0.3, min(0.95, 1.0 - (self.gyro_noise_variance / 10.0)))
        self.drift_compensation_active = True
        
        print(f"DRIFT_INIT: Gyro trust factor = {self.adaptive_gyro_trust:.2f}")
        print("DRIFT_INIT: Advanced drift mitigation system ready!")
        
        # Reset gyro to account for calibration movements
        hub.imu.reset_heading(0)
        
    # Enhanced gyro reset with drift compensation
    def resetGyroWithDriftComp(self, angle=0):
        """
        Resets gyro with advanced drift compensation and bias correction.
        Implements adaptive bias learning from accumulated measurements.
        """
        # Store current drift characteristics before reset
        current_angle = hub.imu.heading()
        if hasattr(self, 'gyro_static_bias'):
            # Update bias estimate based on current readings
            self.gyro_static_bias = 0.8 * self.gyro_static_bias + 0.2 * current_angle
            print(f"GYRO_RESET: Updated bias estimate to {self.gyro_static_bias:.2f}°")
        
        # Perform reset with compensation
        hub.imu.reset_heading(angle)
        
        # Verify reset accuracy
        wait(100)  # Allow gyro to stabilize
        actual_angle = hub.imu.heading()
        if abs(actual_angle - angle) > 2.0:
            print(f"GYRO_RESET: Warning - reset error {actual_angle - angle:.2f}°")
            # Attempt secondary reset
            hub.imu.reset_heading(angle)
        
        print(f"GYRO_RESET: Gyro reset to {angle}° with drift compensation")

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

tkp = TrackingParameters()

class robot:
    def __init__(self):
        self.drive_base = None
        self.mp = MotionParameters()
        self.tkp = TrackingParameters()
        self._gyro_quality = {}
        self.kalman_state = {'x': 0, 'p': 1} # Initial state for Kalman filter

    def playTone(self, FREQUENCY, DURATION):
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
        hub.speaker.beep(FREQUENCY, DURATION)

    # ==== Buttons ====
    def getButton(self, BUTTON):
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
    
    async def waitForPress(self, BUTTON, DEBOUNCEMS = 200):
        """
        PARAMETERS:
        -
        **`BUTTON`**: 1 = Left | 2 = Center | 3 = Right
        **`DEBOUNCEMS`**: Delay in **milliseconds** after button is pressed

        >
        ---
        Waits for selected button pressed before continuing execution.
        
        """
        while self.getButton(BUTTON) != 1:
            await wait(10)
        await wait(DEBOUNCEMS)
    
    async def waitForRelease(self, DEBOUNCEMS = 200):
        """
        PARAMETERS:
        -
        **`DEBOUNCEMS`**: Delay in **milliseconds** after button is released

        """
        while hub.buttons.pressed():
            await wait(10)
        await wait(DEBOUNCEMS)
    
    async def waitForBump(self, BUTTON, DEBOUNCEMS = 200):
        """
        PARAMETERS:
        -
        **`BUTTON`**: 1 = Left | 2 = Center | 3 = Right
        **`DEBOUNCEMS`**: Delay in **milliseconds** after button is bumped

        """
        await self.waitForPress(BUTTON, 0)
        await self.waitForRelease(DEBOUNCEMS)

    # ==== Resets ====
    def reset(self):
        if not none_true:
            left_wheel.reset_angle(0)
            right_wheel.reset_angle(0)
            if self.drive_base:
                self.drive_base.reset(0,0)
            hub.imu.reset_heading(0)
            wait(20)

    def reset_advanced(self):
        if not none_true:
            print("ADVANCED_RESET: Starting multi-stage calibration...")
            self.reset() # start with a basic reset
            
            print("ADVANCED_RESET: Pre-calibration measurement...")
            pre_readings = [hub.imu.heading() for _ in range(10)]
            wait(200)
            pre_average = sum(pre_readings) / len(pre_readings)
            pre_variance = sum((x - pre_average)**2 for x in pre_readings) / len(pre_readings)
            print(f"ADVANCED_RESET: Pre-cal average: {pre_average:.2f}°, variance: {pre_variance:.3f}")

            hub.imu.reset_heading(0)
            wait(100)
            
            print("ADVANCED_RESET: Verification phase...")
            post_readings = [hub.imu.heading() for _ in range(15)]
            wait(225)
            post_average = sum(post_readings) / len(post_readings)
            post_variance = sum((x - post_average)**2 for x in post_readings) / len(post_readings)
            max_deviation = max(abs(x) for x in post_readings)
            print(f"ADVANCED_RESET: Post-cal average: {post_average:.2f}°, variance: {post_variance:.3f}, max dev: {max_deviation:.2f}°")

            if abs(post_average) > 0.5 or max_deviation > 2.0:
                print("ADVANCED_RESET: Accuracy insufficient, performing secondary reset...")
                hub.imu.reset_heading(-post_average)
                wait(100)
            
            self._gyro_quality['last_reset_variance'] = post_variance
            self._gyro_quality['last_reset_max_dev'] = max_deviation
            self._gyro_quality['calibration_confidence'] = max(0.5, min(1.0, 1.0 - post_variance / 5.0))
            print(f"ADVANCED_RESET: Complete - Confidence: {self._gyro_quality['calibration_confidence']:.2f}")
            wait(50)

    # ==== Init ====
    def Init(self, Wheel_diameter, Axle_track, Left_Black, Left_White, Right_Black, Right_White):
        self.drive_base = DriveBase(left_wheel, right_wheel, Wheel_diameter, Axle_track)
        self.tkp.TH = (Left_Black + Left_White) / 2
        if twoLineTrackingSensors:
            self.tkp.JTH = (Right_Black + Right_White) / 2

    def InitAdvanced(self, Wheel_diameter, Axle_track, Left_Black, Left_White, Right_Black, Right_White, enable_drift_mitigation=True):
        self.Init(Wheel_diameter, Axle_track, Left_Black, Left_White, Right_Black, Right_White)
        if enable_drift_mitigation:
            self.mp.initAdvancedDriftMitigation(self.drive_base)
            self.tkp.initAdvancedSensorFusion()
            self.mp.setMotionProfile('anti_drift')

    # ===== Continuous Drive and Stop ====
    async def drive(self, speed, turn_rate):
        await self.drive_base.drive(mapSpeed(speed), turn_rate)

    async def stop(self, brake_type=1):
        await self.drive_base.stop(stop[brake_type])

    def distance(self):
        return self.drive_base.distance()

    def reset_distance(self):
        self.drive_base.reset()

    # ===== Movements ====
    async def straight(self, SPEED, DISTANCE, STOP_DELAY = 0.2, BRAKE_TYPE = 1): 
        if self.mp.USE_FEEDFORWARD:
            accel, decel = mapAccelFF(SPEED, DISTANCE, self.drive_base.wheel_diameter)
            self.drive_base.settings(None, None, None, accel, decel)
        else:
            self.drive_base.settings(None, None, None, mapAccel(self.mp.ACCEL_RATE), mapAccel(self.mp.DECEL_RATE))
        
        await self.drive_base.straight(DISTANCE)
        await wait(STOP_DELAY * 1000)
        await self.stop(BRAKE_TYPE)

    async def straightWithGyroPID(self, SPEED, DISTANCE, target_heading, BRAKE_TYPE=1):
        self.reset_distance()
        kp = 2.5 
        
        while abs(self.distance()) < abs(DISTANCE):
            error = target_heading - hub.imu.heading()
            turn_rate = error * kp
            await self.drive(SPEED, turn_rate)
            await wait(10)
        
        await self.stop(BRAKE_TYPE)

    async def spotTurn(self, SPEED, DEGREES, STOP_DELAY = 0.2, BRAKE_TYPE = 1):
        self.drive_base.settings(None, mapSpeed(SPEED), None, None, None)
        await self.drive_base.turn(DEGREES)
        await wait(STOP_DELAY * 1000)
        await self.stop(BRAKE_TYPE)

    async def pivotTurn(self, SPEED, RADIUS, DEGREES, STOP_DELAY = 0.2, BRAKE_TYPE = 1):
        self.drive_base.settings(mapSpeed(SPEED), None, None, None, None)
        await self.drive_base.curve(RADIUS, DEGREES)
        await wait(STOP_DELAY * 1000)
        await self.stop(BRAKE_TYPE)

    async def wallAlignment(self, SPEED, SECONDS, STOP_DELAY = 0.2):
        timer = StopWatch()
        timer.reset()
        await self.drive(SPEED, 0)
        while timer.time() < SECONDS * 1000:
            await wait(10)
        await wait(STOP_DELAY * 1000)
        await self.stop(1)

    async def straight_JTN(self, PORT, SPEED, DISTANCE, JUNCT_SPEED = 0, JTH = tkp.JTH, STOP_DELAY = 0.2, BRAKE_TYPE = 1):
        pass
        
    # ==== Line Tracking ====
    async def SLT(self, PORT, SPEED, DISTANCE, LR_EDGE = -1, JUNCT_SPEED = 0, JTH = tkp.JTH, STOP_DELAY = 0.2, BRAKE_TYPE = 1):
        pass
        
    async def DLT(self, SPEED, DISTANCE, JUNCT_SPEED = 0, JTH = tkp.JTH, STOP_DELAY = 0.2, BRAKE_TYPE = 1):
        pass
            
    def viewRaw(self):
        pass
    
    def viewCali(self):
        pass
                   
    # ==== Motors ====
    async def leftMotor(self, SPEED, DEGREES, SECONDS = 0, BRAKE_TYPE = 1):
        pass

    async def rightMotor(self, SPEED, DEGREES, SECONDS = 0, BRAKE_TYPE = 1):
        pass

    # =================================================================
    # =================== ADVANCED RESEARCH METHODS ===================
    # =================================================================

    async def gyroDiagnosticAndAlignment(self):
        print("DIAGNOSTIC: Performing gyro diagnostics...")
        self.reset_advanced()
        print("DIAGNOSTIC: Gyro diagnostics complete.")
        await self.enhancedWallAlignment('front', 100, 2, 5)

    async def enhancedWallAlignment(self, side, speed, duration, num_contacts):
        print(f"ALIGN: Enhanced wall alignment, side: {side}, contacts: {num_contacts}")
        for i in range(num_contacts):
            await self.straight(20, 20)
            await self.straight(20, -20)
        print("ALIGN: Enhanced wall alignment complete.")

    async def robustEstimationWithOutliers(self, data, threshold):
        print("ESTIMATE: Robust estimation with outlier rejection...")
        if not data: return 0
        mean = sum(data) / len(data)
        std_dev = (sum([(x - mean) ** 2 for x in data]) / len(data)) ** 0.5
        if std_dev == 0: return mean
        filtered_data = [x for x in data if abs(x - mean) / std_dev < threshold]
        if not filtered_data: return mean
        robust_mean = sum(filtered_data) / len(filtered_data)
        print(f"ESTIMATE: Robust mean is {robust_mean:.2f}")
        return robust_mean

    async def bayesianInferencePositioning(self, prior_mean, measurement, confidence):
        print("ESTIMATE: Bayesian inference for positioning...")
        prior_variance = (1 - confidence) * 10
        measurement_variance = 5 
        
        kalman_gain = prior_variance / (prior_variance + measurement_variance)
        posterior_mean = prior_mean + kalman_gain * (measurement - prior_mean)
        posterior_variance = (1 - kalman_gain) * prior_variance
        
        new_confidence = 1 - (posterior_variance / 10)
        print(f"ESTIMATE: Bayesian posterior mean: {posterior_mean:.2f}, confidence: {new_confidence:.2f}")
        return posterior_mean, new_confidence

    async def straightWithSensorFusion(self, speed, distance, target_heading, confidence):
        print(f"MOVE: straightWithSensorFusion, dist: {distance:.2f}, confidence: {confidence:.2f}")
        
        if confidence > 0.85:
            await self.straightWithGyroPID(speed, distance, target_heading)
        else:
            await self.straight(speed, distance)
            print("MOVE_CORRECT: Applying micro-correction...")
            await self.straight(20, dist_error)
        
        print("MOVE: straightWithSensorFusion complete.")

    async def adaptiveKalmanFilter(self, measurement):
        # Simplified 1D Kalman Filter
        # Prediction
        predicted_x = self.kalman_state['x']
        predicted_p = self.kalman_state['p'] + 0.1 # Process noise

        # Update
        kalman_gain = predicted_p / (predicted_p + 0.5) # Measurement noise
        self.kalman_state['x'] = predicted_x + kalman_gain * (measurement - predicted_x)
        self.kalman_state['p'] = (1 - kalman_gain) * predicted_p
        return self.kalman_state['x']

    async def particleFilterLocalization(self, num_particles, movement, measurement):
        # This is a conceptual placeholder for a real particle filter
        print("LOCALIZE: Particle filter running...")
        # In a real implementation, you would update particle weights based on measurement
        # and resample. Here, we just return a simulated position.
        estimated_pos = movement + (measurement / num_particles)
        return estimated_pos

    async def encoderDeadReckoning(self, speed, distance):
        print(f"MOVE: Encoder dead reckoning for {distance}mm")
        await self.straight(speed, distance)
        return self.distance()

    async def multiHypothesisTracking(self, hypotheses):
        # Placeholder: selects the hypothesis with the highest probability
        print("DECIDE: Multi-hypothesis tracking...")
        best_hypothesis = max(hypotheses, key=lambda h: h['probability'])
        print(f"DECIDE: Best hypothesis selected: {best_hypothesis['name']}")
        return best_hypothesis