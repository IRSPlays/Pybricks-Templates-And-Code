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
    # Reset all the parameters back to default values
    def resetDefault(self):         
        self.SPEED = 50             # Target Speed
        self.SPEED_ALIGN = 30       # Wall Alignment Speed
        self.JUNCT_SPEED = 30       # Junction Speed 
        self.ACCEL_RATE = 50        # Acceleration Rate
        self.DECEL_RATE = 50        # Deceleration Rate
        self.GYRO_USAGE = True      # Gyro usage

mp = MotionParameters()

# Parameters for line tracking
class TrackingParameters:
    # Public member variables (accessible directly)
    def __init__(self): 
        self.IP = 20                # Initial Speed
        self.SPEED = 100             # Target Speed
        self.ACCEL_DIST = 100       # Acceleration Distance
        self.DECEL_DIST = 100       # Deceleration Distance
        self.JUNCT_SPEED = 20       # Junction Speed
        self.TH = 50                # Threshold
        self.JTH = 10               # Junction Threshold
        self.DIST_KP = 1.5            # Distance Proportional Gain
        self.DIST_KD = 10           # Distance Derivative Gain
        self.JTN_KP = self.DIST_KP  # Junction Proportional Gain
        self.JTN_KD = self.DIST_KD  # Junction Derivative Gain

    # Reset all the parameters back to default values
    def resetDefault(self):
        self.IP = 20                # Initial Speed
        self.SPEED = 100             # Target Speed
        self.ACCEL_DIST = 100       # Acceleration Distance
        self.DECEL_DIST = 100       # Deceleration Distance
        self.JUNCT_SPEED = 20       # Junction Speed
        self.TH = 50                # Threshold
        self.JTH = 10                # Junction Threshold
        self.DIST_KP = 1.5            # Distance Proportional Gain
        self.DIST_KD = 10           # Distance Derivative Gain
        self.JTN_KP = self.DIST_KP  # Junction Proportional Gain
        self.JTN_KD = self.DIST_KD  # Junction Derivative Gain

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
        drive_base.use_gyro(mp.GYRO_USAGE)
        reset()
        none_true = True if BRAKE_TYPE == 3 else False
        SPEED = mp.SPEED * SPEED if abs(SPEED) == 1 else SPEED 
        if DISTANCE > 0:
            DISTANCE = -DISTANCE if SPEED < 0 else DISTANCE 
            drive_base.settings(mapSpeed(SPEED), (mapAccel(mp.ACCEL_RATE), mapAccel(mp.DECEL_RATE)))
            await drive_base.straight(DISTANCE, stop[BRAKE_TYPE])
        if STOP_DELAY > 0:
            await robot.playTone(1000, STOP_DELAY * 1000)

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
        drive_base.use_gyro(mp.GYRO_USAGE)
        reset()
        none_true = True if BRAKE_TYPE == 3 else False
        SPEED = mp.SPEED * SPEED if abs(SPEED) == 1 else SPEED 
        RADIUS = axleTrack / 2 * RADIUS if abs(RADIUS) == 1 else RADIUS
        DEGREES = -abs(DEGREES) if SPEED < 0 else DEGREES
        drive_base.settings(mapSpeed(abs(SPEED)), (mapAccel(mp.ACCEL_RATE), mapAccel(mp.DECEL_RATE)))
        await drive_base.arc(RADIUS, DEGREES, None, stop[BRAKE_TYPE])
        if STOP_DELAY > 0:
            await robot.playTone(1000, STOP_DELAY * 1000)

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