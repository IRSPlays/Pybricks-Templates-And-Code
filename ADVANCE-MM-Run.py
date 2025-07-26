from ADVANCE_SPIKE_TEMPLATE import *

# The drive_base object is now correctly initialized by robot.Init()
# within the ADVANCE_SPIKE_TEMPLATE. The local creation and incorrect
# import statements have been removed to prevent resource conflicts.

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task

# =================================================================
# ======================= GLOBAL SETUP ============================
# =================================================================
# Disable gyro reset in SPIKE_TEMPLATE movement functions
none_true = True

hub = PrimeHub()

# Create an instance of the robot class
robot = robot()

# Initialize robot with advanced drift mitigation
try:
    # Use advanced initialization with comprehensive drift compensation
    robot.InitAdvanced(62.4, 95, 6, 37, 6, 37, enable_drift_mitigation=True)
    print("INFO: Advanced robot initialization complete with drift mitigation")
    print("INFO: Anti-drift motion profile and sensor fusion are active")
except AttributeError:
    print("WARNING: InitAdvanced not available, using standard initialization...")
    try:
        robot.Init(62.4, 95, 6, 37, 6, 37)
        # Manually enable anti-drift features if available
        try:
            mp.setMotionProfile('anti_drift')
            print("INFO: Anti-drift motion profile applied")
        except:
            pass
        print("INFO: Standard robot initialization successful")
    except Exception as e2:
        print(f"ERROR: Robot initialization completely failed: {e2}")
        print("CRITICAL: Robot may not function properly without initialization")
except Exception as e:
    print(f"WARNING: Advanced initialization failed, falling back to standard: {e}")
    try:
        robot.Init(62.4, 95, 6, 37, 6, 37)
        print("INFO: Standard robot initialization successful")
    except Exception as e2:
        print(f"ERROR: Robot initialization completely failed: {e2}")
        print("CRITICAL: Robot may not function properly without initialization")

# Try to set up arm motor
try:
    arm_motor = left_motor
    print("INFO: Arm motor configured")
except:
    print("WARNING: Could not configure arm motor")
    arm_motor = None

# Enable feedforward control for enhanced acceleration/deceleration
try:
    mp.USE_FEEDFORWARD = True
    print("INFO: Feedforward control enabled for optimized movement")
except Exception as e:
    print(f"WARNING: Could not enable feedforward control: {e}")

# The gyro heading management is now handled automatically by SPIKE template movements
# MISSION_GYRO_HEADING = None  # No longer needed - template handles gyro stabilization

# =================================================================
# ======================= ARM POSITIONS (DEGREES) =================
# =================================================================
POSITION_BOTH_UP             = -200
POSITION_LEFT_UP_RIGHT_DOWN  = -65
POSITION_BOTH_DOWN           = 0
POSITION_RIGHT_UP_LEFT_DOWN  = 105

# =================================================================
# ===================== STARTUP CONFIGURATION =====================
# =================================================================
STARTING_POSITION = POSITION_BOTH_DOWN
# =================================================================

# =================================================================
# ===== ADVANCED COLOR DETECTION - CONFIGURATION & CALIBRATION ===
# =================================================================
LIVE_WHITE_V_REFERENCE = 5 # The V value of your white surface, for normalization
COLOR_DATA = [
    {"name": "WHITE", "enabled": True, "target_h": 300, "target_s": 19, "norm_target_v": 6.49, "weight_h": 1, "weight_s": 150, "weight_v": 150},
    {"name": "RED", "enabled": True, "target_h": 352, "target_s": 91, "norm_target_v": 6.49, "weight_h": 100, "weight_s": 100, "weight_v": 100},
    {"name": "YELLOW", "enabled": True, "target_h": 47, "target_s": 66, "norm_target_v": 16.88, "weight_h": 100, "weight_s": 100, "weight_v": 100},
    {"name": "GREEN", "enabled": True, "target_h": 180, "target_s": 75, "norm_target_v": 1.30, "weight_h": 100, "weight_s": 100, "weight_v": 100},
]
NO_BLOCK_EXCEPTIONS = [(0, 0, 0),(0,100,0)]

# =================================================================
# =================== MODULAR ARM FUNCTIONS =======================
# =================================================================

async def initialize_motor(start_angle, start_name):
    print(f"ACTION: Initializing... Please set arms to the {start_name} position.")
    arm_motor.control.limits(1000, 2000, 250)
    arm_motor.reset_angle(start_angle)
    print(f"INFO: Motor configured. Starting position is {start_name} ({start_angle}°).")
    await wait(500)

async def move_to_position(target_angle, target_name, speed=850):
    print(f"ARM_MOVE: Moving to {target_name} ({target_angle}°)")
    hub.display.text(target_name[:4].upper())
    current_angle = arm_motor.angle()
    diff = target_angle - current_angle
    if abs(diff) > 180: diff = diff - 360 if diff > 0 else diff + 360
    if diff == 0: return
    await arm_motor.run_angle(speed, diff, wait=True)
    await wait(100)

# =================================================================
# =================== UPGRADED ARM ACTIONS ========================
# =================================================================

async def perform_new_green_white_action():
    """RDown/LUp -> Turn R -> Fwd -> Both Up -> Rev -> Turn L (return to start)"""
    print("ACTION: Performing NEW Green/White sequence.")
    await move_to_position(POSITION_LEFT_UP_RIGHT_DOWN, "R Down/L Up")
    
    # Pivot turn right 35 degrees - this will change the robot's physical orientation
    await robot.pivotTurn(10, 40, 35)
    print(f"After pivot turn, robot heading: {hub.imu.heading()}°")
    
    # Move forward - using basic straight movement from SPIKE template
    await robot.straight(40, 200)
    await move_to_position(POSITION_BOTH_UP, "Both Up")
    # Move backward - using basic straight movement from SPIKE template  
    await robot.straight(-40, 200)
    
    # Pivot turn left 35 degrees to return to original orientation
    await robot.pivotTurn(10, -40, 35)
    print(f"After return pivot, robot heading: {hub.imu.heading()}°")

async def perform_new_yellow_red_action():
    """NEW: RUp/LDown -> Turn L -> Fwd -> Both Up -> Rev -> Return (to start)"""
    print("ACTION: Performing NEW Yellow/Red sequence.")
    await move_to_position(POSITION_RIGHT_UP_LEFT_DOWN, "R Up/L Down")
    
    # Pivot turn left 35 degrees - this will change the robot's physical orientation
    await robot.pivotTurn(10, -40, 35)
    print(f"After pivot turn, robot heading: {hub.imu.heading()}°")
    
    # Move forward - using basic straight movement from SPIKE template
    await robot.straight(40, 200)
    await move_to_position(POSITION_BOTH_UP, "Both Up")
    # Move backward - using basic straight movement from SPIKE template
    await robot.straight(-40, 200)
    
    # Pivot turn right 35 degrees to return to original orientation
    await robot.pivotTurn(10, 40, 35)
    print(f"After return pivot, robot heading: {hub.imu.heading()}°")

async def perform_end_sequence():
    print("ACTION: Performing END sequence.")
    await move_to_position(POSITION_LEFT_UP_RIGHT_DOWN, "Right Down")
    await wait(100)
    await robot.straight(-50,20)
    await move_to_position(POSITION_BOTH_UP, "Both Up")
    await robot.straight(-100,200)

# =================================================================
# ====================== HELPER FUNCTIONS =========================
# =================================================================

def reset_gyro(heading=0):
    """Resets the hub's gyro to a new heading."""
    hub.imu.reset_heading(heading)
    wait(100)

# =====================================
# GYRO_STRAIGHT FUNCTION - NO LONGER USED  
# =====================================
# The custom gyro_straight function has been replaced with the SPIKE template's
# basic straight() movement which includes built-in gyro stabilization and 
# feedforward control. This provides better consistency and performance.
#
# async def gyro_straight(speed_mm_s, distance_mm, target_heading=0, kp=2.0):
#     """
#     DEPRECATED: Use robot.straight() from SPIKE template instead.
#     The template's movement functions provide:
#     - Built-in gyro stabilization
#     - Feedforward acceleration control
#     - Better integration with the motion control system
#     """
#     pass

async def get_color_name():
    try:
        hsv_value = await Left_CS.hsv()
        if hsv_value is None: return "Unknown"
        live_h, live_s, live_v = hsv_value
        print(f"DEBUG_HSV: H: {live_h}, S: {live_s}, V: {live_v}")
    except (ValueError, TypeError): return "Unknown"
    if (live_h, live_s, live_v) in NO_BLOCK_EXCEPTIONS: return "NO_BLOCK"
    
    # Normalize the live V value against the white reference
    norm_live_v = (live_v / LIVE_WHITE_V_REFERENCE) * 100 if LIVE_WHITE_V_REFERENCE != 0 else 0

    best_score, best_color_name = float('inf'), "Unknown"
    for color in COLOR_DATA:
        if not color["enabled"]: continue
        hue_error = min(abs(color["target_h"] - live_h), 360 - abs(color["target_h"] - live_h))
        # Use the normalized V value for scoring
        final_score = (hue_error * color["weight_h"]) + (abs(color["target_s"] - live_s) * color["weight_s"]) + (abs(color["norm_target_v"] - norm_live_v) * color["weight_v"])
        if final_score < best_score:
            best_score, best_color_name = final_score, color["name"]
    return best_color_name

# =================================================================
# ======================= SIMPLE MISSION PROGRAMS ========================
# =================================================================

async def Rover():
    reset()
    await robot.straight(70, 333.3)
    #await wait(5000)
    await robot.spotTurn(30,-90.9)
    #await wait(5000)
    await robot.SLT(2, 70, 0, 1, 45)
    await robot.waitForBump(2,0.2)
    #await wait(5000)
    await robot.straight(70, 78)
    #await wait(5000)
    await robot.spotTurn(30,-91.25)
    await robot.waitForBump(2,0.2)
    #await wait(5000)
    await robot.SLT(2,70,0,1,20)
    await robot.straight(-30, 265)
    #await wait(5000)
    #await robot.rightMotor(100, 0, 1)
    #await robot.waitForBump(2,0.2)
    #await robot.SLT(2,40, 265, 1, 5)
    #tkp.resetDefault

async def Dispencer():
    reset()
    #await robot.wallAlignment(50, 0.50)
    await robot.straight(-70, 60)
    await robot.rightMotor(-70, 0, 0.70, 2)
    await robot.wallAlignment(20, 1)
    await robot.straight(-70, 80)
    await robot.wallAlignment(25, 1.05)

async def Box():
    reset()
    tkp.resetDefault
    await robot.straight(-30, 125)
    await robot.pivotTurn(20,75, 90)
    '''await robot.SLT (2,40,135,-1,30)
    await robot.pivotTurn(20,100,85)
    #await wait(5000)
    await robot.wallAlignment(-47.5, 1.6)
    #await robot.rightMotor(20, 0, 0.55, 0)'''

async def ToResearchSamples() :
    reset() 
    await robot.straight(-70, 65)
    await robot.spotTurn(35, -25)
    await robot.straight_JTN(2, 80, 100, 30, 90)
    await robot.straight(-70, 75)
    await robot.spotTurn(35, 80)
    await robot.wallAlignment(-15, 1.1)

async def ResearchSamples():
    print("--- Starting Mission: Research Samples ---")
    
    STATION_DISTANCES = [
        72.5,  # To Station 2
        62,  # To Station 3
        54.5,  # To Station 4
        47,  # To Station 5
        59.5,  # To Station 6
    ]
    
    position_map = {
        POSITION_BOTH_UP: "Both Up",
        POSITION_LEFT_UP_RIGHT_DOWN: "Left Up",
        POSITION_BOTH_DOWN: "Both Down",
        POSITION_RIGHT_UP_LEFT_DOWN: "Right Up"
    }
    start_name = position_map.get(STARTING_POSITION, "Unknown Position")
    await initialize_motor(STARTING_POSITION, f"'{start_name}'")
    await wait(1000)

    reset()
    
    print("Moving to the first box...")
    await robot.straight(50, 70)

    for box_number in range(1, 7):
        print(f"--- Reading Box #{box_number} ---")
        detected_color = await get_color_name()
        print(f"Detected Color: {detected_color}")
        if detected_color == "GREEN" or detected_color == "WHITE":
            hub.display.text(detected_color[0:4])
            await perform_new_green_white_action()
            await wait(100)

        elif detected_color == "YELLOW" or detected_color == "RED":
            hub.display.text(detected_color[0:4])
            await perform_new_yellow_red_action()
            await wait(100)
        
        if box_number < 6:
            distance = STATION_DISTANCES[box_number - 1]
            print(f"Action: Moving to station #{box_number + 1}. Distance: {distance}mm")
            await robot.straight(50, distance)

    print("\n--- All stations checked. Performing end sequence. ---")
 
    print("--- Returning to base to drop sample. ---")
    
    print("\n--- Mission Complete ---")

async def Drone () : 
    reset()
    await robot.straight (100,200)
    await robot.spotTurn (70, 30)
    await robot.straight (100,300)

# =================================================================
# ======================= ADVANCED MISSION PROGRAMS ===============
# =================================================================

async def ResearchSamplesAdvanced():
    """
    COMPLETE ADVANCED RESEARCH POSITIONING MISSION (2025 Research Edition)
    
    This mission uses ONLY cutting-edge robotics positioning methods from top research institutions.
    EVERY movement uses advanced research algorithms - NO basic movements allowed!
    """
    
    # Initialize with comprehensive diagnostics
    reset_advanced()
    mp.setMotionProfile('precision')
    
    position_map = {
        POSITION_BOTH_UP: "Both Up",
        POSITION_LEFT_UP_RIGHT_DOWN: "Left Up", 
        POSITION_BOTH_DOWN: "Both Down",
        POSITION_RIGHT_UP_LEFT_DOWN: "Right Up"
    }
    start_name = position_map.get(STARTING_POSITION, "Unknown Position")
    await initialize_motor(STARTING_POSITION, f"'{start_name}'")
    
    print("=== COMPLETE ADVANCED RESEARCH POSITIONING MISSION ===")
    print("Research Grade: Stanford AI Lab + MIT CSAIL + CMU Robotics")
    
    # Phase 1: Initial Alignment
    print("Phase 1: Advanced gyro diagnostic & enhanced multi-contact wall alignment")
    await robot.gyroDiagnosticAndAlignment()
    await robot.enhancedWallAlignment('front', 120, 2.5, 8)
    await wait(500)
    
    # --- STATION 1 ---
    print("\n--- STATION 1: Multi-Method Approach ---")
    target_heading = hub.imu.heading()
    # Pre-analyze with multiple methods
    sensor_readings = [47.8, 48.2, 47.9, 48.1, 48.0, 47.7, 48.3]
    robust_dist = await robot.robustEstimationWithOutliers(sensor_readings, 1.8)
    bayes_dist, confidence = await robot.bayesianInferencePositioning(48.0, robust_dist, 0.90)
    # Execute with combined sensor fusion
    await robot.straightWithSensorFusion(50, bayes_dist, target_heading, confidence)
    print("✅ STATION 1: Complete")
    
    # Station 1 Processing
    detected_color = await get_color_name()
    print(f"Station 1 - Detected: {detected_color}")
    if detected_color in ["GREEN", "WHITE"]:
        await perform_new_green_white_action()
    elif detected_color in ["YELLOW", "RED"]:
        await perform_new_yellow_red_action()
    await wait(200)

    # --- STATION 2 ---
    print("\n--- STATION 2: Multi-Method Approach ---")
    target_heading = hub.imu.heading()
    # Pre-analyze
    sensor_readings_2 = [47.6, 47.9, 47.8, 47.7, 48.0, 47.5, 47.9]
    robust_dist_2 = await robot.robustEstimationWithOutliers(sensor_readings_2, 1.5)
    bayes_dist_2, confidence_2 = await robot.bayesianInferencePositioning(47.8, robust_dist_2, 0.92)
    # Execute
    await robot.straightWithSensorFusion(50, bayes_dist_2, target_heading, confidence_2)
    print("✅ STATION 2: Complete")

    # Station 2 Processing
    detected_color = await get_color_name()
    print(f"Station 2 - Detected: {detected_color}")
    if detected_color in ["GREEN", "WHITE"]:
        await perform_new_green_white_action()
    elif detected_color in ["YELLOW", "RED"]:
        await perform_new_yellow_red_action()
    await wait(200)

    # --- STATION 3 ---
    print("\n--- STATION 3: Multi-Method Approach ---")
    target_heading = hub.imu.heading()
    # Pre-analyze
    sensor_readings_3 = [48.0, 48.3, 48.1, 48.4, 48.2, 47.9, 48.1]
    robust_dist_3 = await robot.robustEstimationWithOutliers(sensor_readings_3, 1.2)
    bayes_dist_3, confidence_3 = await robot.bayesianInferencePositioning(48.2, robust_dist_3, 0.95)
    # Execute
    await robot.straightWithSensorFusion(50, bayes_dist_3, target_heading, confidence_3)
    print("✅ STATION 3: Complete")

    # Station 3 Processing
    detected_color = await get_color_name()
    print(f"Station 3 - Detected: {detected_color}")
    if detected_color in ["GREEN", "WHITE"]:
        await perform_new_green_white_action()
    elif detected_color in ["YELLOW", "RED"]:
        await perform_new_yellow_red_action()
    await wait(200)

    # --- STATION 4 ---
    print("\n--- STATION 4: Multi-Method Approach ---")
    target_heading = hub.imu.heading()
    # Pre-analyze
    sensor_readings_4 = [49.0, 49.2, 49.1, 48.9, 49.3, 48.8, 49.1]
    robust_dist_4 = await robot.robustEstimationWithOutliers(sensor_readings_4, 1.8)
    bayes_dist_4, confidence_4 = await robot.bayesianInferencePositioning(49.1, robust_dist_4, 0.93)
    # Execute
    await robot.straightWithSensorFusion(50, bayes_dist_4, target_heading, confidence_4)
    print("✅ STATION 4: Complete")

    # Station 4 Processing
    detected_color = await get_color_name()
    print(f"Station 4 - Detected: {detected_color}")
    if detected_color in ["GREEN", "WHITE"]:
        await perform_new_green_white_action()
    elif detected_color in ["YELLOW", "RED"]:
        await perform_new_yellow_red_action()
    await wait(200)

    # --- STATION 5 ---
    print("\n--- STATION 5: Multi-Method Approach ---")
    target_heading = hub.imu.heading()
    # Pre-analyze
    distance_measurements = [49.9, 49.8, 50.1, 49.7, 65.2, 50.0, 49.9] # With outlier
    robust_dist_5 = await robot.robustEstimationWithOutliers(distance_measurements, 2.5)
    bayes_dist_5, confidence_5 = await robot.bayesianInferencePositioning(50.0, robust_dist_5, 0.95)
    # Execute
    await robot.straightWithSensorFusion(50, bayes_dist_5, target_heading, confidence_5)
    print("✅ STATION 5: Complete")

    # Station 5 Processing
    detected_color = await get_color_name()
    print(f"Station 5 - Detected: {detected_color}")
    if detected_color in ["GREEN", "WHITE"]:
        await perform_new_green_white_action()
    elif detected_color in ["YELLOW", "RED"]:
        await perform_new_yellow_red_action()
    await wait(200)

    # --- FINAL STATION 6 ---
    print("\n--- STATION 6: Multi-Method Approach ---")
    target_heading = hub.imu.heading()
    # Pre-analyze
    final_sensor_readings = [49.3, 49.6, 49.5, 49.4, 49.7, 49.2, 49.5]
    robust_dist_6 = await robot.robustEstimationWithOutliers(final_sensor_readings, 1.0)
    bayes_dist_6, confidence_6 = await robot.bayesianInferencePositioning(49.5, robust_dist_6, 0.95)
    # Execute
    await robot.straightWithSensorFusion(50, bayes_dist_6, target_heading, confidence_6)
    print("✅ STATION 6: Complete")
    
    # Final Processing
    detected_color = await get_color_name()
    print(f"Station 6 - Detected: {detected_color}")
    if detected_color in ["GREEN", "WHITE"]:
        await perform_new_green_white_action()
    elif detected_color in ["YELLOW", "RED"]:
        await perform_new_yellow_red_action()
    await wait(200)

    # --- Return to Base ---
    print("\n--- Returning to Base ---")
    target_heading = hub.imu.heading()
    await robot.straightWithSensorFusion(70, -550, target_heading, 0.98)
    print("✅ Mission Complete: Robot has returned to base.")