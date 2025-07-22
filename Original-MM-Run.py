from SPIKE_TEMPLATE import*
import umath

# =================================================================
# ======================= GLOBAL SETUP ============================
# =================================================================
hub = PrimeHub()
robot.Init(62.4,95,6,37,6,37)
arm_motor = left_motor

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
    await robot.spotTurn(60,35)
    await robot.straight(50, 40)
    await move_to_position(POSITION_BOTH_UP, "Both Up")
    await robot.straight(-50, 60)
    await robot.spotTurn(-40, -2.5)

async def perform_new_yellow_red_action():
    """NEW: RUp/LDown -> Turn R -> Fwd -> Both Up -> Rev -> Return (to start)"""
    print("ACTION: Performing NEW Yellow/Red sequence.")
    await move_to_position(POSITION_RIGHT_UP_LEFT_DOWN, "R Up/L Down")
    await robot.spotTurn(-60,35)
    await robot.straight(50, 40)
    await move_to_position(POSITION_BOTH_UP, "Both Up")
    await robot.straight(-50, 60)
    await robot.spotTurn(40, 2.5)

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
# ======================= MISSION PROGRAMS ========================
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

run_task (Rover())
#run_task (Dispencer())
#run_task (Box())
#run_task (ToResearchSamples())
run_task (ResearchSamples())
#run_task (Drone())