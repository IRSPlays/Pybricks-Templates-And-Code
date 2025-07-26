from SPIKE_TEMPLATE2 import*
import umath

# =================================================================
# ======================= GLOBAL SETUP ============================
# =================================================================
hub = PrimeHub()
robot.Init(62.4,90.5,6,37,5,41)
arm_motor = right_motor

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
    await robot.spotTurn(40,35)
    await robot.straight(40, 40)
    await move_to_position(POSITION_BOTH_UP, "Both Up")
    await robot.straight(-40, 60)
    await robot.spotTurn(-40, 35)

async def perform_new_yellow_red_action():
    """NEW: RUp/LDown -> Turn R -> Fwd -> Both Up -> Rev -> Return (to start)"""
    print("ACTION: Performing NEW Yellow/Red sequence.")
    await move_to_position(POSITION_RIGHT_UP_LEFT_DOWN, "R Up/L Down")
    await robot.spotTurn(-40,35)
    await robot.straight(40, 40)
    await move_to_position(POSITION_BOTH_UP, "Both Up")
    await robot.straight(-40, 60)
    await robot.spotTurn(40, 35)

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

async def get_safe_reflection():
    """
    Safely get reflection value from the color sensor, handling async issues.
    Returns a numeric value or None if unable to read.
    """
    try:
        # Method 1: Try awaiting reflection directly (like HSV)
        try:
            reflection = await Left_CS.reflection()
            if isinstance(reflection, (int, float)):
                return reflection
        except:
            pass
        
        # Method 2: Try direct reflection call
        try:
            reflection = Left_CS.reflection()
            if isinstance(reflection, (int, float)):
                return reflection
        except:
            pass
                
        # Method 3: Try RGB average as fallback with await
        try:
            r, g, b = await Left_CS.rgb()
            if all(isinstance(x, (int, float)) for x in [r, g, b]):
                return (r + g + b) / 3
        except:
            pass
            
        # Method 4: Try RGB without await
        try:
            r, g, b = Left_CS.rgb()
            if all(isinstance(x, (int, float)) for x in [r, g, b]):
                return (r + g + b) / 3
        except:
            pass
            
        # Method 5: Try ambient light with await
        try:
            ambient = await Left_CS.ambient()
            if isinstance(ambient, (int, float)):
                return ambient
        except:
            pass
            
        # Method 6: Try ambient light without await
        try:
            ambient = Left_CS.ambient()
            if isinstance(ambient, (int, float)):
                return ambient
        except:
            pass
            
        return None
        
    except Exception as e:
        print(f"Error getting reflection: {e}")
        return None

async def align_left_to_right_slow(scan_speed=50, step_size=5.0):
    """
    Turn RIGHT +10°, then scan from RIGHT (+10°) to LEFT (-10°) with steps.
    Used after RED/YELLOW actions for precise alignment.
    Total scan range: 20° (+10° to -10°)
    """
    print("--- Starting RIGHT-TO-LEFT alignment scan (RED/YELLOW) ---")
    
    initial_heading = hub.imu.heading()
    print(f"Initial heading: {initial_heading:.1f}°")
    
    # FIRST: Turn 10 degrees to the RIGHT - MUCH FASTER
    pre_turn_angle = 10  # Turn 10 degrees right
    print(f"Pre-turning RIGHT by {pre_turn_angle:.1f}° for RED/YELLOW alignment...")
    await robot.spotTurn(pre_turn_angle, scan_speed)  # Note: angle first, then speed
    await wait(300)
    
    # We're now at +10°, scan from here to -10° (20° total range)
    scan_range = 20
    best_value = 0
    best_angle = 10  # Start at +10°
    current_angle = 10
    
    steps = int(scan_range / step_size)
    print(f"Scanning {steps} positions from RIGHT(+10°)→LEFT(-10°) FAST...")
    
    for step in range(steps + 1):
        # Get sensor reading
        sensor_value = await get_safe_reflection()
        
        if sensor_value is not None:
            print(f"R→L Step {step}: Angle {current_angle:.1f}°, Value {sensor_value:.1f}")
            
            if sensor_value > best_value:
                best_value = sensor_value
                best_angle = current_angle
                print(f"  → NEW BEST: {best_value:.1f} at {best_angle:.1f}°")
        else:
            print(f"R→L Step {step}: Could not read sensor")
        
        # Move to next position (step to the LEFT)
        if step < steps:
            await robot.spotTurn(-step_size, scan_speed)  # NEGATIVE for left turn
            current_angle -= step_size
            await wait(200)
    
    # Go back to the best position found
    print(f"Best reading: {best_value:.1f} at angle {best_angle:.1f}°")
    
    # Calculate turn needed to get to best position
    turn_needed = best_angle - current_angle
    if abs(turn_needed) > 1:
        print(f"Going to BEST position: turning {turn_needed:.1f}°")
        await robot.spotTurn(turn_needed, scan_speed)
    else:
        print("Already at BEST position!")
    
    # Final confirmation
    final_value = await get_safe_reflection()
    final_heading = hub.imu.heading()
    print(f"Final R→L: Heading {final_heading:.1f}°, Value {final_value}")
    print("--- RIGHT-TO-LEFT alignment complete ---")
    return True

async def align_right_to_left_slow(scan_speed=50, step_size=5.0):
    """
    Turn LEFT -10°, then scan from LEFT (-10°) to RIGHT (+10°) with steps.
    Used after GREEN/WHITE actions for precise alignment.
    Total scan range: 20° (-10° to +10°)
    """
    print("--- Starting LEFT-TO-RIGHT alignment scan (GREEN/WHITE) ---")
    
    initial_heading = hub.imu.heading()
    print(f"Initial heading: {initial_heading:.1f}°")
    
    # FIRST: Turn 10 degrees to the LEFT - MUCH FASTER
    pre_turn_angle = -10  # Turn 10 degrees left (negative angle)
    print(f"Pre-turning LEFT by {abs(pre_turn_angle):.1f}° for GREEN/WHITE alignment...")
    await robot.spotTurn(pre_turn_angle, scan_speed)  # Note: angle first, then speed
    await wait(300)
    
    # We're now at -10°, scan from here to +10° (20° total range)
    scan_range = 20
    best_value = 0
    best_angle = -10  # Start at -10°
    current_angle = -10
    
    steps = int(scan_range / step_size)
    print(f"Scanning {steps} positions from LEFT(-10°)→RIGHT(+10°) FAST...")
    
    for step in range(steps + 1):
        # Get sensor reading
        sensor_value = await get_safe_reflection()
        
        if sensor_value is not None:
            print(f"L→R Step {step}: Angle {current_angle:.1f}°, Value {sensor_value:.1f}")
            
            if sensor_value > best_value:
                best_value = sensor_value
                best_angle = current_angle
                print(f"  → NEW BEST: {best_value:.1f} at {best_angle:.1f}°")
        else:
            print(f"L→R Step {step}: Could not read sensor")
        
        # Move to next position (step to the RIGHT)
        if step < steps:
            await robot.spotTurn(step_size, scan_speed)  # POSITIVE for right turn
            current_angle += step_size
            await wait(200)
    
    # Go back to the best position found
    print(f"Best reading: {best_value:.1f} at angle {best_angle:.1f}°")
    
    # Calculate turn needed to get to best position
    turn_needed = best_angle - current_angle
    if abs(turn_needed) > 1:
        print(f"Going to BEST position: turning {turn_needed:.1f}°")
        await robot.spotTurn(turn_needed, scan_speed)
    else:
        print("Already at BEST position!")
    
    # Final confirmation
    final_value = await get_safe_reflection()
    final_heading = hub.imu.heading()
    print(f"Final L→R: Heading {final_heading:.1f}°, Value {final_value}")
    print("--- LEFT-TO-RIGHT alignment complete ---")
    return True

async def align_to_highest_reflection_left_to_right(scan_range=30, scan_speed=8, step_size=2):
    """
    Scan from left to right and STOP at the highest reflection position.
    Does NOT return to center - stays at the best position found.
    """
    print("--- Starting LEFT-TO-RIGHT alignment scan ---")
    
    initial_heading = hub.imu.heading()
    print(f"Initial heading: {initial_heading:.1f}°")
    
    # Move to start position (left side)
    start_angle = -scan_range / 2
    await robot.spotTurn(scan_speed, start_angle, 0)
    
    best_value = 0
    best_step = 0
    current_angle = start_angle
    
    steps = int(scan_range / step_size)
    print(f"Scanning {steps} positions from left to right...")
    
    readings = []  # Store all readings
    
    for step in range(steps + 1):
        # Get sensor reading
        sensor_value = await get_safe_reflection()
        heading = hub.imu.heading()
        
        if sensor_value is not None:
            print(f"Step {step}: Angle {current_angle:.1f}°, Value {sensor_value:.1f}")
            
            readings.append({
                'step': step,
                'angle': current_angle,
                'value': sensor_value
            })
            
            if sensor_value > best_value:
                best_value = sensor_value
                best_step = step
        else:
            print(f"Step {step}: Could not read sensor")
            readings.append({
                'step': step,
                'angle': current_angle,
                'value': 0
            })
        
        # Move to next position
        if step < steps:
            await robot.spotTurn(scan_speed, step_size, 0)
            current_angle += step_size
    
    # Now go back to the best position found
    print(f"Best reading: {best_value:.1f} at step {best_step}")
    
    # Calculate how many steps back we need to go
    current_step = steps
    steps_back = current_step - best_step
    
    if steps_back > 0:
        turn_back = -(steps_back * step_size)
        print(f"Going back {steps_back} steps ({turn_back:.1f}°) to best position")
        await robot.spotTurn(scan_speed, turn_back, 0.2)
    else:
        print("Already at best position!")
    
    # Final confirmation
    final_value = await get_safe_reflection()
    final_heading = hub.imu.heading()
    print(f"Final: Heading {final_heading:.1f}°, Value {final_value}")
    print("--- LEFT-TO-RIGHT alignment complete ---")
    return True

async def align_to_highest_reflection_simple(scan_range=30, scan_speed=8, step_size=2):
    """
    Simplified version of reflection alignment with better error handling.
    """
    print("--- Starting SIMPLE alignment scan ---")
    
    initial_heading = hub.imu.heading()
    print(f"Initial heading: {initial_heading:.1f}°")
    
    # Move to start position
    start_angle = -scan_range / 2
    await robot.spotTurn(scan_speed, start_angle, 0)
    
    best_value = 0
    best_angle = 0
    current_angle = start_angle
    
    steps = int(scan_range / step_size)
    print(f"Scanning {steps} positions...")
    
    for step in range(steps + 1):
        # Get sensor reading
        sensor_value = await get_safe_reflection()
        heading = hub.imu.heading()
        
        if sensor_value is not None:
            print(f"Step {step}: Angle {current_angle:.1f}°, Value {sensor_value:.1f}")
            
            if sensor_value > best_value:
                best_value = sensor_value
                best_angle = current_angle
        else:
            print(f"Step {step}: Could not read sensor")
        
        # Move to next position
        if step < steps:
            await robot.spotTurn(scan_speed, step_size, 0)
            current_angle += step_size
    
    # Move to best position
    turn_needed = best_angle - current_angle
    if abs(turn_needed) > 1:
        print(f"Moving to best position: turning {turn_needed:.1f}°")
        await robot.spotTurn(scan_speed, turn_needed, 0.2)
    
    # Final confirmation
    final_value = await get_safe_reflection()
    final_heading = hub.imu.heading()
    print(f"Final: Heading {final_heading:.1f}°, Value {final_value}")
    print("--- Alignment complete ---")
    return True

async def align_to_highest_reflection(scan_range=60, scan_speed=10, step_size=3):
    """
    Scan left to right to find the position with highest reflection value.
    This helps align the robot perfectly with the next sample.
    
    PARAMETERS:
    - scan_range: Total angle range to scan in degrees (default: 60)
    - scan_speed: Speed for scanning movements (default: 10)
    - step_size: Step size in degrees for each measurement (default: 3)
    """
    print("--- Starting alignment scan for highest reflection ---")
    
    # Store initial position
    initial_heading = hub.imu.heading()
    print(f"Initial heading: {initial_heading}°")
    
    # Move to start position (left side of scan range)
    start_angle = -scan_range / 2
    await robot.spotTurn(scan_speed, start_angle, 0)
    
    # Variables to track best position
    best_reflection = 0
    best_angle = 0
    current_scan_angle = start_angle
    reflection_readings = []
    
    # Scan from left to right
    steps = int(scan_range / step_size)
    print(f"Scanning {steps} positions over {scan_range}° range...")
    
    for step in range(steps + 1):
        try:
            # Take reflection reading - try different methods to get proper value
            current_reflection = None
            try:
                current_reflection = Left_CS.reflection()
                # Check if it's a wait object and handle it
                if hasattr(current_reflection, '__class__') and 'wait' in str(current_reflection.__class__):
                    # This is likely an async method, let's try to get the actual value
                    current_reflection = await current_reflection
            except:
                # Fallback: try getting RGB and calculate brightness
                try:
                    r, g, b = Left_CS.rgb()
                    current_reflection = (r + g + b) / 3  # Average RGB as brightness
                except:
                    current_reflection = 50  # Default fallback value
            
            current_heading = hub.imu.heading()
            
            # Ensure we have a numeric value
            if not isinstance(current_reflection, (int, float)):
                current_reflection = 50  # Default fallback
                
            reflection_readings.append({
                'angle': current_scan_angle,
                'heading': current_heading,
                'reflection': current_reflection
            })
            
            print(f"Step {step}: Angle {current_scan_angle:.1f}°, Heading {current_heading:.1f}°, Reflection {current_reflection}")
            
            # Check if this is the best reflection so far
            if current_reflection > best_reflection:
                best_reflection = current_reflection
                best_angle = current_scan_angle
                best_heading = current_heading
            
            # Move to next position (except on last step)
            if step < steps:
                await robot.spotTurn(scan_speed, step_size, 0)
                current_scan_angle += step_size
                
        except Exception as e:
            print(f"Error during scan step {step}: {e}")
            continue
    
    # Move to the position with highest reflection
    print(f"Best reflection: {best_reflection} at scan angle {best_angle:.1f}°")
    
    # Calculate how much to turn to get to best position
    current_position = current_scan_angle
    turn_needed = best_angle - current_position
    
    if abs(turn_needed) > 1:  # Only turn if significant difference
        print(f"Turning {turn_needed:.1f}° to align with highest reflection")
        await robot.spotTurn(scan_speed, turn_needed, 0.2)
    else:
        print("Already at optimal position")
    
    # Final reading confirmation
    try:
        # Get final reflection reading with proper error handling
        final_reflection = None
        try:
            final_reflection = Left_CS.reflection()
            # Check if it's a wait object and handle it
            if hasattr(final_reflection, '__class__') and 'wait' in str(final_reflection.__class__):
                final_reflection = await final_reflection
        except:
            # Fallback: try getting RGB and calculate brightness
            try:
                r, g, b = Left_CS.rgb()
                final_reflection = (r + g + b) / 3
            except:
                final_reflection = "Error reading"
        
        final_heading = hub.imu.heading()
        print(f"Final position - Heading: {final_heading:.1f}°, Reflection: {final_reflection}")
    except:
        print("Could not get final readings")
    
    print("--- Alignment complete ---")
    return True

async def find_sample_center(search_speed=20, max_search_distance=100, threshold_difference=15):
    """
    Find the center of a research sample prop using HSV color sensor readings.
    
    PARAMETERS:
    - search_speed: Speed to move while searching (default: 20)
    - max_search_distance: Maximum distance to search in mm (default: 100)
    - threshold_difference: HSV difference threshold to detect sample edges (default: 15)
    
    Returns: True if successfully centered, False if no sample found
    """
    print("--- Starting sample centering procedure ---")
    
    # Store initial readings to establish baseline
    initial_hsv = Left_CS.hsv()
    if initial_hsv is None:
        print("ERROR: Cannot read initial HSV values")
        return False
    
    initial_h, initial_s, initial_v = initial_hsv
    print(f"Initial HSV: H:{initial_h}, S:{initial_s}, V:{initial_v}")
    
    # Phase 1: Find the left edge of the sample
    print("Phase 1: Finding left edge of sample...")
    left_edge_found = False
    distance_traveled = 0
    
    while distance_traveled < max_search_distance and not left_edge_found:
        await robot.straight(search_speed, 5, 0)  # Move in small increments
        distance_traveled += 5
        
        current_hsv = Left_CS.hsv()
        if current_hsv is None:
            continue
            
        current_h, current_s, current_v = current_hsv
        
        # Check if we've detected a significant color change (sample edge)
        h_diff = min(abs(current_h - initial_h), 360 - abs(current_h - initial_h))
        s_diff = abs(current_s - initial_s)
        v_diff = abs(current_v - initial_v)
        
        total_diff = h_diff + s_diff + v_diff
        
        if total_diff > threshold_difference:
            left_edge_found = True
            print(f"Left edge found at {distance_traveled}mm")
            print(f"Edge HSV: H:{current_h}, S:{current_s}, V:{current_v}")
            break
    
    if not left_edge_found:
        print("ERROR: Could not find left edge of sample")
        return False
    
    # Reset position tracking
    reset()
    left_edge_position = 0
    
    # Phase 2: Find the right edge of the sample
    print("Phase 2: Finding right edge of sample...")
    right_edge_found = False
    sample_distance = 0
    
    while sample_distance < max_search_distance and not right_edge_found:
        await robot.straight(search_speed, 5, 0)
        sample_distance += 5
        
        current_hsv = Left_CS.hsv()
        if current_hsv is None:
            continue
            
        current_h, current_s, current_v = current_hsv
        
        # Check if we've exited the sample (color change back)
        h_diff = min(abs(current_h - initial_h), 360 - abs(current_h - initial_h))
        s_diff = abs(current_s - initial_s)
        v_diff = abs(current_v - initial_v)
        
        total_diff = h_diff + s_diff + v_diff
        
        if total_diff < threshold_difference / 2:  # Less strict threshold for exit
            right_edge_found = True
            print(f"Right edge found at {sample_distance}mm from left edge")
            break
    
    if not right_edge_found:
        print("WARNING: Could not find right edge, using maximum search distance")
        sample_distance = max_search_distance
    
    # Phase 3: Calculate and move to center
    sample_width = sample_distance
    center_offset = sample_width / 2
    
    print(f"Sample width: {sample_width}mm")
    print(f"Moving back {center_offset}mm to center")
    
    # Move back to the center of the sample
    await robot.straight(-search_speed, center_offset, 0.2)
    
    # Verify we're at the center by taking a final reading
    final_hsv = Left_CS.hsv()
    if final_hsv:
        final_h, final_s, final_v = final_hsv
        print(f"Final center HSV: H:{final_h}, S:{final_s}, V:{final_v}")
        
        # Detect what color we're centered on
        centered_color = await get_color_name()
        print(f"Centered on: {centered_color}")
        hub.display.text(centered_color[0:4])
    
    print("--- Sample centering complete ---")
    return True

async def scan_for_samples(scan_distance=200, scan_speed=30):
    """
    Scan horizontally to find and identify research samples using HSV readings.
    
    PARAMETERS:
    - scan_distance: Total distance to scan in mm (default: 200)
    - scan_speed: Speed to scan at (default: 30)
    
    Returns: List of detected samples with their positions and colors
    """
    print("--- Starting sample scanning procedure ---")
    
    samples_found = []
    reset()
    
    # Get baseline reading
    baseline_hsv = Left_CS.hsv()
    if baseline_hsv is None:
        print("ERROR: Cannot establish baseline HSV")
        return samples_found
    
    baseline_h, baseline_s, baseline_v = baseline_hsv
    print(f"Baseline HSV: H:{baseline_h}, S:{baseline_s}, V:{baseline_v}")
    
    distance_scanned = 0
    in_sample = False
    sample_start_pos = 0
    
    while distance_scanned < scan_distance:
        await robot.straight(scan_speed, 10, 0)  # Move in 10mm increments
        distance_scanned += 10
        
        current_hsv = Left_CS.hsv()
        if current_hsv is None:
            continue
        
        current_h, current_s, current_v = current_hsv
        
        # Calculate difference from baseline
        h_diff = min(abs(current_h - baseline_h), 360 - abs(current_h - baseline_h))
        s_diff = abs(current_s - baseline_s)
        v_diff = abs(current_v - baseline_v)
        total_diff = h_diff + s_diff + v_diff
        
        # Check if we're entering or exiting a sample
        if not in_sample and total_diff > 20:  # Entering a sample
            in_sample = True
            sample_start_pos = distance_scanned
            print(f"Sample detected at position {distance_scanned}mm")
            
        elif in_sample and total_diff < 10:  # Exiting a sample
            in_sample = False
            sample_end_pos = distance_scanned
            sample_center = (sample_start_pos + sample_end_pos) / 2
            
            # Move back to center of the detected sample to identify it
            back_distance = distance_scanned - sample_center
            await robot.straight(-scan_speed, back_distance, 0)
            
            # Identify the sample color
            sample_color = await get_color_name()
            print(f"Sample identified: {sample_color} at center position {sample_center}mm")
            
            samples_found.append({
                'color': sample_color,
                'center_position': sample_center,
                'start_position': sample_start_pos,
                'end_position': sample_end_pos,
                'width': sample_end_pos - sample_start_pos
            })
            
            # Move back to continue scanning
            await robot.straight(scan_speed, back_distance, 0)
            distance_scanned = sample_end_pos
    
    print(f"--- Scanning complete. Found {len(samples_found)} samples ---")
    for i, sample in enumerate(samples_found):
        print(f"Sample {i+1}: {sample['color']} at {sample['center_position']}mm (width: {sample['width']}mm)")
    
    return samples_found

async def find_sample_center_reflection(search_speed=15, max_search_distance=60, threshold_difference=8):
    """
    Find the center of a research sample prop using reflection values (more reliable than HSV).
    
    PARAMETERS:
    - search_speed: Speed to move while searching (default: 15)
    - max_search_distance: Maximum distance to search in mm (default: 60) 
    - threshold_difference: Reflection difference threshold to detect sample edges (default: 8)
    
    Returns: True if successfully centered, False if no sample found
    """
    print("--- Starting sample centering using reflection values ---")
    
    # Store initial reflection to establish baseline
    try:
        initial_reflection = Left_CS.reflection()
        print(f"Initial reflection: {initial_reflection}")
    except:
        print("ERROR: Cannot read initial reflection values")
        return False
    
    # Phase 1: Find the left edge of the sample
    print("Phase 1: Finding left edge of sample...")
    left_edge_found = False
    distance_traveled = 0
    
    while distance_traveled < max_search_distance and not left_edge_found:
        await robot.straight(search_speed, 3, 0)  # Move in small increments
        distance_traveled += 3
        
        try:
            current_reflection = Left_CS.reflection()
            reflection_diff = abs(current_reflection - initial_reflection)
            
            if reflection_diff > threshold_difference:
                left_edge_found = True
                print(f"Left edge found at {distance_traveled}mm")
                print(f"Reflection changed from {initial_reflection} to {current_reflection}")
                break
        except:
            continue
    
    if not left_edge_found:
        print("ERROR: Could not find left edge of sample")
        return False
    
    # Reset position tracking
    reset()
    
    # Phase 2: Find the right edge of the sample
    print("Phase 2: Finding right edge of sample...")
    right_edge_found = False
    sample_distance = 0
    
    while sample_distance < max_search_distance and not right_edge_found:
        await robot.straight(search_speed, 3, 0)
        sample_distance += 3
        
        try:
            current_reflection = Left_CS.reflection()
            reflection_diff = abs(current_reflection - initial_reflection)
            
            if reflection_diff < threshold_difference / 2:  # Less strict threshold for exit
                right_edge_found = True
                print(f"Right edge found at {sample_distance}mm from left edge")
                break
        except:
            continue
    
    if not right_edge_found:
        print("WARNING: Could not find right edge, using maximum search distance")
        sample_distance = max_search_distance
    
    # Phase 3: Calculate and move to center
    sample_width = sample_distance
    center_offset = sample_width / 2
    
    print(f"Sample width: {sample_width}mm")
    print(f"Moving back {center_offset}mm to center")
    
    # Move back to the center of the sample
    await robot.straight(-search_speed, center_offset, 0.2)
    
    # Verify we're at the center by taking readings
    try:
        final_reflection = Left_CS.reflection()
        print(f"Final reflection at center: {final_reflection}")
        
        # Try to get HSV at center position for color identification
        final_hsv = Left_CS.hsv()
        if final_hsv:
            final_h, final_s, final_v = final_hsv
            print(f"Final center HSV: H:{final_h}, S:{final_s}, V:{final_v}")
            
            # Detect what color we're centered on
            centered_color = await get_color_name()
            print(f"Centered on: {centered_color}")
            hub.display.text(centered_color[0:4])
        
    except Exception as e:
        print(f"Error getting final readings: {e}")
    
    print("--- Sample centering complete ---")
    return True

async def demo_hsv_centering():
    """
    Simple demonstration function to test HSV-based sample centering.
    Place the robot near a research sample and run this function.
    """
    print("=== HSV SAMPLE CENTERING DEMO ===")
    print("Place robot near a research sample and press CENTER button to start")
    
    await robot.waitForBump(2)  # Wait for center button press
    
    print("Starting centering procedure...")
    success = await find_sample_center_reflection(search_speed=15, max_search_distance=60, threshold_difference=8)
    
    if success:
        print("SUCCESS: Robot centered on sample!")
        hub.light.on(Color.GREEN)
        
        # Get final color reading
        final_color = await get_color_name()
        print(f"Final detected color: {final_color}")
        
        # Play success tone
        await robot.playTone(800, 200)
        await robot.playTone(1000, 200)
        
    else:
        print("FAILED: Could not center on sample")
        hub.light.on(Color.RED)
        
        # Play failure tone
        await robot.playTone(300, 500)
    
    print("Demo complete. Press any button to exit.")
    await robot.waitForBump(2)

async def test_alignment_only():
    """
    Simple test function to test just the reflection-based alignment.
    Place robot near samples and run this to see the alignment in action.
    """
    print("=== TESTING REFLECTION ALIGNMENT ===")
    print("Place robot in front of samples and press CENTER to start")
    
    await robot.waitForBump(2)
    
    print("Testing simple alignment scan...")
    success = await align_to_highest_reflection_simple(scan_range=30, scan_speed=12, step_size=2)
    
    if success:
        print("Alignment test complete!")
        hub.light.on(Color.GREEN)
        await robot.playTone(1000, 300)
    else:
        print("Alignment test failed!")
        hub.light.on(Color.RED)
        await robot.playTone(300, 500)
    
    print("Test finished. Press CENTER to exit.")
    await robot.waitForBump(2)

# =================================================================
# =================== ROBOT CALIBRATION ROUTINE ===================
# =================================================================

async def calibrate_robot():
    """
    Comprehensive calibration routine for SPIKE template robots.
    - Calibrates wheel diameter, axle track, and gyro heading.
    - Guides user through physical measurements and updates parameters.
    - Prints recommended values for best accuracy.
    """
    print("=== ROBOT CALIBRATION ROUTINE START ===")
    print("Step 1: Wheel Diameter Calibration")
    print("Place the robot on a straight line. It will drive forward 300mm.")
    await wait(2000)
    reset()
    await robot.straight(50, 300)
    print("Measure the ACTUAL distance traveled (in mm) and enter below.")
    try:
        actual_distance = float(input("Actual distance traveled (mm): "))
    except Exception:
        print("Input not available. Please update wheel diameter manually.")
        actual_distance = 300
    expected_distance = 300
    wheel_diameter = 62.4  # Your current value
    new_wheel_diameter = wheel_diameter * (actual_distance / expected_distance)
    print(f"Recommended Wheel Diameter: {new_wheel_diameter:.2f} mm")
    print("Update your robot.Init() with this value for best straight accuracy.")

    print("\nStep 2: Axle Track Calibration")
    print("Robot will turn 360 degrees in place.")
    await wait(2000)
    reset()
    await robot.spotTurn(50, 360)
    print("Check if the robot returns to the exact starting orientation.")
    print("If it OVER-rotates, DECREASE axle track. If it UNDER-rotates, INCREASE axle track.")
    print("Enter the actual angle turned (use a protractor or mark the floor):")
    try:
        actual_angle = float(input("Actual angle turned (degrees): "))
    except Exception:
        print("Input not available. Please update axle track manually.")
        actual_angle = 360
    expected_angle = 360
    axle_track = 95  # Your current value
    new_axle_track = axle_track * (expected_angle / actual_angle)
    print(f"Recommended Axle Track: {new_axle_track:.2f} mm")
    print("Update your robot.Init() with this value for best turning accuracy.")

    print("\nStep 3: Gyro Drift Calibration")
    print("Robot will sit still for 10 seconds. Do not touch it.")
    reset()
    await wait(10000)
    drift = hub.imu.heading()
    print(f"Gyro drift after 10s: {drift:.2f} degrees")
    if abs(drift) < 1.0:
        print("Gyro drift is excellent.")
    else:
        print("If drift is high, restart the hub and recalibrate on a stable surface.")

    print("\nStep 4: Speed/Accel Tuning (Optional)")
    print("Try different values for mp.SPEED, mp.ACCEL_RATE, mp.DECEL_RATE for best results.")
    print("Test with:")
    print("  await robot.straight(50, 300)")
    print("  await robot.spotTurn(50, 90)")
    print("Adjust until you get consistent, accurate results.")

    print("\n=== CALIBRATION COMPLETE ===")
    print("Update your Init() call with the new values for best accuracy.")
    print("Example:")
    print(f"robot.Init({new_wheel_diameter:.2f}, {new_axle_track:.2f}, 6, 37, 6, 37)")

# To run calibration, just call:
# await calibrate_robot()

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
    mp.ACCEL_RATE = 30
    mp.DECEL_RATE = 30 
    
    STATION_DISTANCES = [
        55,  # To Station 2
        55,  # To Station 3
        55,  # To Station 4
        55,  # To Station 5
        55,  # To Station 6
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
    await robot.straight(30, 75)

    for box_number in range(1, 7):
        print(f"--- Reading Box #{box_number} ---")
        
        # Simple color detection using the original working method
        detected_color = await get_color_name()
        print(f"Detected Color: {detected_color}")
        
        # Display reflection value for debugging  
        try:
            current_reflection = await get_safe_reflection()
            if current_reflection is not None:
                print(f"Current sensor value: {current_reflection:.1f}")
            else:
                print("Could not read sensor value")
        except:
            print("Error reading sensor")
        
        if detected_color == "GREEN" or detected_color == "WHITE":
            hub.display.text(detected_color[0:4])
            await perform_new_green_white_action()
            await wait(100)
            
            # Move to next station and align LEFT-TO-RIGHT (for GREEN/WHITE)
            if box_number < 6:
                distance = STATION_DISTANCES[box_number - 1]
                print(f"Action: Moving to station #{box_number + 1}. Distance: {distance}mm")
                await robot.straight(30, distance)
                
                # Check if there's a block at the new position
                await wait(200)  # Let robot settle
                new_color = await get_color_name()
                print(f"New position color check: {new_color}")
                
                if new_color == "NO_BLOCK" or new_color == "Unknown":
                    print("No block detected, moving forward to find next sample...")
                    await robot.straight(30, 30)  # Move forward 30mm more
                    await wait(200)
                    new_color = await get_color_name()
                    print(f"After forward move, color: {new_color}")
                
                print("GREEN/WHITE → Aligning LEFT-TO-RIGHT for next sample...")
                await align_right_to_left_slow()

        elif detected_color == "YELLOW" or detected_color == "RED":
            hub.display.text(detected_color[0:4])
            await perform_new_yellow_red_action()
            await wait(100)
            
            # Move to next station and align RIGHT-TO-LEFT (for YELLOW/RED)
            if box_number < 6:
                distance = STATION_DISTANCES[box_number - 1]
                print(f"Action: Moving to station #{box_number + 1}. Distance: {distance}mm")
                await robot.straight(30, distance)
                
                # Check if there's a block at the new position
                await wait(200)  # Let robot settle
                new_color = await get_color_name()
                print(f"New position color check: {new_color}")
                
                if new_color == "NO_BLOCK" or new_color == "Unknown":
                    print("No block detected, moving forward to find next sample...")
                    await robot.straight(30, 30)  # Move forward 30mm more
                    await wait(200)
                    new_color = await get_color_name()
                    print(f"After forward move, color: {new_color}")
                
                print("YELLOW/RED → Aligning RIGHT-TO-LEFT for next sample...")
                await align_left_to_right_slow()
        
        else:
            print(f"Unknown color detected: {detected_color}")
            hub.display.text("????")
            await wait(500)
            
            # Still move to next station even if color unknown (use default alignment)
            if box_number < 6:
                distance = STATION_DISTANCES[box_number - 1]
                print(f"Action: Moving to station #{box_number + 1}. Distance: {distance}mm")
                await robot.straight(30, distance)
                
                # Check if there's a block at the new position
                await wait(200)  # Let robot settle
                new_color = await get_color_name()
                print(f"New position color check: {new_color}")
                
                if new_color == "NO_BLOCK" or new_color == "Unknown":
                    print("No block detected, moving forward to find next sample...")
                    await robot.straight(30, 30)  # Move forward 30mm more
                    await wait(200)
                    new_color = await get_color_name()
                    print(f"After forward move, color: {new_color}")
                
                print("Unknown color → Using default RIGHT-TO-LEFT alignment...")
                await align_left_to_right_slow()

    print("\n--- All stations checked. Performing end sequence. ---")
 
    print("--- Returning to base to drop sample. ---")
    
    print("\n--- Mission Complete ---")

async def Drone () : 
    reset()
    await robot.straight (100,200)
    await robot.spotTurn (70, 30)
    await robot.straight (100,300)


#un_task (Rover())
#run_task (Dispencer())
#run_task (Box())
#run_task (ToResearchSamples())
run_task (ResearchSamples())
#run_task (Drone())

# UNCOMMENT ONE OF THE LINES BELOW TO TEST:
#run_task(demo_hsv_centering())           # Test HSV centering (complex)
#run_task(test_alignment_only())          # Test reflection alignment (simple)

# UPDATED ResearchSamples now uses old detection + new alignment after arm actions