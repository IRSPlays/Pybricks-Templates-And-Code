from SPIKE_TEMPLATE import *
import umath

# Test function to check if HSV centering works
async def test_hsv_functionality():
    """Test HSV color sensor functionality and centering"""
    print("=== HSV Color Sensor Test ===")
    
    # Initialize robot (using your existing setup)
    robot.Init(62.4, 90.5, 6, 37, 5, 41)
    
    print("Testing basic HSV reading...")
    try:
        hsv_reading = Left_CS.hsv()
        if hsv_reading:
            h, s, v = hsv_reading
            print(f"HSV Reading successful: H={h}, S={s}, V={v}")
        else:
            print("HSV Reading returned None")
    except Exception as e:
        print(f"Error reading HSV: {e}")
    
    print("\nTesting reflection values for comparison...")
    try:
        reflection = Left_CS.reflection()
        print(f"Reflection value: {reflection}")
    except Exception as e:
        print(f"Error reading reflection: {e}")
    
    print("\nTesting color detection...")
    try:
        color = Left_CS.color()
        print(f"Detected color: {color}")
    except Exception as e:
        print(f"Error reading color: {e}")

async def find_sample_center_simple():
    """
    Simplified version using reflection values instead of HSV
    This uses the same principle but with reflection values which are more reliable
    """
    print("--- Starting SIMPLE sample centering procedure ---")
    
    # Get initial reflection reading
    try:
        initial_reflection = Left_CS.reflection()
        print(f"Initial reflection: {initial_reflection}")
    except:
        print("ERROR: Cannot read initial reflection value")
        return False
    
    # Phase 1: Find left edge by looking for significant change
    print("Phase 1: Finding left edge...")
    left_edge_found = False
    distance_traveled = 0
    max_search = 80
    
    while distance_traveled < max_search and not left_edge_found:
        await robot.straight(20, 5, 0)  # Move 5mm at a time
        distance_traveled += 5
        
        try:
            current_reflection = Left_CS.reflection()
            reflection_diff = abs(current_reflection - initial_reflection)
            
            if reflection_diff > 10:  # Significant change detected
                left_edge_found = True
                print(f"Left edge found at {distance_traveled}mm")
                print(f"Reflection changed from {initial_reflection} to {current_reflection}")
                break
        except:
            continue
    
    if not left_edge_found:
        print("Could not find left edge")
        return False
    
    # Reset and track sample width
    reset()
    
    # Phase 2: Find right edge
    print("Phase 2: Finding right edge...")
    right_edge_found = False
    sample_width = 0
    
    while sample_width < max_search and not right_edge_found:
        await robot.straight(20, 5, 0)
        sample_width += 5
        
        try:
            current_reflection = Left_CS.reflection()
            reflection_diff = abs(current_reflection - initial_reflection)
            
            if reflection_diff < 5:  # Back to similar values
                right_edge_found = True
                print(f"Right edge found at {sample_width}mm from left edge")
                break
        except:
            continue
    
    if not right_edge_found:
        print("Using maximum search distance for sample width")
        sample_width = max_search
    
    # Phase 3: Move to center
    center_distance = sample_width / 2
    print(f"Sample width: {sample_width}mm, moving back {center_distance}mm to center")
    
    await robot.straight(-20, center_distance, 0.2)
    
    # Final reading
    try:
        final_reflection = Left_CS.reflection()
        print(f"Final reflection at center: {final_reflection}")
        
        # Try to get HSV at center position
        hsv_at_center = Left_CS.hsv()
        if hsv_at_center:
            h, s, v = hsv_at_center
            print(f"HSV at center: H={h}, S={s}, V={v}")
        
    except Exception as e:
        print(f"Error getting final readings: {e}")
    
    print("--- Centering complete ---")
    return True

# Test the centering without using the complex ResearchSamples function
async def test_centering():
    """Simple test of the centering functionality"""
    print("=== Testing Sample Centering ===")
    
    # First test basic sensor functionality
    await test_hsv_functionality()
    
    print("\nStarting centering test...")
    print("Place the robot near a research sample and press CENTER button to start")
    
    await robot.waitForBump(2)  # Wait for center button
    
    # Try the simple centering approach
    success = await find_sample_center_simple()
    
    if success:
        print("Centering successful!")
        hub.light.on(Color.GREEN)
    else:
        print("Centering failed!")
        hub.light.on(Color.RED)

# Main program - uncomment to test
# run_task(test_centering())
