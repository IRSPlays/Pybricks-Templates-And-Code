# Enhanced Movement Examples
# This file demonstrates how to use the new improved movement functions

"""
Example 1: Basic enhanced straight movement with validation
"""
async def example_enhanced_straight():
    # Move 500mm forward with automatic validation and recovery
    await robot.straightEnhanced(50, 500)
    
    # Move 200mm backward with validation disabled for speed
    await robot.straightEnhanced(-40, 200, VALIDATE=False)

"""
Example 2: Ultra-smooth movements for delicate operations
"""
async def example_ultra_smooth():
    # Set ultra-smooth motion profile
    mp.setMotionProfile('ultra_smooth')
    
    # Movements will now be extremely smooth and precise
    await robot.straight(30, 300)  # Uses ultra-smooth automatically
    await robot.spotTurn(25, 90)   # Ultra-smooth turning

"""
Example 3: Enhanced spot turn with precision validation
"""
async def example_enhanced_turn():
    # Turn 90 degrees with automatic validation
    await robot.spotTurnEnhanced(50, 90)
    
    # Turn with recovery disabled for faster operation
    await robot.spotTurnEnhanced(60, -45, RECOVER=False)

"""
Example 4: Different motion profiles for different scenarios
"""
async def example_motion_profiles():
    # For speed
    mp.setMotionProfile('speed')
    await robot.straight(80, 1000)
    
    # For precision tasks
    mp.setMotionProfile('precision')
    await robot.straight(20, 50)
    
    # For anti-tilt on rough surfaces
    mp.setMotionProfile('anti_tilt')
    await robot.straight(40, 500)

"""
Example 5: Manual validation for custom requirements
"""
async def example_manual_validation():
    # Perform movement
    await robot.straight(50, 400)
    
    # Manually validate with custom tolerance
    success, dist_error, angle_error = robot.validateMovement(
        expected_distance=400, 
        tolerance_mm=1  # Very strict tolerance
    )
    
    if not success:
        print("Movement failed validation!")
        # Perform custom recovery
        await robot.recoverMovement(dist_error, angle_error, max_speed=15)