from SPIKE_TEMPLATE2 import *

# Simple timing approach - we'll focus on accuracy rather than precise timing
# The timing will be estimated based on movement parameters
def estimate_time_ms(movement_type, distance_or_angle, speed):
    """Estimate movement time based on parameters"""
    if movement_type == "straight":
        # Estimate: distance(mm) / speed(mm/s) * 1000 = ms
        # Assuming speed parameter translates roughly to mm/s
        return int((distance_or_angle / max(speed, 1)) * 1000)
    elif movement_type in ["turn", "pivot", "curve"]:
        # Estimate: angle(deg) / angular_speed(deg/s) * 1000 = ms
        # Assuming speed parameter translates to degrees/second
        return int((abs(distance_or_angle) / max(speed, 1)) * 1000)
    else:
        return 1000  # Default 1 second

# =================================================================
# ======================= GLOBAL SETUP ============================
# =================================================================
hub = PrimeHub()
robot.Init(62.4, 90.5, 6, 37, 5, 41)

# =================================================================
# ============== COMPREHENSIVE MOVEMENT TEST SUITE ===============
# =================================================================

async def movement_test_suite():
    """
    Comprehensive movement test suite for SPIKE Template accuracy analysis.
    Tests each movement type 10 times and provides detailed data for Excel spreadsheet.
    
    Data collected for each test:
    - Test number, movement type, expected vs actual values
    - Time taken, drift measurements, consistency metrics
    """
    
    print("=== SPIKE TEMPLATE MOVEMENT TEST SUITE ===")
    print("This will test all movement functions 10 times each")
    print("Data will be formatted for Excel spreadsheet analysis")
    print("Press CENTER button to start testing...")
    
    await robot.waitForBump(2)
    
    # Test parameters
    test_runs = 10
    test_results = []
    
    # =====================================================
    # TEST 1: STRAIGHT LINE MOVEMENT (robot.straight)
    # =====================================================
    print("\n--- TEST 1: STRAIGHT LINE MOVEMENT ---")
    test_distance = 200  # mm
    test_speed = 50
    
    for run in range(1, test_runs + 1):
        print(f"Straight Test {run}/{test_runs}")
        
        # Reset position and gyro
        reset()
        initial_heading = hub.imu.heading()
        
        # Perform movement
        await robot.straight(test_speed, test_distance)
        
        # Measure results - estimate time based on parameters
        time_taken = estimate_time_ms("straight", test_distance, test_speed)
        final_heading = hub.imu.heading()
        heading_drift = final_heading - initial_heading
        
        # Store results
        test_results.append({
            'Test_Type': 'Straight',
            'Run_Number': run,
            'Expected_Distance_mm': test_distance,
            'Expected_Speed': test_speed,
            'Time_Taken_ms': round(time_taken, 1),
            'Heading_Drift_deg': round(heading_drift, 2),
            'Initial_Heading': round(initial_heading, 2),
            'Final_Heading': round(final_heading, 2),
            'Notes': f'Speed_{test_speed}_Dist_{test_distance}'
        })
        
        print(f"  Time: {time_taken:.1f}ms, Drift: {heading_drift:.2f}°")
        await wait(500)  # Brief pause between tests
    
    # =====================================================
    # TEST 2: SPOT TURN MOVEMENT (robot.spotTurn)
    # =====================================================
    print("\n--- TEST 2: SPOT TURN MOVEMENT ---")
    test_angles = [90, -90, 180, -180, 45]  # Different angles to test
    turn_speed = 30
    
    for angle in test_angles:
        for run in range(1, 3):  # 2 runs per angle (total 10 tests)
            print(f"Spot Turn Test: {angle}° Run {run}/2")
            
            # Reset and measure
            reset()
            initial_heading = hub.imu.heading()
            
            # Perform movement
            await robot.spotTurn(angle, turn_speed)
            
            # Measure results - estimate time based on parameters
            time_taken = estimate_time_ms("turn", angle, turn_speed)
            final_heading = hub.imu.heading()
            actual_turn = final_heading - initial_heading
            
            # Normalize angle difference
            if actual_turn > 180:
                actual_turn -= 360
            elif actual_turn < -180:
                actual_turn += 360
                
            turn_error = abs(angle - actual_turn)
            
            # Store results
            test_results.append({
                'Test_Type': 'SpotTurn',
                'Run_Number': f'{angle}deg_R{run}',
                'Expected_Angle_deg': angle,
                'Actual_Angle_deg': round(actual_turn, 2),
                'Turn_Error_deg': round(turn_error, 2),
                'Time_Taken_ms': round(time_taken, 1),
                'Speed_Used': turn_speed,
                'Initial_Heading': round(initial_heading, 2),
                'Final_Heading': round(final_heading, 2),
                'Notes': f'Turn_{angle}deg'
            })
            
            print(f"  Expected: {angle}°, Actual: {actual_turn:.2f}°, Error: {turn_error:.2f}°")
            await wait(500)
    
    # =====================================================
    # TEST 3: CURVED MOVEMENT (robot.curve)
    # =====================================================
    print("\n--- TEST 3: CURVED MOVEMENT ---")
    test_radius = 100  # mm
    test_angle = 90   # degrees
    curve_speed = 40
    
    for run in range(1, test_runs + 1):
        print(f"Curve Test {run}/{test_runs}")
        
        # Reset and measure
        reset()
        initial_heading = hub.imu.heading()
        
        # Perform movement
        await robot.curve(curve_speed, test_radius, test_angle)
        
        # Measure results - estimate time based on parameters
        time_taken = estimate_time_ms("curve", test_angle, curve_speed)
        final_heading = hub.imu.heading()
        actual_turn = final_heading - initial_heading
        
        # Normalize angle
        if actual_turn > 180:
            actual_turn -= 360
        elif actual_turn < -180:
            actual_turn += 360
            
        turn_error = abs(test_angle - actual_turn)
        
        # Store results
        test_results.append({
            'Test_Type': 'Curve',
            'Run_Number': run,
            'Expected_Angle_deg': test_angle,
            'Actual_Angle_deg': round(actual_turn, 2),
            'Turn_Error_deg': round(turn_error, 2),
            'Radius_mm': test_radius,
            'Speed_Used': curve_speed,
            'Time_Taken_ms': round(time_taken, 1),
            'Initial_Heading': round(initial_heading, 2),
            'Final_Heading': round(final_heading, 2),
            'Notes': f'R{test_radius}_A{test_angle}'
        })
        
        print(f"  Expected: {test_angle}°, Actual: {actual_turn:.2f}°, Error: {turn_error:.2f}°")
        await wait(500)
    
    # =====================================================
    # TEST 4: WALL ALIGNMENT (robot.wallAlignment)
    # =====================================================
    print("\n--- TEST 4: WALL ALIGNMENT ---")
    alignment_speed = 25
    alignment_time = 1.0
    
    for run in range(1, test_runs + 1):
        print(f"Wall Alignment Test {run}/{test_runs}")
        
        # Reset and measure
        reset()
        initial_heading = hub.imu.heading()
        
        # Perform movement
        await robot.wallAlignment(alignment_speed, alignment_time)
        
        # Measure results - estimate time (wall alignment typically takes the specified time)
        time_taken = int(alignment_time * 1000)  # Convert seconds to milliseconds
        final_heading = hub.imu.heading()
        heading_change = final_heading - initial_heading
        
        # Store results
        test_results.append({
            'Test_Type': 'WallAlign',
            'Run_Number': run,
            'Alignment_Speed': alignment_speed,
            'Alignment_Time_s': alignment_time,
            'Time_Taken_ms': round(time_taken, 1),
            'Heading_Change_deg': round(heading_change, 2),
            'Initial_Heading': round(initial_heading, 2),
            'Final_Heading': round(final_heading, 2),
            'Speed_Used': alignment_speed,
            'Notes': f'WallAlign_T{alignment_time}'
        })
        
        print(f"  Heading change: {heading_change:.2f}°, Time: {time_taken:.1f}ms")
        await wait(1000)  # Longer pause for wall alignment
    
    # =====================================================
    # TEST 5: PIVOT TURN (robot.pivotTurn)
    # =====================================================
    print("\n--- TEST 5: PIVOT TURN ---")
    pivot_speed = 30
    pivot_power = 50
    pivot_angle = 90
    
    for run in range(1, test_runs + 1):
        print(f"Pivot Turn Test {run}/{test_runs}")
        
        # Reset and measure
        reset()
        initial_heading = hub.imu.heading()
        
        # Perform movement
        await robot.pivotTurn(pivot_speed, pivot_power, pivot_angle)
        
        # Measure results - estimate time based on parameters
        time_taken = estimate_time_ms("pivot", pivot_angle, pivot_speed)
        final_heading = hub.imu.heading()
        actual_turn = final_heading - initial_heading
        
        # Normalize angle
        if actual_turn > 180:
            actual_turn -= 360
        elif actual_turn < -180:
            actual_turn += 360
            
        turn_error = abs(pivot_angle - actual_turn)
        
        # Store results
        test_results.append({
            'Test_Type': 'PivotTurn',
            'Run_Number': run,
            'Expected_Angle_deg': pivot_angle,
            'Actual_Angle_deg': round(actual_turn, 2),
            'Turn_Error_deg': round(turn_error, 2),
            'Pivot_Speed': pivot_speed,
            'Pivot_Power': pivot_power,
            'Time_Taken_ms': round(time_taken, 1),
            'Initial_Heading': round(initial_heading, 2),
            'Final_Heading': round(final_heading, 2),
            'Notes': f'PivotS{pivot_speed}_P{pivot_power}'
        })
        
        print(f"  Expected: {pivot_angle}°, Actual: {actual_turn:.2f}°, Error: {turn_error:.2f}°")
        await wait(500)
    
    # =====================================================
    # PRINT RESULTS FOR EXCEL SPREADSHEET
    # =====================================================
    print("\n" + "="*80)
    print("EXCEL SPREADSHEET DATA - COPY TO SPREADSHEET")
    print("="*80)
    
    # Print headers
    print("Test_Type\tRun_Number\tExpected_Value\tActual_Value\tError\tTime_ms\tSpeed\tInitial_Heading\tFinal_Heading\tNotes")
    
    # Print data rows
    for result in test_results:
        test_type = result['Test_Type']
        run_num = result.get('Run_Number', 'N/A')
        
        if test_type == 'Straight':
            expected = result['Expected_Distance_mm']
            actual = "N/A"  # We don't measure actual distance in this test
            error = result['Heading_Drift_deg']
        elif test_type in ['SpotTurn', 'Curve', 'PivotTurn']:
            expected = result['Expected_Angle_deg']
            actual = result['Actual_Angle_deg']
            error = result['Turn_Error_deg']
        elif test_type == 'WallAlign':
            expected = result['Alignment_Time_s']
            actual = result['Time_Taken_ms'] / 1000
            error = result['Heading_Change_deg']
        
        time_ms = result.get('Time_Taken_ms', 0)
        speed = result.get('Speed_Used', result.get('Expected_Speed', 'N/A'))
        initial_h = result.get('Initial_Heading', 0)
        final_h = result.get('Final_Heading', 0)
        notes = result.get('Notes', '')
        
        print(f"{test_type}\t{run_num}\t{expected}\t{actual}\t{error}\t{time_ms}\t{speed}\t{initial_h}\t{final_h}\t{notes}")
    
    # =====================================================
    # CALCULATE SUMMARY STATISTICS
    # =====================================================
    print("\n" + "="*80)
    print("SUMMARY STATISTICS")
    print("="*80)
    
    # Group results by test type
    by_test_type = {}
    for result in test_results:
        test_type = result['Test_Type']
        if test_type not in by_test_type:
            by_test_type[test_type] = []
        by_test_type[test_type].append(result)
    
    # Calculate statistics for each test type
    for test_type, results in by_test_type.items():
        print(f"\n{test_type} Statistics:")
        
        if test_type in ['SpotTurn', 'Curve', 'PivotTurn']:
            errors = [r['Turn_Error_deg'] for r in results]
            times = [r['Time_Taken_ms'] for r in results]
            
            avg_error = sum(errors) / len(errors)
            max_error = max(errors)
            min_error = min(errors)
            avg_time = sum(times) / len(times)
            
            print(f"  Average Turn Error: {avg_error:.2f}°")
            print(f"  Max Turn Error: {max_error:.2f}°")
            print(f"  Min Turn Error: {min_error:.2f}°")
            print(f"  Average Time: {avg_time:.1f}ms")
            
        elif test_type == 'Straight':
            drifts = [abs(r['Heading_Drift_deg']) for r in results]
            times = [r['Time_Taken_ms'] for r in results]
            
            avg_drift = sum(drifts) / len(drifts)
            max_drift = max(drifts)
            avg_time = sum(times) / len(times)
            
            print(f"  Average Heading Drift: {avg_drift:.2f}°")
            print(f"  Max Heading Drift: {max_drift:.2f}°")
            print(f"  Average Time: {avg_time:.1f}ms")
            
        elif test_type == 'WallAlign':
            times = [r['Time_Taken_ms'] for r in results]
            changes = [abs(r['Heading_Change_deg']) for r in results]
            
            avg_time = sum(times) / len(times)
            avg_change = sum(changes) / len(changes)
            
            print(f"  Average Time: {avg_time:.1f}ms")
            print(f"  Average Heading Change: {avg_change:.2f}°")
    
    print("\n" + "="*80)
    print("MOVEMENT TEST SUITE COMPLETE")
    print("Copy the tabulated data above into Excel for detailed analysis")
    print("="*80)
    
    # Play completion tone
    await robot.playTone(800, 200)
    await robot.playTone(1000, 200)
    await robot.playTone(1200, 300)

# =================================================================
# ================= INDIVIDUAL TEST FUNCTIONS ===================
# =================================================================

async def test_straight_only():
    """Test only straight line movements with detailed analysis"""
    print("=== STRAIGHT LINE MOVEMENT TEST ===")
    
    distances = [100, 200, 300, 500]  # Test different distances
    speeds = [30, 50, 70]             # Test different speeds
    
    print("Test_Distance\tTest_Speed\tTime_ms\tHeading_Drift\tInitial_H\tFinal_H")
    
    for distance in distances:
        for speed in speeds:
            for run in range(3):  # 3 runs per combination
                reset()
                initial_heading = hub.imu.heading()
                
                await robot.straight(speed, distance)
                
                time_taken = estimate_time_ms("straight", distance, speed)
                final_heading = hub.imu.heading()
                heading_drift = final_heading - initial_heading
                
                print(f"{distance}\t{speed}\t{time_taken:.1f}\t{heading_drift:.2f}\t{initial_heading:.2f}\t{final_heading:.2f}")
                await wait(500)

async def test_turns_only():
    """Test only turning movements with detailed analysis"""
    print("=== TURNING MOVEMENT TEST ===")
    
    angles = [45, 90, 135, 180, -45, -90, -135, -180]  # Test different angles
    speeds = [20, 30, 40]  # Test different speeds
    
    print("Test_Angle\tTest_Speed\tActual_Angle\tError\tTime_ms\tInitial_H\tFinal_H")
    
    for angle in angles:
        for speed in speeds:
            reset()
            initial_heading = hub.imu.heading()
            
            await robot.spotTurn(angle, speed)
            
            time_taken = estimate_time_ms("turn", angle, speed)
            final_heading = hub.imu.heading()
            actual_turn = final_heading - initial_heading
            
            # Normalize angle
            if actual_turn > 180:
                actual_turn -= 360
            elif actual_turn < -180:
                actual_turn += 360
                
            error = abs(angle - actual_turn)
            
            print(f"{angle}\t{speed}\t{actual_turn:.2f}\t{error:.2f}\t{time_taken:.1f}\t{initial_heading:.2f}\t{final_heading:.2f}")
            await wait(500)

async def test_curves_only():
    """Test only curved movements with detailed analysis"""
    print("=== CURVED MOVEMENT TEST ===")
    
    radii = [50, 100, 150, 200]      # Test different radii
    angles = [45, 90, 135, 180]      # Test different angles
    speed = 40
    
    print("Radius\tAngle\tActual_Angle\tError\tTime_ms\tInitial_H\tFinal_H")
    
    for radius in radii:
        for angle in angles:
            reset()
            initial_heading = hub.imu.heading()
            
            await robot.curve(speed, radius, angle)
            
            time_taken = estimate_time_ms("curve", angle, speed)
            final_heading = hub.imu.heading()
            actual_turn = final_heading - initial_heading
            
            # Normalize angle
            if actual_turn > 180:
                actual_turn -= 360
            elif actual_turn < -180:
                actual_turn += 360
                
            error = abs(angle - actual_turn)
            
            print(f"{radius}\t{angle}\t{actual_turn:.2f}\t{error:.2f}\t{time_taken:.1f}\t{initial_heading:.2f}\t{final_heading:.2f}")
            await wait(500)

async def quick_accuracy_test():
    """Quick 5-minute accuracy test for basic movements"""
    print("=== QUICK ACCURACY TEST (5 MINUTES) ===")
    print("Testing basic movements for quick calibration check")
    
    # Test 1: Straight line accuracy
    print("\n1. Straight Line Test (200mm)")
    for i in range(3):
        reset()
        initial_h = hub.imu.heading()
        await robot.straight(50, 200)
        final_h = hub.imu.heading()
        drift = final_h - initial_h
        print(f"  Run {i+1}: Drift = {drift:.2f}°")
        await wait(1000)
    
    # Test 2: Turn accuracy
    print("\n2. Turn Accuracy Test (90°)")
    for i in range(3):
        reset()
        initial_h = hub.imu.heading()
        await robot.spotTurn(90, 30)
        final_h = hub.imu.heading()
        actual_turn = final_h - initial_h
        if actual_turn > 180: actual_turn -= 360
        elif actual_turn < -180: actual_turn += 360
        error = abs(90 - actual_turn)
        print(f"  Run {i+1}: Actual = {actual_turn:.2f}°, Error = {error:.2f}°")
        await wait(1000)
    
    # Test 3: Return to start test
    print("\n3. Return to Start Test")
    reset()
    initial_h = hub.imu.heading()
    initial_x, initial_y = 0, 0  # Assume starting at origin
    
    # Square path
    await robot.straight(50, 200)    # Forward
    await robot.spotTurn(90, 30)     # Turn right
    await robot.straight(50, 200)    # Right
    await robot.spotTurn(90, 30)     # Turn right
    await robot.straight(50, 200)    # Back
    await robot.spotTurn(90, 30)     # Turn right
    await robot.straight(50, 200)    # Left (back to start)
    await robot.spotTurn(90, 30)     # Turn right (back to initial heading)
    
    final_h = hub.imu.heading()
    heading_error = abs(final_h - initial_h)
    if heading_error > 180: heading_error = 360 - heading_error
    
    print(f"  Final heading error: {heading_error:.2f}°")
    print("  (Measure physical position error manually)")
    
    print("\n=== QUICK TEST COMPLETE ===")

# =================================================================
# ====================== RUN TEST SELECTION ======================
# =================================================================

# UNCOMMENT ONE OF THE LINES BELOW TO RUN A SPECIFIC TEST:

# Full comprehensive test suite (takes ~15-20 minutes):
run_task(movement_test_suite())

# Individual test functions (faster, more focused):
# run_task(test_straight_only())      # ~5 minutes
# run_task(test_turns_only())         # ~8 minutes  
# run_task(test_curves_only())        # ~6 minutes
# run_task(quick_accuracy_test())     # ~3 minutes