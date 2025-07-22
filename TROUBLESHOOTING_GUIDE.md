# 🛠️ TROUBLESHOOTING GUIDE - Super Easy Fixes! 🛠️

## 🚨 **Common Error: "drive_base not found"**

### 🔍 **What happened?**
Your robot is trying to use a part that might not be set up correctly in the SPIKE_TEMPLATE.

### ✅ **How to fix it (Super Easy!):**

#### **Step 1: Check Your SPIKE_TEMPLATE.py File**
1. Open your `SPIKE_TEMPLATE.py` file
2. Look for a line that says `drive_base = ...` 
3. If you don't see it, that's the problem!

#### **Step 2: Add the Missing DriveBase (Copy this exactly):**
Add this code to your SPIKE_TEMPLATE.py file after the motor definitions:

```python
# Create the drive base (add this if missing)
drive_base = DriveBase(left_wheel, right_wheel, wheel_diameter=62.4, axle_track=95)
```

#### **Step 3: Alternative Quick Fix**
If that doesn't work, just change this line in MM-Run.py:
```python
# Change this line at the bottom:
run_task(ResearchSamplesAdvanced())   # This uses advanced functions

# To this line instead:
run_task(Box())                       # This uses simpler functions
```

## 🎯 **Testing Your Robot (Baby Steps):**

### **Test 1: Simple Movement Test**
```python
# Replace the mission with this simple test:
run_task(test())   # Just tests basic line following
```

### **Test 2: Basic Box Mission**
```python
# Try this simpler mission first:
run_task(Box())    # Basic box handling without advanced features
```

### **Test 3: Research Mission (Advanced)**
```python
# Only try this after the simple ones work:
run_task(ResearchSamplesAdvanced())   # Full advanced features
```

## 🔧 **Step-by-Step Robot Setup (Like Building with LEGO!):**

### **Step 1: Physical Setup**
1. ✅ Robot is turned ON
2. ✅ USB cable is connected
3. ✅ All motors are plugged into the right ports:
   - Port A: Left wheel
   - Port C: Right wheel  
   - Port D: Left motor (arm)
   - Port B: Right motor (arm)
4. ✅ Color sensors are plugged in:
   - Port F: Left color sensor
   - Port E: Right color sensor (if you have two)

### **Step 2: Software Setup**
1. ✅ SPIKE_TEMPLATE.py is in the same folder
2. ✅ MM-Run.py is in the same folder
3. ✅ Both files have no red squiggly lines (errors)

### **Step 3: Running Your Code**
1. Click on MM-Run.py to select it
2. Press the green ▶️ PLAY button
3. Watch your robot wake up and be amazing!

## 🎨 **Easy Customization (Change things safely!):**

### **Make Robot Go Slower (If it's too fast):**
Find lines like this:
```python
await robot.straight(70, 200)  # 70 = speed, 200 = distance
```
Change to:
```python
await robot.straight(30, 200)  # Now it goes slower (30 instead of 70)
```

### **Make Robot Go Shorter Distances:**
```python
await robot.straight(50, 100)  # Goes 100mm instead of 200mm
```

### **Make Robot Turn Less:**
```python
await robot.spotTurn(30, 45)   # Turns 45 degrees instead of 90
```

## 🚨 **Emergency Stops:**

### **If Robot Goes Crazy:**
1. **Press the CENTER button** on the robot hub (the round button)
2. **Unplug the USB cable**
3. **Turn the robot OFF and ON again**

### **If Code Won't Run:**
1. **Save your file** (Ctrl+S)
2. **Close and reopen VS Code**
3. **Try a simpler mission first** (like `test()`)

## 🎉 **Success Checklist:**

### **Your robot should:**
- ✅ Move in straight lines without wobbling
- ✅ Turn accurately 
- ✅ Stop exactly where you want
- ✅ Print helpful messages on the screen
- ✅ Make beep sounds when successful

### **If any of these don't work:**
1. Start with the simplest test: `run_task(test())`
2. Check all your cable connections
3. Make sure your robot is on a flat, clean surface
4. Try turning the robot off and on again

## 📞 **When to Ask for Help:**

### **Ask a grown-up if:**
- Red error messages appear and won't go away
- Robot doesn't move at all
- Robot moves but goes in wrong directions
- You want to try something new but aren't sure how

## 🌟 **Remember:**
- Every great robot builder started as a beginner!
- Mistakes are how we learn
- Your robot is already super smart with these advanced features
- Have fun and don't be afraid to experiment!

## 🏆 **You Did It!**
You now have one of the smartest SPIKE Prime robots in the world using the same technology as NASA rockets and self-driving cars! Pretty awesome! 🚀🤖
