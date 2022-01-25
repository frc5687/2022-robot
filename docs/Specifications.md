Speficiations:
Robot actions are initiated by operator input ei. button press.
# Robot Specific:
  ## Movement
  
    - Translate left, right, up, down.
    - Rotate.
    
  ## Shoot
  Ideal shoot anywhere on field.
  
    - Check if arm is lowered & if not lowered, lower arm.
    - Check if cargo is loaded & check cargo color.
    - If emply, load cargo & check cargo color.
    
    - If wrong cargo color, set springs to low strength and eject cargo.
    
    - Look for target using vision system.
    - Centered shooter with target.
    - Determine distance to target.
    - Adjust spring testion to achieve correct parabola.
    - Realese pin, to fire cargo.
    - Pull arm down for reloading.
    
  ## Intaking
  
    - Put intake out / release intake
    - Intake balls by rotating rollers.
    - Pull intake in / retract intake.  
  ## Information lights
    - Climbing = Fun patteren.  
    - Shooting fail = red flashing (we didn't shoot the ball).
    - Shooting success = Yellow flashing light(we shot the ball).
    - Intake = purple parterns. 
    - If centered with target = green flashing.
  
    
  ## Climb
  Start climbing at 45 seconds or more.
  
    - Drive to hanger.
    - Set up under mid bar.
    - Raise left arm.
    - Raise off of ground.
    - Raise right arm and grab high bar.
    - Pivior center of mass to under high bar.
    - Release left arm and retract.
    - Raise robot higher.
    - Piviot left arm for high run.
    - Extend left arm
    - Grab traversal rung with left arm
    - Piviot center of mass under traversal rung.
    - Release right arm.
  
## Will Do / Can Do Table

As we write our specification and code we will fill out this table.

Category | Specification | Will Do | Can Do
---------|---------------|---------|-------
Move | Move responding to remote human manual control | YES | YES
Shoot | Shoot cargo into high goal | YES | NO
Fly | Levitate in place | NO | NO