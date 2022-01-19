Automation is similar to autonomous operation, but occurs during the teleoperated period. The goals of automation are to

1. reduce the load on the human driver and operator,
2. improve robot capabilities, and
3. score more points during a match.

# Moving the Robot

Our differential swerve drive (DSD) drive train allows us to rotate and translate the robot at the same time. We automate controls so that the driver controls translation and operator controls rotation. We also smooth the inputs so the robot accelerates and decelerates smoothly.

Other ideas for automation while moving the robot:

- slow down the robot if we're about to drive off the field so we don't slam into walls
- have waypoints that we can send the robot to by pressing a button (such as moving to the hangar for the endgame)
- dodging other robots
- tractor beam to pick up balls (i.e. acquire target lock on ball and move to it)

# Aiming and Shooting

During auto we need to aim and shoot. During teleop, automated aiming will help us score more points. One idea is to have the robot automatically line up on the target when the operator presses the shoot button and shoot when the operator releases the shoot button.

# Climbing

This year's climbing could benefit from automation. One idea is to have the operator initiate the climb sequence, and then have the robot automatically climb and move from bar to bar.