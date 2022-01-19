This season we will have a chassis bot to program while the team is building the competition bot.  We're asking the mechies to do a few things to the chassis bot:

- Fasten everything down so we can drive it fast without breaking it.
- Add ballast so its mass approximates the competition bot's.
- Add lights so we can tell when we've issued commands (getting ready to control subsystems like the shooter and climber on the competition bot).
- Add a selector so we can let the robot know which of four positions it is starting from.

# Milestone 1: Joystick Control

The swerve-base code assumes a game pad controlller.  We want to use two joysticks, one for translation and the other for rotation.

Once we can drive the chassis bot with two joysticks, our drive team can practice driving.  Also, we can log performance of critical sub-systems like odometry.

# Milestone 2: Zero-Ball Taxi Auto

The first 15 seconds of the game are completely autonomous.  Moving our robot out of the tarmac scores two points during auto.  Once we can run our auto from any of four starting positions, we can test more complex autos.

Odometry should be good enough for this auto.

# Milestone 3: Move to All the Balls Auto

A key capability for a high-scoring auto is moving to three positions to pick up stationary balls.  Running this auto will show how consistently and how far we can move the robot in 15 seconds, which will help us decide how many balls to try to score during auto.

Odometry may or may not be good enough for this auto.  If odometry has too much error, we will try vision-based localization in this order:

- Limelight (keep hub target in view)
- V-SLAM
- neural net object detection

We will also see if we can minimize or correct odometry error based on an analysis of logs showing what throws off our odometry.

# Milestone 4: Light Up One-Ball Auto Movement

Use the lights on the chassis bot to show when we've issued our shoot command.  When the competition bot is ready, we can swap out the call so that our code calls the real sub-system command instead of turning on a light.

# Milestone 5: Two-Ball Auto Movement

Once we have the lights working, we can use them to flag when we will be issuing commands to subsystems during our auto routines.  The two-ball auto will require shooting, moving, intaking, indexing and shooting again.  For this milestone we use lights to indicate shooting, intaking and indexing.  We're dialing in the robot movement, and the sequencing and timing of issuing commands to the subsystems.

# Milestone 6: Three-Ball Auto Movement

Three-ball auto requires picking up two balls.  We need to decide whether to go for the wall ball and the field ball or the field ball and the terminal ball.  We may decide to write two different three-ball autos and be able to switch between them.

# Milestone 7: Four-Ball Auto Movement

Depending on what we find during milestone 3, we may decide to go for a four-ball auto or we may decide the prospect is too unlikely to attempt.  A few things to help us decide whether to try:

- Can we move consistently and quickly enough to pick up three balls in 15 seconds?
- Is our intake fast enough to pick up a stationary ground ball in one second once we arrive at the ball's location?
- Can we aim the shooting mechanism or is it a fixed trajectory?
- What are the chances we'll hit another ball that is near or in the upper goal of the hub?

# Milestone 8: Shoot a Ball

Once the mechies have built the shooting mechanism, we'll know whether it can be aimed or is a fixed trajectory.  We'll start by not worrying about adjusting the aim, just use a fixed trajectory and make sure we can consistently launch a ball toward the goal.  Key data to collect while testing are

 - consistency
 - reliability
 - time required to shoot
 - time required to reset and reload

# Milestone 9: Aim the Shooter

If the mech team designs and builds an adjustable shooter, we can aim it at the goal using

- Odometry (better than nothing)
- Limelight
- Another vision system

The goal is to be able to aim and shoot accurately while the robot is moving.

# Milestone 10: Intake Stationary Ground Balls

Get the intake mechanism working.  Measure how quickly we can get a stationary ball off the ground and move to a new position while indexing the ball.

# Milestone 11: Intake Bouncing Balls

If there are any special controls for intaking bouncing balls, get them working.

# Milestone 12: One Ball Auto for Real

Shoot a ball and move out of the tarmac.

# Milestone 13: Two-Ball Auto for Real

- Shoot a ball.
- Move out of the tarmac to the position of a stationary ground ball.
- Intake the ball.
- Index the ball.
- Shoot the ball.

# Milestone 14: Three-Ball Auto for Real

Shoot a ball, then go pick up and shoot two more balls, all within 15 seconds.

# Milestone 15: Four-Ball Auto for Real

Shoot a ball, then go pick up and shoot three more balls, all within 15 seconds.

# Milestone 16: Climb on One Bar

Get the climbing mechanism to work on one bar.

# Milestone 17: Climb and Move to Next Bar

Climb and then move to the next bar in the hangar.

# Milestone 18: Climb and Traverse

Climb and then move to the traversal bar.  This could be a manual or automatic process.

# Milestone 19: Automatic Move to Hangar

Push a button and have the robot automatically drive itself to the hangar to start climbing.

# Milestone 20: Target Lock on Balls

Allow robot to recognize balls, lock on target, automatically move to the ball for intake.

# Milestone 21: Dodge Other Robots

Allow robot to recognize other robots and dodge them while moving or aiming.

# Milestone 22: Five-Ball Auto

Just to prove it can be done.