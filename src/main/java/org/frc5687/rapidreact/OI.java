/** Team 5687 (C)2021-2022
 * Joystick and gamepad control for the robot.
 * Also has button inits.
 * Has some instructions on how to switch controls.
*/
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.util.Helpers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.frc5687.rapidreact.commands.AutoIntake;
import org.frc5687.rapidreact.commands.Climber.AutoClimb;
import org.frc5687.rapidreact.commands.Climber.Stow;
import org.frc5687.rapidreact.commands.catapult.SetSetpoint;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.util.AxisButton;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;

public class OI extends OutliersProxy {

    // Joysticks and gamepads
    private Gamepad _driverGamepad;
    private Gamepad _operatorGamepad;
    private Gamepad _debug;

    // Buttons
    private JoystickButton _autoAim;
    private JoystickButton _catapultDebugButton;
    private JoystickButton _deployRetract;
    private JoystickButton _dropArm;
    private JoystickButton _exitKill;
    private AxisButton _intakeButton;
    private JoystickButton _kill;
    private JoystickButton _preloadButton;
    private JoystickButton _readyToClimb;
    private JoystickButton _stowClimber;
    private JoystickButton _release;
    private JoystickButton _resetNavX;
    private AxisButton _shootButton;

//    private JoystickButton _shootSetpointOne;
//    private JoystickButton _shootSetpointTwo;
//    private JoystickButton _shootSetpointThree;

    // "Raw" joystick values
    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _debug = new Gamepad(2);

        // debug gamepad
        _catapultDebugButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.A.getNumber());
        _preloadButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.B.getNumber());
//        _release = new JoystickButton(_debug, Gamepad.Buttons.X.getNumber());
        _readyToClimb = new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());
        _stowClimber = new JoystickButton(_driverGamepad, Gamepad.Buttons.Y.getNumber());

        // adding buttons while driving: Ben pls look

//        _shootButton = new JoystickButton(_debug, Gamepad.Buttons.Y.getNumber());

        // rotation joystickj
        _intakeButton = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), 0.2);
        _autoAim = new JoystickButton(_operatorGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());

        // translation joystick
        _shootButton= new AxisButton (_operatorGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.2);
        _release = new JoystickButton(_driverGamepad, Gamepad.Buttons.X.getNumber());
        _resetNavX = new JoystickButton(_driverGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());

        // while driving, ben check.
//        _shootSetpointOne = new JoystickButton(_translation, 9);
//        _shootSetpointTwo = new JoystickButton(_translation, 10);
//        _shootSetpointThree = new JoystickButton(_translation, 11);

//        _exitKill = new JoystickButton(_translation, 6);
//        _kill = new JoystickButton(_translation, 7);
        
    }

    public void initializeButtons(DriveTrain driveTrain, Catapult catapult, Intake intake, Climber climber) {
        // driving, Ben check pls.
//        _shootSetpointOne.whenPressed(new SetSetpoint(catapult, Catapult.CatapultSetpoint.FAR));
//        _shootSetpointTwo.whenPressed(new SetSetpoint(catapult, Catapult.CatapultSetpoint.MID));
//        _shootSetpointThree.whenPressed(new SetSetpoint(catapult, Catapult.CatapultSetpoint.NEAR));

        _intakeButton.whenHeld(new AutoIntake(intake));
        _resetNavX.whenPressed(driveTrain::resetYaw);
        _readyToClimb.whenPressed(new AutoClimb(climber));
        _stowClimber.whenPressed(new Stow(climber));
    }

    public boolean readyToClimb() { return _readyToClimb.get(); }
    public boolean isShootButtonPressed() { return _shootButton.get(); }
    public boolean exitDebugCatapult() { return _catapultDebugButton.get(); }
    public boolean preloadCatapult() { return _preloadButton.get(); }
    public boolean releaseArm() { return _release.get(); }
    public boolean exitKill() { return _exitKill.get(); }
    public boolean kill() { return _kill.get(); }
    public boolean autoAim() { return _autoAim.get(); }

    public double getDriveY() {
        //Comment for gamepad control
        yIn = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        // yIn = getSpeedFromAxis(Gamepad, Gamepad.getYChannel());
        yIn = applyDeadband(yIn, Constants.DriveTrain.DEADBAND);
        metric("yIn", yIn);

        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        yOut = (yOut + (yIn * 2)) / 3.0;
        return yOut;
    }

    public double getDriveX() {
        //Comment for gamepad control
        xIn = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        //xIn = -getSpeedFromAxis(Gamepad, Gamepad.getXChannel());
        xIn = applyDeadband(xIn, Constants.DriveTrain.DEADBAND);
        metric("xIn", xIn);
        double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        xOut = (xOut + (xIn * 2)) / 3.0; // numbers from empirical testing.
        return xOut;
    }

    public double getRotationX() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, Constants.DEADBAND);
        return speed;
    }

    public double getSpringMotorSpeed() {
        double speed = -getSpeedFromAxis(_debug, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DEADBAND);
        return speed;
    }

    public double getWinchMotorSpeed() {
        double speed = -getSpeedFromAxis(_debug, _debug.getXChannel());
        speed = applyDeadband(speed, Constants.DEADBAND);
        return speed;
    }

    public double getStationarySpeed() {
        double speed = -getSpeedFromAxis(_debug, Gamepad.Axes.LEFT_X.getNumber());
        speed = applyDeadband(speed, Constants.DEADBAND);
        speed = 0;
        return speed;
    }
    
    public double getRockerSpeed() {
        double speed = -getSpeedFromAxis(_debug, Gamepad.Axes.RIGHT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DEADBAND);
        speed = 0;
        return speed;
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {

    }
}