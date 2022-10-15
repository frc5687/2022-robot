/** Team 5687 (C)2021-2022
 * Joystick and gamepad control for the robot.
 * Also has button inits.
 * Has some instructions on how to switch controls.
*/
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.subsystems.Catapult.CatapultState.ZEROING;
import static org.frc5687.rapidreact.util.Helpers.*;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.frc5687.rapidreact.commands.AutoIntake;
import org.frc5687.rapidreact.commands.Climber.AutoClimb;
import org.frc5687.rapidreact.commands.Climber.RockerFlip;
import org.frc5687.rapidreact.commands.Climber.SemiAutoClimb;
import org.frc5687.rapidreact.commands.Climber.Stow;
import org.frc5687.rapidreact.commands.DriveTrajectory;
import org.frc5687.rapidreact.commands.catapult.SetSetpoint;
import org.frc5687.rapidreact.commands.catapult.SetState;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Indexer;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Lights;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;

public class OI extends OutliersProxy {

    // Joysticks and gamepads
    private Joystick _translation;
    private Joystick _rotation;
    private Gamepad _buttonpad;
    private Gamepad _drivepad;

    // Buttons
    private JoystickButton _autoAim;
    private JoystickButton _aimBall;
    private JoystickButton _catapultDebugButton;
    private JoystickButton _deployRetract;
    private JoystickButton _dropArm;
    private JoystickButton _exitKill;
    private JoystickButton _intakeButton;
    private JoystickButton _kill;
    private JoystickButton _preloadButton;
    private JoystickButton _readyToClimb;
    private JoystickButton _stowClimber;
    private JoystickButton _rockerFlip;
    private JoystickButton _release;
    private JoystickButton _resetNavX;
    private JoystickButton _shootButton;
    private JoystickButton _manualIndexer;
    private JoystickButton _turboDrive;
    private JoystickButton _extraClimb;


    private JoystickButton _shootSetpointOne;
    private JoystickButton _shootSetpointTwo;
    private JoystickButton _shootSetpointThree;

    private JoystickButton _autoShootToggle;

    private Lights _lights;

    private boolean _useGamepad = true;
    // "Raw" joystick values
    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        _translation = new Joystick(0);
        _rotation = new Joystick(1);
        _buttonpad = new Gamepad(2);
        _drivepad = new Gamepad(3);

        // buttonpad (fightstick)
        _catapultDebugButton = new JoystickButton(_buttonpad, Gamepad.Buttons.LEFT_BUMPER.getNumber());
        _preloadButton = new JoystickButton(_buttonpad, Gamepad.Buttons.LEFT_STICK.getNumber());
        _release = new JoystickButton(_buttonpad, Gamepad.Buttons.RIGHT_STICK.getNumber());
        _readyToClimb = new JoystickButton(_buttonpad, Gamepad.Buttons.B.getNumber());
        _stowClimber = new JoystickButton(_buttonpad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());
        _rockerFlip = new JoystickButton(_buttonpad, Gamepad.Buttons.X.getNumber());
        _manualIndexer = new JoystickButton(_buttonpad, Gamepad.Buttons.A.getNumber());
        _autoShootToggle = new JoystickButton(_buttonpad, Gamepad.Buttons.Y.getNumber());

        // adding buttons while driving: Ben pls look

//        _shootButton = new JoystickButton(_buttonpad, Gamepad.Buttons.Y.getNumber());

        // rotation joystick

        if (_useGamepad == true) {
            _intakeButton = new JoystickButton(_drivepad, 5); //right bumper
            _autoAim = new JoystickButton(_drivepad, 6); //left bumper
            // _deployRetract = new JoystickButton(_drivepad, 8); // what is this? mapped arbitrarily
            // _aimBall = new JoystickButton(_drivepad, 8); // what is this?
            // _extraClimb = new JoystickButton(_drivepad, 3);
        } else {
            _intakeButton = new JoystickButton(_rotation, 1);
            _autoAim = new JoystickButton(_rotation, 2);
            // _deployRetract = new JoystickButton(_rotation, 3);
            // _aimBall = new JoystickButton(_rotation, 4);
            // _extraClimb = new JoystickButton(_rotation, 7);
        }

        // translation joystick
        if (_useGamepad == true) {
            _shootButton= new JoystickButton(_drivepad, 1);
            // _release = new JoystickButton(_drivepad, 2);
            // _dropArm = new JoystickButton(_drivepad, 7);
            // _resetNavX = new JoystickButton(_drivepad, 9);
            _turboDrive = new JoystickButton(_drivepad, 1);
        } else {
            _shootButton= new JoystickButton(_translation, 1);
            // _release = new JoystickButton(_translation, 6);
            // _dropArm = new JoystickButton(_translation, 3);
            // _resetNavX = new JoystickButton(_translation, 5);
            _turboDrive = new JoystickButton(_translation, 2);
        }

        // while driving, ben check.
        if (_useGamepad == true) {
            // _shootSetpointOne = new JoystickButton(_translation, 9);
            // _shootSetpointTwo = new JoystickButton(_translation, 10);
            // _shootSetpointThree = new JoystickButton(_translation, 11);

            // _exitKill = new JoystickButton(_translation, 8);
            // _kill = new JoystickButton(_translation, 7);
        // } else if (_useGamepad == true) { uncomment later to add buttons
        //     _shootSetpointOne = new JoystickButton(_drivepad, 9);
        //     _shootSetpointTwo = new JoystickButton(_drivepad, 10);
        //     _shootSetpointThree = new JoystickButton(_drivepad, 11);
    
        //     _exitKill = new JoystickButton(_drivepad, 8);
        //     _kill = new JoystickButton(_drivepad, 7);
        } else {
            // _shootSetpointOne = new JoystickButton(_translation, 9);
            // _shootSetpointTwo = new JoystickButton(_translation, 10);
            // _shootSetpointThree = new JoystickButton(_translation, 11);
    
            // _exitKill = new JoystickButton(_translation, 8);
            // _kill = new JoystickButton(_translation, 7);
        }
        
    }

    public void initializeButtons(DriveTrain driveTrain, Catapult catapult, Intake intake, Climber climber, Indexer indexer) {
        // driving, Ben check pls.
        // _shootSetpointOne.whenPressed(new SetSetpoint(catapult, Catapult.CatapultSetpoint.FAR));
        // _shootSetpointTwo.whenPressed(new SetSetpoint(catapult, Catapult.CatapultSetpoint.MID));
        // _shootSetpointThree.whenPressed(new SetSetpoint(catapult, Catapult.CatapultSetpoint.NEAR));


        _preloadButton.whenPressed(new SetState(catapult, ZEROING));
        _intakeButton.whenHeld(new AutoIntake(intake, catapult));
        // _resetNavX.whenPressed(driveTrain::resetYaw);
        _readyToClimb.whenPressed(new AutoClimb(climber));
        // _extraClimb.whenPressed(new AutoClimb(climber));
        _stowClimber.whenPressed(new Stow(climber));
        _rockerFlip.whenPressed(new RockerFlip(climber));
        _manualIndexer.whenPressed(indexer::up);
        _autoShootToggle.whenPressed(catapult::toggleAutomatShoot);
    }

    public boolean hold = false;

    public boolean readyToClimb() { return _readyToClimb.get(); }
    public boolean isShootButtonPressed() { return _shootButton.get(); }
    public boolean exitDebugCatapult() { return _catapultDebugButton.get(); }
    public boolean preloadCatapult() {
        return false;
//        return _preloadButton.get();
    }
    public boolean releaseArm() {return false;} //{ return _release.get(); }
    public boolean intakeDeployRetract() {return false;} //{ return _deployRetract.get(); }
    public boolean exitKill() {return false;} //{ return _exitKill.get(); }
    public boolean kill() {return false;} //{ return _kill.get(); }
    public boolean autoAim() { return _autoAim.get(); }
    public boolean aimBall() {return false;} //{ return _aimBall.get(); }
    public boolean turbo() {return _turboDrive.get(); }

    public double getDriveY() {
        if (_useGamepad == true) {
            yIn = Math.pow(-getSpeedFromAxis(_drivepad, Gamepad.Axes.LEFT_Y.getNumber()), 3);
        } else {
            yIn = -getSpeedFromAxis(_translation, _translation.getYChannel());
        }
        yIn = applyDeadband(yIn, Constants.DriveTrain.DEADBAND);

        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        yOut = (yOut + (yIn * 2)) / 3.0;
        return yOut;
    }

    public double getDriveX() {
        if (_useGamepad == true) {
            xIn = Math.pow(-getSpeedFromAxis(_drivepad, Gamepad.Axes.LEFT_X.getNumber()), 3);   
        } else {
            xIn = -getSpeedFromAxis(_translation, _translation.getXChannel());
        }
        xIn = applyDeadband(xIn, Constants.DriveTrain.DEADBAND);
        double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        xOut = (xOut + (xIn * 2)) / 3.0; // numbers from empirical testing.
        return xOut;
    }

    public double getRotationX() {
        double speed;
        if (_useGamepad == true) {
            speed = Math.pow(-getSpeedFromAxis(_drivepad, Gamepad.Axes.RIGHT_X.getNumber()), 3);
        } else {
            speed = -getSpeedFromAxis(_rotation, _rotation.getXChannel());
        }
        speed = applyDeadband(speed, Constants.DEADBAND);
        return speed;
    }

    // public double getSpringMotorSpeed() {
    //     double speed = -getSpeedFromAxis(_buttonpad, Gamepad.Axes.LEFT_Y.getNumber());
    //     speed = applyDeadband(speed, Constants.DEADBAND);
    //     return speed;
    // }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {

    }
}