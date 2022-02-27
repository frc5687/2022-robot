/* Team 5687 (C)2020-2021 */
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.util.Helpers.*;
import org.frc5687.rapidreact.commands.Climber.PrepToClimb;
import org.frc5687.rapidreact.commands.Climber.Stow;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;

public class OI extends OutliersProxy {
    private Joystick _translation;
    private Joystick _rotation;
    private Gamepad _debug;
    private Button _catapultDebugButton;
    private Button _preloadButton;
    private JoystickButton _shootButton;
    private Button _release;

    private JoystickButton _kill;
    private JoystickButton _exitKill;

    private JoystickButton _stow;
    private JoystickButton _prepClimb;

    private JoystickButton _deployRetract;
    private JoystickButton _intakeButton;

    private JoystickButton  _setState;
    private JoystickButton resetNavX;
    private JoystickButton _resetNavX;
    private JoystickButton _dropArm;
    // "Raw" joystick values
    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        _translation = new Joystick(0);
        _rotation = new Joystick(1);
        _debug = new Gamepad(2);

        _stow = new JoystickButton(_debug, Gamepad.Buttons.RIGHT_BUMPER.getNumber());
        _prepClimb = new JoystickButton(_debug, Gamepad.Buttons.A.getNumber());
//        _release = new JoystickButton(_debug, Gamepad.Buttons.X.getNumber());
//        _shootButton = new JoystickButton(_debug, Gamepad.Buttons.Y.getNumber());

    }

    public void initializeButtons(DriveTrain driveTrain, Catapult catapult , Intake intake, Climber climber) {
        //There's nothing to init here
//        _shootButton.whenPressed(new TestSpring(catapult, 0.105, 0.245));
//        _lowerArm.whenPressed(catapult::lockArm);
//        _lowerArm.whenPressed(new LowerCatapult(catapult));
//        _shootButtonTest.whenPressed(new Reset(catapult));
//        _release.whenPressed(catapult::releaseArm);
//        _setState.whenPressed(new SetState(catapult, Catapult.CatapultState.AIMING));
//        _dropArm.whenHeld(new Feed(servoStop));
        _stow.whenPressed(new Stow(climber));
        _prepClimb.whenPressed(new PrepToClimb(climber));

    }


    public double getDriveY() {
        //Comment for gamepad control
        yIn = getSpeedFromAxis(_translation, _translation.getYChannel());
        // yIn = getSpeedFromAxis(Gamepad, Gamepad.getYChannel());
        yIn = applyDeadband(yIn, Constants.DriveTrain.DEADBAND);

        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        yOut = (yOut + (yIn * 2)) / 3.0;
        return yOut;
    }

    public double getDriveX() {
        //Comment for gamepad control
        xIn = -getSpeedFromAxis(_translation, _translation.getXChannel());
        //xIn = -getSpeedFromAxis(Gamepad, Gamepad.getXChannel());
        xIn = applyDeadband(xIn, Constants.DriveTrain.DEADBAND);

        double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        xOut = (xOut + (xIn * 2)) / 3.0; // numbers from empirical testing.
        return xOut;
    }

    public double getRotationX() {
        double speed = getSpeedFromAxis(_rotation, _rotation.getXChannel());
        speed = applyDeadband(speed, Constants.DEADBAND);
        return speed;
    }

    public double getSpringMotorSpeed() {
        double speed = -getSpeedFromAxis(_debug, _debug.getYChannel());
        speed = applyDeadband(speed, Constants.DEADBAND);
        return speed;
    }

    public double getWinchMotorSpeed() {
        double speed = -getSpeedFromAxis(_debug, _debug.getXChannel());
        speed = applyDeadband(speed, Constants.DEADBAND);
        return speed;
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {}
}