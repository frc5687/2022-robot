/* Team 5687 (C)2020-2021 */
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.util.Helpers.*;

import org.frc5687.rapidreact.commands.Intaker;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.rapidreact.commands.LowerCatapult;
import org.frc5687.rapidreact.commands.Reset;
import org.frc5687.rapidreact.commands.ShootSetpoint;
import org.frc5687.rapidreact.commands.TestSpring;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;

public class OI extends OutliersProxy {
    private Joystick _translation;
    private Joystick _rotation;
    private Gamepad _debug;

    private Button _lowerArm;
    private JoystickButton _shootButton;
    private Button _shootButtonTest;
    private Button _release;

    private JoystickButton _intakeButton;

    private JoystickButton resetNavX;
    // "Raw" joystick values
    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        _translation = new Joystick(0);
        _rotation = new Joystick(1);

        _debug = new Gamepad(2);

//        _lowerArm = new JoystickButton(_debug, Gamepad.Buttons.A.getNumber());
        _shootButton= new JoystickButton(_translation, 1);
//        _shootButtonTest = new JoystickButton(_debug, Gamepad.Buttons.X.getNumber());
        _release = new JoystickButton(_debug, Gamepad.Buttons.Y.getNumber());
        _intakeButton = new JoystickButton(_rotation, 4);
    }

    public void initializeButtons(DriveTrain driveTrain, Catapult catapult, Intake intake) {
        //There's nothing to init here
//        _shootButton.whenPressed(new TestSpring(catapult, 0.115, 0.18));
//        _lowerArm.whenPressed(catapult::lockArm);
//        _lowerArm.whenPressed(new LowerCatapult(catapult));
//        _shootButtonTest.whenPressed(new Reset(catapult));
        _release.whenPressed(catapult::releaseArm);
        _shootButton.whenHeld(new Intaker(intake));
    }

    public boolean isShootButtonPressed() {
        return _shootButton.get();
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
