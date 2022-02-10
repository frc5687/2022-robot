/** Team 5687 (C)2021-2022
 * Joystick and gamepad control for the robot.
 * Also has button inits.
 * Has some instructions on how to switch controls.
*/
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.util.Helpers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.rapidreact.commands.LowerCatapult;
import org.frc5687.rapidreact.commands.ShootSetpoint;
import org.frc5687.rapidreact.commands.TestSpring;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;

public class OI extends OutliersProxy {
    // Joysticks and XBox controller
    protected Gamepad _gamepad;
    protected Joystick _rotation;
    protected Joystick _translation;

    private Gamepad _debug;

    private Button _shootButton;
    private Button _shootButton1;


    private JoystickButton resetNavX;
    // "Raw" joystick values
    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        _translation = new Joystick(0);
        _rotation = new Joystick(1);

        _debug = new Gamepad(2);

        _shootButton = new JoystickButton(_debug, Gamepad.Buttons.A.getNumber());
        _shootButton1 = new JoystickButton(_debug, Gamepad.Buttons.B.getNumber());
    }

    public void initializeButtons(DriveTrain driveTrain, Catapult catapult) {
        //There's nothing to init here
        _shootButton.whenPressed(new TestSpring(catapult));
        _shootButton1.whenPressed(new LowerCatapult(catapult));
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
