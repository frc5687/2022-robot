/* Team 5687 (C)2020-2021 */
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.util.Helpers.*;
import org.frc5687.rapidreact.commands.Intaker;
import org.frc5687.rapidreact.commands.Climber.ArmUp;
import org.frc5687.rapidreact.commands.Climber.ClimberDown;
import org.frc5687.rapidreact.commands.Climber.Rock;
import org.frc5687.rapidreact.commands.Climber.SecondStep;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;

public class OI extends OutliersProxy {
    protected Gamepad _debug;
    protected Joystick _leftJoystick;
    protected Joystick _rightJoystick;

    protected Button _driverRightStickButton;

    private JoystickButton _intakeBTN;
    private JoystickButton  _climberUp;
    private JoystickButton _firstStep;
    private JoystickButton _secondStep;

    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        _debug = new Gamepad(0);
        _leftJoystick = new Joystick(1);
        _rightJoystick = new Joystick(0);
        _intakeBTN = new JoystickButton(_leftJoystick, 4);
        _climberUp = new JoystickButton(_debug, Gamepad.Buttons.A.getNumber());
        _firstStep = new JoystickButton(_debug, Gamepad.Buttons.B.getNumber());
        _secondStep = new JoystickButton(_debug, Gamepad.Buttons.X.getNumber());
    }

    public void initializeButtons(DriveTrain driveTrain, Intake intake, Climber climber) {
        _intakeBTN.whenHeld(new Intaker(intake));
        _climberUp.whenPressed(new ArmUp(climber));
        _firstStep.whenPressed(new ClimberDown(climber));
        _secondStep.whenPressed(new SecondStep(climber));
    }

    public double getDriveY() {
        yIn = getSpeedFromAxis(_leftJoystick, _leftJoystick.getYChannel());
        //yIn = getSpeedFromAxis(_debug, Gamepad.Axes.LEFT_Y.getNumber());
        yIn = applyDeadband(yIn, Constants.DriveTrain.DEADBAND);

        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        yOut = (yOut + (yIn * 2)) / 3.0;
        return yOut;
    }

    public double getDriveX() {
        xIn = -getSpeedFromAxis(_leftJoystick, _leftJoystick.getXChannel());
       // xIn = -getSpeedFromAxis(_debug, Gamepad.Axes.LEFT_X.getNumber());
        xIn = applyDeadband(xIn, Constants.DriveTrain.DEADBAND);

        double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        xOut = (xOut + (xIn * 2)) / 3.0;
        return xOut;
    }

    public double getRotationX() {
        double speed = getSpeedFromAxis(_rightJoystick, _rightJoystick.getXChannel());
        speed = applyDeadband(speed, 0.2);
        return speed;
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {}
}