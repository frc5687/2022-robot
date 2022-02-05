/** Team 5687 (C)2021-2022
 * Joystick and gamepad control for the robot.
 * Also has button inits.
 * Has some instructions on how to switch controls.
*/
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.util.Helpers.*;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;

public class OI extends OutliersProxy {

    private double yIn = 0;
    private double xIn = 0;

    private ArrayList<Joystick> _joysticks = new ArrayList<Joystick>(10); 

    public OI() {

        addJoystick(ButtonMap.Controllers.DRIVER_JOYSTICK);
        addJoystick(ButtonMap.Controllers.OPERATOR_JOYSTICK);
        addGamepad(ButtonMap.Controllers.OPERATOR_GAMEPAD);
    }

    private void addJoystick(int port) {
        _joysticks.add(port, new Joystick(port));
    }

    private void addGamepad(int port) {
        _joysticks.add(port, new Gamepad(port));
    }


    private Joystick getJoystick(int port) {
        return _joysticks.get(port);
    }

    public void initializeButtons(DriveTrain driveTrain) {
        //There's nothing to init here
    }

    public double getDriveY() {
        Joystick translation = getJoystick(ButtonMap.Axes.Translation.Controller);

        //Comment for gamepad control
        yIn = getSpeedFromAxis(translation, ButtonMap.Axes.Translation.Y);
        
        //Uncomment for gamepad control
        // yIn = getSpeedFromAxis(Gamepad, Gamepad.getYChannel());
        yIn = applyDeadband(yIn, Constants.DriveTrain.DEADBAND);

        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        yOut = (yOut + (yIn * 2)) / 3.0;
        return yOut;
    }

    public double getDriveX() {
        Joystick translation = getJoystick(ButtonMap.Axes.Translation.Controller);
        //Comment for gamepad control
        xIn = -getSpeedFromAxis(translation, ButtonMap.Axes.Translation.Y);
        //Uncomment for gamepad control
        //xIn = -getSpeedFromAxis(Gamepad, Gamepad.getXChannel());
        xIn = applyDeadband(xIn, Constants.DriveTrain.DEADBAND);
        double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        xOut = (xOut + (xIn * 2)) / 3.0;
        return xOut;
    }

    public double getRotationX() {
        Joystick rotation = getJoystick(ButtonMap.Axes.Rotation.Controller);

        double speed = getSpeedFromAxis(rotation, ButtonMap.Axes.Rotation.Twist);
        speed = applyDeadband(speed, 0.2);
        return speed;
    }

    protected double getSpeedFromAxis(Joystick joystick, int axisNumber) {
        return joystick.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {}
}
