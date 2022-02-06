/** Team 5687 (C)2021-2022
 * Joystick and gamepad control for the robot.
 * Also has button inits.
 * Has some instructions on how to switch controls.
*/
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.util.Helpers.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.util.AxisButton;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;

/**
 * To add a button to control a subsystem there are a number of steps needed.  I'll use SHOOT as an example:
 * 1) Define the button in the ButtonMap.Buttons class:
 * 
 *         public static class SHOOT {
 *           public static int Controller = Controllers.DRIVER_JOYSTICK;
 *           public static int Button = 0;
 *         }
 *
 * 2) Add a private member variable for the button:
 * 
 *        private JoystickButton _shootButton;
 * 
 * 3) Instantiate the button in the OI() constructor, referencing the new ButtonMap entry:
 * 
 *        _shootButton = addJoystickButton(ButtonMap.Buttons.SHOOT.Controller, ButtonMap.Buttons.SHOOT.Button);
 * 
 * 4) Add the subsystem to the signature for OI.initializeButtons:
 * 
 *        public void initializeButtons(DriveTrain driveTrain, Shooter shooter)
 * 
 * 5) Initialize the button in OI.initializeButtons:
 * 
 *        _shootButton.whenHeld(new Shoot(shooter));
 * 
 * 6) Add the subsystem to the RobotContainer.init() call to _oi.inializeButtons:
 * 
 *        _oi.initializeButtons(_driveTrain, _shooter);
 * 
 */
public class OI extends OutliersProxy {

    private double yIn = 0;
    private double xIn = 0;

    private Joystick[] _joysticks = new Joystick[10]; 

    // private JoystickButton _shootButton;

    public OI() {
        addJoystick(ButtonMap.Controllers.DRIVER_JOYSTICK);
        addJoystick(ButtonMap.Controllers.OPERATOR_JOYSTICK);
        addGamepad(ButtonMap.Controllers.OPERATOR_GAMEPAD);

        // shootButton = addJoystickButton(ButtonMap.Buttons.SHOOT.Controller, ButtonMap.Buttons.SHOOT.Button);
    }

    public void initializeButtons(DriveTrain driveTrain/*, Shooter shooter*/) {
        //There's nothing to init here

        // shootButton.whenHeld(new Shoot(shooter));

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

    /**
     * Instantiates a Joystick on the specified port and adds it to the _joysticks array.
     * 
     * @param port Pass -1 to skip this joystick.
     */
    private void addJoystick(int port) {
        if (port < 0) { return; }
        _joysticks[port] = new Joystick(port);
    }

    /**
     * Instantiates Gamepad on the specified port and adds it to the _joysticks array.
     * 
     * @param port Pass -1 to skip this gamepad.
     */
    private void addGamepad(int port) {
        if (port < 0) { return; }
        _joysticks[port] = new Gamepad(port);
    }

    /**
     * Returns the joystick assigned to a specific port.  Returns null is no joystick assigned or port is -1.
     * @param port
     * @return
     */
    private Joystick getJoystick(int port) {
        if (port < 0) { return null; }
        return _joysticks[port];
    }

    private JoystickButton addJoystickButton(int port, int buttonNumber) {
        Joystick joystick = getJoystick(port);
        return new JoystickButton(joystick, buttonNumber);
    }

    private AxisButton addAxisButton(int port, int buttonNumber, double threshold) {
        Joystick joystick = getJoystick(port);
        return new AxisButton(joystick, buttonNumber, threshold);
    }


}
