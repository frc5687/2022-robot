/* Team 5687 (C)2020-2022 */
package org.frc5687.rapidreact;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frc5687.rapidreact.util.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends OutliersRobot implements ILoggingSource {

    private int _updateTick = 0;
    private RobotContainer _robotContainer;
    private boolean _fmsConnected;
    private Command _autoCommand;
    private Timer _timer;
    private double _prevTime;
    private double _time;

    /** Run once when robot is turned on */
    @Override
    public void robotInit() {
        super.robotInit();

        LiveWindow.disableAllTelemetry();
        DriverStation.silenceJoystickConnectionWarning(true);

        _robotContainer = new RobotContainer(this, _identityMode);
        _timer = new Timer();
        _robotContainer.init();

        _time = _timer.get();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        ourPeriodic();
    }

    /** Run once every time entering autonomous mode
     * 
     * <p> Autonomous command set in RobotContainer
     */
    @Override
    public void autonomousInit() {
        //wrapCommand drops intake before the autocommand.
        // Jack did this to save typing in robotContainer
        _autoCommand = _robotContainer.wrapCommand(_robotContainer.getAutonomousCommand());
        _fmsConnected = DriverStation.isFMSAttached();
        _robotContainer.autonomousInit();
        if (_autoCommand != null) {
            _autoCommand.schedule();
        }
    }

    public void teleopInit() {
        _fmsConnected = DriverStation.isFMSAttached();

        // Good practice to cancel the autonomous command that may still be running.
        // If you want the autonomous to continue until interrupted by another command,
        // comment the following out.
        if (_autoCommand != null) {
            _autoCommand.cancel();
        }

        _robotContainer.teleopInit();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    private void ourPeriodic() {

        // Example of starting a new row of metrics for all instrumented objects.
        // MetricTracker.newMetricRowAll();
        MetricTracker.newMetricRowAll();
        // If you comment out _robotContainer.periodic(), provide another way to poll
        // Drive Station for starting position and auto mode to run
        _robotContainer.periodic();
        CommandScheduler.getInstance().run();
        update();
        updateDashboard();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // _limelight.disableLEDs();
        RioLogger.getInstance().forceSync();
        _robotContainer.disabledInit();
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
        _robotContainer.disabledPeriodic();
    }

    public void updateDashboard() {
        _updateTick++;
        if (_updateTick >= Constants.TICKS_PER_UPDATE) {
            _updateTick = 0;
            _robotContainer.updateDashboard();
        }
    }


    private void update() {}
}
