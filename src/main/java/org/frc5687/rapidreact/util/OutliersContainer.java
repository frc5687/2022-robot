/* (C)2021 */
package org.frc5687.rapidreact.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.LinkedList;
import java.util.List;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;

/**
 * The Outliers
 */
public abstract class OutliersContainer implements ILoggingSource {
    private List<OutliersSubsystem> _subsystems = new LinkedList<OutliersSubsystem>();
    private IdentityMode _identityMode;
    private boolean _isLogging;

    public OutliersContainer(IdentityMode identityMode, boolean isLogging) {
        _identityMode = identityMode;
        _isLogging = isLogging;
    }

    public void metric(String name, boolean value) {
        if (_isLogging) {
            SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
        }
    }

    public void metric(String name, String value) {
        if (_isLogging) {
            SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
        }
    }

    public void metric(String name, double value) {
        if (_isLogging) {
            SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
        }
    }

    @Override
    public void error(String message) {
        RioLogger.error(this, message);
    }

    @Override
    public void warn(String message) {
        RioLogger.warn(this, message);
    }

    @Override
    public void info(String message) {
        RioLogger.info(this, message);
    }

    @Override
    public void debug(String message) {
        RioLogger.debug(this, message);
    }

    /***
     * Registers a subsystem for periodic actions.
     * @param subsystem
     */
    public void registerSubSystem(OutliersSubsystem subsystem) {
        if (!_subsystems.contains(subsystem)) {
            _subsystems.add(subsystem);
        }
    }

    public boolean isLogging() {
        return _isLogging;
    }

    /***
     * Unregisters a subsystem for periodic actions.
     * @param subsystem
     */
    public void unregisterSubSystem(OutliersSubsystem subsystem) {
        if (_subsystems.contains(subsystem)) {
            _subsystems.remove(subsystem);
        }
    }

    public void updateDashboard() {
        if (_isLogging) {
            _subsystems.forEach(OutliersSubsystem::updateDashboard);
        }
    }

    public void disabledPeriodic() {}

    public void disabledInit() {}

    public void teleopInit() {}

    public void autonomousInit() {}

    /**
     * The identity mode of the robot currently running the code.
     * IdentityMode allows SubSystems and Commands to behave differently on different versions of the robot.
     * For example, the programming bot usually doesn't have all subsystems on it, and the PID constants
     * for practice and competition bots are often different.
     * 
     * Identity is set by creating a file called "frc5687.cfg" on a USB thumbdrive inserted into the RoboRio.
     * On startup, OutliersRobot reads this file and gets the identity, name, and logging levels.
     * 
     */
    public enum IdentityMode {
        competition(0),
        practice(1),
        programming(2);

        private int _value;

        IdentityMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public IdentityMode getIdentityMode() {
        return _identityMode;
    }
}
