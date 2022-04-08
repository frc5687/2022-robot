/* Team 5687 (C)2020-2022 */
package org.frc5687.rapidreact.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5687.rapidreact.util.*;

/**
 * Base class to provide metrics and logging infrustructure.
 *
 * <p>See ILoggingSource for logging details.
 *
 * <p>See MetricTracker for metrics details.
 *
 * <p>Example of metrics collection. In a child class's constructor:
 *
 * <p>public SampleSystem() { logMetrics("foo", "bar", "baz"); }
 *
 * <p>Later on in the child class, eg in setSpeed():
 *
 * <p>... metric("foo", 123); metric("bar", "elvis"); metric("baz", 42.42); metric("pants", 99); <~
 * This metric won't get written to USB storage because it wasn't registered. ...
 *
 * <p>Note that metric logging is expensive, so it is turned OFF by default. To turn it on, either
 * call enableMetrics() from code (eg at the start of an auto command) or set
 * Metrics/[SubSystemName] to true in the SmartDashboard
 */
public abstract class OutliersSubsystem extends SubsystemBase
        implements ILoggingSource, OutlierPeriodic {
    private MetricTracker _metricTracker;
    private boolean _isLogging;

    public OutliersSubsystem(OutliersContainer container) {
        container.registerSubSystem(this);
        _isLogging = container.isLogging();
    }

    public boolean isLogging() {
        return _isLogging;
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

    public void metric(String name, String value) {
        if (_isLogging) {
            SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
        }
        if (_metricTracker != null) {
            _metricTracker.put(name, value);
        }
    }

    public void metric(String name, double value) {
        if (_isLogging) {
            SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
        }
        if (_metricTracker != null) {
            _metricTracker.put(name, value);
        }
    }

    public void metric(String name, boolean value) {
        if (_isLogging) {
            SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
        }
        if (_metricTracker != null) {
            _metricTracker.put(name, value);
        }
    }

    protected void logMetrics(String... metrics) {
        _metricTracker = MetricTracker.createMetricTracker(getClass().getSimpleName(), metrics);
    }

    public abstract void updateDashboard();

    public void enableMetrics() {
        if (_metricTracker != null) {
            _metricTracker.enable();
        }
    }

    public void disableMetrics() {
        if (_metricTracker != null) {
            _metricTracker.disable();
        }
    }

    @Override
    public void controlPeriodic(double timestamp, double dt) {}

    @Override
    public void dataPeriodic(double timestamp, double dt) {}
}
