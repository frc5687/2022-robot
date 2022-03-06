/* (C)2020-2021 */
package org.frc5687.rapidreact.subsystems;

import org.frc5687.rapidreact.util.ILoggingSource;
import org.frc5687.rapidreact.util.MetricTracker;
import org.frc5687.rapidreact.util.OutliersContainer;
import org.frc5687.rapidreact.util.RioLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base class to provide metrics and logging infrustructure.
 * 
 * See ILoggingSource for logging details.
 * 
 * See MetricTracker for metrics details.
 * 
 * Example of metrics collection. In a child class's constructor:
 * 
 * public SampleSystem() {
 *    logMetrics("foo", "bar", "baz");
 * }
 * 
 * Later on in the child class, eg in setSpeed():
 * 
 * ...
 * metric("foo", 123);
 * metric("bar", "elvis");
 * metric("baz", 42.42);
 * metric("pants", 99);    <~ This metric won't get written to USB storage because it wasn't registered.
 * ...
 * 
 * Note that metric logging is expensive, so it is turned OFF by default.
 * To turn it on, either call enableMetrics() from code (eg at the start of an auto command)
 * or set Metrics/[SubSystemName] to true in the SmartDashboard 
 * 
 */
public abstract class OutliersSubsystem extends SubsystemBase implements ILoggingSource {
    private MetricTracker _metricTracker;

    public OutliersSubsystem(OutliersContainer container) {
        container.registerSubSystem(this);
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
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
        if (_metricTracker != null) {
            _metricTracker.put(name, value);
        }
    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
        if (_metricTracker != null) {
            _metricTracker.put(name, value);
        }
    }

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
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
}
