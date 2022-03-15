/* (C)2020-2021 */
package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.util.ILoggingSource;
import org.frc5687.rapidreact.util.MetricTracker;
import org.frc5687.rapidreact.util.RioLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The OutliersCommand base class introduces logging and metric tracking capabilities for Commands.
 * 
 * <p> Commands are simple state machines that perform high-level robot functions using the methods
 * defined by subsystems. Commands can be either idle, in which they do nothing, or scheduled,
 * in which caseCommandScheduler will execute a specific set of the command’s code depending on the state
 * of the command:
 * 
 * <ol>
 *  <li> initializing => initialize()
 *  <li> executing => execute()
 *  <li> ending => end()
 * </ol> 
 *
 * 
 * <p> Example of metrics collection. In a child class's constructor:
 * 
 * <pre>
 * public SampleCommand() {
 *    logMetrics("foo", "bar", "baz");
 * }
 * </pre>
 * 
 * Later on in the child class, eg in run():
 * 
 * <pre>
 * metric("foo", 123);
 * metric("bar", "elvis");
 * metric("baz", 42.42);
 * metric("pants", 99); // This metric won't get written to USB storage because it wasn't registered.
 * </pre>
 * 
 * <p>Note that metric logging is expensive, so it is turned OFF by default.
 * To turn it on, either call enableMetrics() from code (eg at the start of an auto command)
 * or set Metrics/[SubSystemName] to true in the SmartDashboard 
 * 
 * @see ILoggingSource ILoggingSource for logging details.
 * @see MetricTracker MetricTracker for metrics details.
 */
 public abstract class OutliersCommand extends CommandBase implements ILoggingSource {
    private MetricTracker _metricTracker;

    public OutliersCommand() {}

    public OutliersCommand(double timeout) {
        super.withTimeout(timeout);
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
        _metricTracker.pause();
    }

    @Override
    public void initialize() {
        super.initialize();
        info("Initialized");
        if (_metricTracker != null) {
            _metricTracker.resume();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        info("Ended");
        if (_metricTracker != null) {
            _metricTracker.pause();
        }
    }

    @Override
    public void execute() {
        if (_metricTracker != null && _metricTracker.isPaused()) {
            _metricTracker.resume();
        }
    }

    protected void enableMetrics() {
        if (_metricTracker != null) {
            _metricTracker.enable();
        }
    }

    protected void disableMetrics() {
        if (_metricTracker != null) {
            _metricTracker.disable();
        }
    }

    protected void innerExecute() {}
}
