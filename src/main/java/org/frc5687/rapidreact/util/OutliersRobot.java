/* (C)2021 */
package org.frc5687.rapidreact.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import org.frc5687.rapidreact.Constants;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class OutliersRobot extends TimedRobot implements ILoggingSource {

    public static OutliersContainer.IdentityMode _identityMode = OutliersContainer.IdentityMode.competition;
    private RioLogger.LogLevel _dsLogLevel = RioLogger.LogLevel.warn;
    private RioLogger.LogLevel _fileLogLevel = RioLogger.LogLevel.warn;
    private String _name = "Anonymous";


    private Notifier _metricNotifier;

    public OutliersRobot() {
        super(Constants.UPDATE_PERIOD);
    }

    @Override
    public void robotInit() {
        super.robotInit();
        loadConfigFromUSB();
        RioLogger.getInstance().init(_fileLogLevel, _dsLogLevel);

        metric("Name", _name);
        metric("Identity", _identityMode.toString());
        metric("Commit", Version.REVISION);
        metric("Branch", Version.BRANCH);

        info("Robot " + _name + " running in " + _identityMode.toString() + " mode");
        info("Running commit " + Version.REVISION + " of branch " + Version.BRANCH);

        _metricNotifier = new Notifier(MetricTracker::flushAll);
        _metricNotifier.startPeriodic(Constants.METRIC_FLUSH_PERIOD);

    }

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, String value) {
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
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

    /**
     * Loads the robot config settings from a thumbdrive in the RoboRio's USB port, if available.
     * 
     *  
     * A typical file would look like:
     * name=Freddy
     * mode=competition
     * dsloglevel=error
     * fileloglevel=none
     * 
     * name specifies the name of the robot, if it is named.
     * mode specifis the identiy mode (see OutliersContainer.IdentityMode)
     * dsloglevel specifies the lowest level message to send to the driverstation
     * fileloglevel specifies the lowest level message to send to the USB drive
     * levels are: error > warn > info > debug
     * 
     * If no file is found, or the USB drive can't be read, OutliersRobot will default to competition mode.
     */
    protected void loadConfigFromUSB() {
        try {
            String usbDir = "/U/"; // USB drive is mounted to /U on roboRIO
            String configFileName = usbDir + "frc5687.cfg";
            File configFile = new File(configFileName);
            FileReader reader = new FileReader(configFile);
            BufferedReader bufferedReader = new BufferedReader(reader);

            String line;
            while ((line = bufferedReader.readLine()) != null) {
                processConfigLine(line);
            }

            bufferedReader.close();
            reader.close();
        } catch (Exception e) {
            _identityMode = OutliersContainer.IdentityMode.competition;
        }
    }

    private void processConfigLine(String line) {
        try {
            if (line.startsWith("#")) {
                return;
            }
            String[] a = line.split("=");
            if (a.length == 2) {
                String key = a[0].trim().toLowerCase();
                String value = a[1].trim();
                switch (key) {
                    case "name":
                        _name = value;
                        break;
                    case "mode":
                        _identityMode = OutliersContainer.IdentityMode.valueOf(value.toLowerCase());
                        break;
                    case "fileloglevel":
                    case "usblevel":
                        _fileLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
                        break;
                    case "dsloglevel":
                    case "driverstationloglevel":
                        _dsLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
                        break;
                }
            }
        } catch (Exception e) {

        }
    }

}
