/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.frc5687.rapidreact.commands.*;
import org.frc5687.rapidreact.commands.auto.*;
import org.frc5687.rapidreact.commands.Climber.IdleClimber;

import org.frc5687.rapidreact.config.Auto;

import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Indexer;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;

import org.frc5687.rapidreact.util.AutoChooser;
import org.frc5687.rapidreact.util.JetsonProxy;
import org.frc5687.rapidreact.util.Limelight;
import org.frc5687.rapidreact.util.OutliersContainer;

public class RobotContainer extends OutliersContainer {
    
    private OI _oi;
    private AHRS _imu;
    private JetsonProxy _proxy;
    private Limelight _limelight;

    private Robot _robot;
    private DriveTrain _driveTrain;
    private Catapult _catapult;
    private Intake _intake;
    private Climber _climber;

    private AutoChooser _autoChooser;
    private Boolean _positionOne;
    private Boolean _positionTwo;
    private Boolean _positionThree;
    private Boolean _positionFour;
    private String _autoModeString;
    AutoChooser.Position autoPosition;
    AutoChooser.Mode autoMode;

    private UsbCamera _cam;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    /** Run once when robot code starts. */
    public void init() {
        // initialize peripherals. Do this before subsystems.
        // info("Running RobotContainer.init()");

        // Initialize starting position and mode to unknown
        // In disabledPeriodic() we will poll Drive Station for values
        _positionOne = false;
        _positionTwo = false;
        _positionThree = false;
        _positionFour = false;
        autoPosition = AutoChooser.Position.Unknown;
        _autoModeString = "Unknown";
        autoMode = AutoChooser.Mode.Unknown;

        // Display starting position value
        SmartDashboard.putString("DB/String 0", "Starting Position:");
        SmartDashboard.putString("DB/String 5", "Unknown");
   
        // Display auto mode value
        SmartDashboard.putString("DB/String 1", "Auto Mode:");
        String _automodeString = SmartDashboard.getString("Auto Selector", _autoModeString);
        SmartDashboard.putString("DB/String 6", _automodeString);

        // Auto mode chooser
        String [] modes = { "Zero Ball", "One Ball", "Two Ball" };
        // String [] modes = { "Zero Ball", "One Ball", "Two Ball", "Three Ball", "Four Ball", "Five Ball" };
        SmartDashboard.putStringArray("Auto List", modes);

        _oi = new OI();
        //Config the NavX
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _proxy = new JetsonProxy(10);
        _limelight = new Limelight("limelight");

        // then subsystems
        _driveTrain = new DriveTrain(this, _oi, _proxy, _limelight, _imu);
        _intake = new Intake(this);
        _climber = new Climber(this, _driveTrain);
        _catapult = new Catapult(this);
        _autoChooser = new AutoChooser();

        initializeCamera();

        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        setDefaultCommand(_intake, new IdleIntake(_intake, _oi));
        setDefaultCommand(_catapult, new DriveCatapult(_catapult, _intake, _driveTrain, _oi));
//        setDefaultCommand(_catapult, new IdleCatapult(_catapult, _oi));
        setDefaultCommand(_climber, new IdleClimber(_climber, _oi));

        // initialize OI after subsystems.
        _oi.initializeButtons(_driveTrain, _catapult, _intake, _climber);

        // Run periodic for each swerve module faster than regular cycle time
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);

        _imu.reset();
    }

    public void periodic() {}

    /** Run every cycle when robot disabled */
    public void disabledPeriodic() {

        // Poll Drive Station for starting position and auto mode selector

        // Use existing value as default so if WiFi drops we don't lose settings

        // Poll starting position buttons
        _positionOne = SmartDashboard.getBoolean("DB/Button 0", _positionOne);
        _positionTwo = SmartDashboard.getBoolean("DB/Button 1", _positionTwo);
        _positionThree = SmartDashboard.getBoolean("DB/Button 2", _positionThree);
        _positionFour = SmartDashboard.getBoolean("DB/Button 3", _positionFour);

        // Set position to highest value that is selected
        if (_positionOne) {
            autoPosition = AutoChooser.Position.First;
            SmartDashboard.putString("DB/String 5", "One");
        }
        if (_positionTwo) {
            autoPosition = AutoChooser.Position.Second;
            SmartDashboard.putString("DB/String 5", "Two");
        }
        if (_positionThree) {
            autoPosition = AutoChooser.Position.Third;
            SmartDashboard.putString("DB/String 5", "Three");
        }
        if (_positionFour) {
            autoPosition = AutoChooser.Position.Fourth;
            SmartDashboard.putString("DB/String 5", "Four");
        }
        if (autoPosition == AutoChooser.Position.Unknown) {
            SmartDashboard.putString("DB/String 5", "Unknown");
        }

        // Set auto mode based on Smart Dashboard pull down
        _autoModeString = SmartDashboard.getString("Auto Selector", _autoModeString);
        switch(_autoModeString) {
            case "Zero Ball":
                autoMode = AutoChooser.Mode.ZeroBall;
                metric("Zero Ball", true);
                break;
            case "One Ball":
                autoMode = AutoChooser.Mode.OneBall;
                metric("One ball", true);
                break;
            case "Two Ball":
                autoMode = AutoChooser.Mode.TwoBall;
                metric("Two ball", true);
                break;
            case "Three Ball":
            case "Four Ball":
            case "Five Ball":
            default:
                autoMode = AutoChooser.Mode.Unknown;
        }

        // Display auto mode selector
        SmartDashboard.putString("DB/String 6", _autoModeString);

    }

    @Override
    public void disabledInit() {
        //Runs once during disabled
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void autonomousInit() {
        // Run once when entering auto mode
        // info("Running RobotContainer.autonomousInit()");
    }

    /**
     * Initialize web camera mounted on the robot.
     * Either use auto web exposure or use custom exposure
     */
    public void initializeCamera(){
        _cam = CameraServer.startAutomaticCapture();
        _cam.setBrightness(Constants.Camera.BRIGHTNESS);
        _cam.setResolution(Constants.Camera.HEIGHT, Constants.Camera.WIDTH);
        _cam.setFPS(Constants.Camera.FPS_LIMIT);
        if(Constants.Camera.AUTO_EXPOSURE){
            _cam.setExposureAuto();
        }else{
            _cam.setExposureManual(Constants.Camera.EXPOSURE);
        }
    }

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    public Command wrapCommand(Command command) {
        return new SequentialCommandGroup(new DropIntake(_intake), new WaitCommand(0.5), command);
    }

    public Command getAutonomousCommand() {
        // This is just for testing so Dennis and Jack can bypass chooser code
        Boolean _bypass = false;

        // Set _bypass to true to set autonomous command here instead of using Drive Station
        if (_bypass) {
            AutoChooser.Position _startingPosition = AutoChooser.Position.Third;
            info("Running twoball.");
            return new TwoBallAuto(_driveTrain, _catapult, _intake, _startingPosition);
        }

        // Return command sequence based on auto mode selected and pass in starting position
        switch (autoMode) {
            case ZeroBall:
                return new ZeroBallAuto(_driveTrain, autoPosition);
            case OneBall:
                return new OneBallAuto(_driveTrain, _catapult, autoPosition);
            case TwoBall:
                return new TwoBallAuto(_driveTrain, _catapult, _intake, autoPosition);
            default:
                return new Wait(15);
        }

    }

    @Override
    public void updateDashboard() {
//        if (_proxy.getLatestFrame() != null) {
//            metric("Millis", _proxy.getLatestFrame().getMillis());
//            metric("Has goal", _proxy.getLatestFrame().hasTarget());
//            metric("Object Distance", _proxy.getLatestFrame().getTargetDistance());
//            metric("Object Angle", _proxy.getLatestFrame().getTargetAngle());
//        }
//        _driveTrain.updateDashboard();
        _catapult.updateDashboard();
        //Updates the driver station
//        _autoChooser.updateDashboard();
        //metric("Proxy/Millis", _proxy.getLatestFrame().getMillis());
        _driveTrain.updateDashboard();
    }

    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.controllerPeriodic();
        }
    }
    public void dataPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.dataPeriodic();
        }
    }
}
