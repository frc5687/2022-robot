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
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Lights;
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

    private Lights _lights;
    private AutoChooser _autoChooser;
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
//        info("Running RobotContainer.init()");

        // Initialize starting position and mode to unknown
        // Later we will poll Drive Station for values
        autoPosition = AutoChooser.Position.Unknown;
        autoMode = AutoChooser.Mode.Unknown;
//
//         Display starting position value
        SmartDashboard.putString("DB/String 0", "Starting Position:");
        SmartDashboard.putString("DB/String 5", "Unknown");
   
        // Display auto mode value
        SmartDashboard.putString("DB/String 1", "Auto Mode:");
        String _automodeString = SmartDashboard.getString("Auto Selector", "Unknown");
        SmartDashboard.putString("DB/String 6", _automodeString);

        // Auto mode chooser
        String [] modes = { "Zero Ball", "One Ball", "Two Ball", "Three Ball", "Four Ball", "Five Ball" };
        SmartDashboard.putStringArray("Auto List", modes);

        _oi = new OI();
        //Config the NavX
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _proxy = new JetsonProxy(10);
        _limelight = new Limelight("limelight");
        _lights = new Lights(this, RobotMap.PWM.BLINKENS);
        // then subsystems
        _driveTrain = new DriveTrain(this, _oi, _proxy, _limelight, _imu);
        _intake = new Intake(this, _lights);
        _climber = new Climber(this, _driveTrain, _lights);
        _catapult = new Catapult(this, _lights);
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

    public void periodic() {

        // Poll Drive Station for starting position and auto mode selector

        // This could move to disabledPeriodic if we don't want to be doing
        // this during auto, teleop or test mode.

        // Check starting position buttons
        Boolean _positionOne = SmartDashboard.getBoolean("DB/Button 0", false);
        Boolean _positionTwo = SmartDashboard.getBoolean("DB/Button 1", false);
        Boolean _positionThree = SmartDashboard.getBoolean("DB/Button 2", false);
        Boolean _positionFour = SmartDashboard.getBoolean("DB/Button 3", false);

        // Set position to highest value that is selected
        autoPosition = AutoChooser.Position.Unknown;
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
        String _automode = SmartDashboard.getString("Auto Selector", "Unknown");
        switch(_automode) {
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
            case "Three Ball":
            case "Four Ball":
            case "Five Ball":
            default:
                autoMode = AutoChooser.Mode.Unknown;
        }

        // Display auto mode selector
        String _automodeString = SmartDashboard.getString("Auto Selector", "Unknown");
        SmartDashboard.putString("DB/String 6", _automodeString);
    }

    public void disabledPeriodic() {
        //Runs every 20ms during disabled
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
        info("Running RobotContainer.autonomousInit()");
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
        Boolean _bypass = true;

        // Set _bypass to true to set autonomous command here instead of using Drive Station
        if (_bypass) {
            _driveTrain.resetOdometry(Auto.RobotPositions.THIRD);
            Rotation2d rot = Rotation2d.fromDegrees(40);
            info("Running twoball.");
            return new TwoBallAuto(_driveTrain, _catapult, _intake, Auto.BallPositions.BALL_TWO, rot);
        }

        // Return command sequence based on starting position and auto mode selectded

        Pose2d[] destinationsZeroBall = { new Pose2d() };
        Pose2d[] destinationsOneBall = { new Pose2d() };
        Pose2d[] destinationsTwoBall = { new Pose2d() };

        Rotation2d[] rotationsZeroBall = { new Rotation2d() };
        Rotation2d[] rotationsOneBall = { new Rotation2d() };
        Rotation2d[] rotationsTwoBall = { new Rotation2d() };
        metric("autoPose", autoPosition.toString());
        switch(autoPosition) {
            case First:
                _driveTrain.resetOdometry(Auto.RobotPositions.FIRST);
                destinationsZeroBall[0] = Auto.BallPositions.BALL_ONE;
                destinationsOneBall[0] = Auto.BallPositions.BALL_ONE;
                destinationsTwoBall[0] = Auto.BallPositions.BALL_ONE;
                rotationsZeroBall[0] = new Rotation2d();
                rotationsOneBall[0] = new Rotation2d();
                rotationsTwoBall[0] = new Rotation2d();
                break;
            case Second:
                _driveTrain.resetOdometry(Auto.RobotPositions.SECOND);
                destinationsZeroBall[0] = Auto.FieldPositions.ROBOT_POS_TWO_DEST;
                destinationsOneBall[0] = Auto.FieldPositions.ROBOT_POS_TWO_DEST;
                destinationsTwoBall[0] = Auto.BallPositions.BALL_ONE;
                rotationsZeroBall[0] = new Rotation2d();
                rotationsOneBall[0] = new Rotation2d();
                rotationsTwoBall[0] = new Rotation2d();
                break;
            case Third:
                _driveTrain.resetOdometry(Auto.RobotPositions.THIRD);
                destinationsZeroBall[0] = Auto.BallPositions.BALL_TWO;
                destinationsOneBall[0] = Auto.BallPositions.BALL_TWO;
                destinationsTwoBall[0] = Auto.BallPositions.BALL_TWO;
                rotationsZeroBall[0] = new Rotation2d();
                rotationsOneBall[0] = new Rotation2d();
                rotationsTwoBall[0] = new Rotation2d();
                break;
            case Fourth:
                _driveTrain.resetOdometry(Auto.RobotPositions.FOURTH);
                destinationsZeroBall[0] = Auto.FieldPositions.PARALLEL_PARK;
                destinationsOneBall[0] = Auto.FieldPositions.SAFE_BALL_THREE;
                destinationsTwoBall[0] = Auto.FieldPositions.SAFE_BALL_THREE;
                rotationsZeroBall[0] = new Rotation2d();
                rotationsOneBall[0] = Auto.Rotations.BALL_THREE_FROM_FOURTH;
                rotationsTwoBall[0] = new Rotation2d();
                break;
            default:
                return new Wait(15);
        }

        switch (autoMode) {
            case ZeroBall:
                return new ZeroBallAuto(_driveTrain, destinationsZeroBall[0], rotationsZeroBall[0]);
            case OneBall:
                return new OneBallAuto(_driveTrain, _catapult, destinationsOneBall[0], rotationsOneBall[0]);
            case TwoBall:
                return new TwoBallAuto(_driveTrain, _catapult, _intake, destinationsTwoBall[0], rotationsTwoBall[0]);
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
