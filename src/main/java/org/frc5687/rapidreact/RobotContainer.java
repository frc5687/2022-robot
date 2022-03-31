/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.frc5687.rapidreact.commands.*;
import org.frc5687.rapidreact.commands.catapult.AutoShoot;
import org.frc5687.rapidreact.commands.catapult.DriveCatapult;
import org.frc5687.rapidreact.commands.auto.*;
import org.frc5687.rapidreact.commands.Climber.IdleClimber;

import org.frc5687.rapidreact.config.Auto;

import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Indexer;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;

import org.frc5687.rapidreact.util.*;

import java.util.Arrays;
import java.util.List;

public class RobotContainer extends OutliersContainer {
    
    private OI _oi;
    private AHRS _imu;
    private JetsonProxy _proxy;
//    private Limelight _limelight;

    private Robot _robot;
    private DriveTrain _driveTrain;
    private Catapult _catapult;
    private Intake _intake;
    private Climber _climber;
    private Indexer _indexer;

    private AutoChooser _autoChooser;

    AutoChooser.Position _autoPosition;
    AutoChooser.Mode _autoMode;

    private UsbCamera _cam;

    public RobotContainer(Robot robot, IdentityMode identityMode, boolean isLogging) {
        super(identityMode, isLogging);
        _robot = robot;
    }

    /** Run once when robot code starts. */
    public void init() {
        // initialize peripherals. Do this before subsystems.
        // info("Running RobotContainer.init()");

        // Initialize starting position and mode to unknown
        // In disabledPeriodic() we will poll Drive Station for values
        _autoPosition = AutoChooser.Position.Unknown;
        _autoMode = AutoChooser.Mode.Unknown;

        // initialize these peripherals first as subsystems require them.
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _proxy = new JetsonProxy(10);
        _autoChooser = new AutoChooser();
//        _limelight = new Limelight("limelight");
        _indexer = new Indexer(this);

        // then subsystems
        _driveTrain = new DriveTrain(this, _oi, _proxy,/*, _limelight, */_imu);
        _intake = new Intake(this);
        _climber = new Climber(this, _driveTrain);
        _catapult = new Catapult(this);

        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        setDefaultCommand(_intake, new IdleIntake(_intake, _oi));
        setDefaultCommand(_catapult, new DriveCatapult(_catapult, _intake, _driveTrain, _indexer, _oi));
        setDefaultCommand(_climber, new IdleClimber(_climber, _oi));

        // initialize OI after subsystems.
        _oi.initializeButtons(_driveTrain, _catapult, _intake, _climber, _indexer);

        // Run periodic for each swerve module faster than regular cycle time
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
//        _limelight.disableLEDs();
        _imu.reset();
        _driveTrain.startModules();
    }

    public void periodic() {}

    public void disabledPeriodic() {
        // update the auto chooser for more values.
//        metric("xIn", _oi.getDriveX());
//        metric("yIn", _oi.getDriveY());
        _autoChooser.updateChooser();
        // set the values from the auto chooser.
        _autoMode = _autoChooser.getAutoMode();
        _autoPosition = _autoChooser.getAutoPosition();
    }

    @Override
    public void disabledInit() {
        //Runs once during disabled
    }

    @Override
    public void teleopInit() {
//        _driveTrain.startModules();
    }

    @Override
    public void autonomousInit() {
        _driveTrain.startModules();
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
        return new SequentialCommandGroup(new DropIntake(_intake), new WaitCommand(0.4), command);
    }

    public Command getAutonomousCommand() {
        // Return command sequence based on auto mode selected and pass in starting position
        switch (_autoMode) {
            case ZeroBall:
                return new ZeroBallAuto(_driveTrain, _autoPosition);
            case OneBall:
                return new OneBallAuto(_driveTrain, _catapult, _indexer, _autoPosition);
            case TwoBall:
                return new TwoBallAuto(_driveTrain, _catapult, _intake,  _indexer, _autoPosition);
            case ThreeBall:
                return new ThreeBallAuto(_driveTrain, _catapult, _intake, _indexer, _autoPosition);
            case FourBall:
                return new FourBallAuto(_driveTrain, _catapult, _intake, _indexer, _autoPosition);
            default:
                return new Wait(15);
        }
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
