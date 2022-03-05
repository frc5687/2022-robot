/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.frc5687.rapidreact.commands.Drive;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.commands.SetState;
import org.frc5687.rapidreact.commands.auto.DropIntake;
import org.frc5687.rapidreact.commands.auto.OneBallAuto;
import org.frc5687.rapidreact.commands.auto.Wait;
import org.frc5687.rapidreact.commands.auto.ZeroBallAuto;
import org.frc5687.rapidreact.commands.DriveCatapult;
import org.frc5687.rapidreact.commands.IdleIntake;
import org.frc5687.rapidreact.commands.Climber.IdleClimber;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;

import org.frc5687.rapidreact.util.AutoChooser;
import org.frc5687.rapidreact.util.JetsonProxy;
import org.frc5687.rapidreact.util.OutliersContainer;
import org.frc5687.rapidreact.util.AutoChooser.Mode;
import org.frc5687.rapidreact.util.AutoChooser.Position;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private JetsonProxy _proxy;

    private Robot _robot;
    private DriveTrain _driveTrain;
    private Catapult _catapult;
    private Intake _intake;
    private Climber _climber;
    private boolean _hold;
    private AutoChooser _autoChooser;
    private UsbCamera _cam;


    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        // initialize peripherals. Do this before subsystems.
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        // proxy need to be before drivetrain as drivetrain requires it.
        _proxy = new JetsonProxy(10);

        _catapult = new Catapult(this);
        _driveTrain = new DriveTrain(this, _oi, _proxy, _imu);
        _intake = new Intake(this);
        _climber = new Climber(this);
        _proxy = new JetsonProxy(10);
        _autoChooser = new AutoChooser();

        initializeCamera();
        
        //The robots default command will run so long as another command isn't activated
        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        setDefaultCommand(_intake, new IdleIntake(_intake, _oi));
        setDefaultCommand(_catapult, new DriveCatapult(_catapult, _intake, _oi));
        setDefaultCommand(_climber, new IdleClimber(_climber, _oi));

        // initialize OI after subsystems.
        _oi.initializeButtons(_driveTrain, _catapult, _intake, _climber);
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
        _imu.reset();
    }

    public void periodic() {
    }

    public void disabledPeriodic() {
        //Runs every 20ms during disabled
    }
    @Override
    public void disabledInit() {}

    @Override
    public void teleopInit() {
    }

    @Override
    public void autonomousInit() {
//        _catapult.setState(CatapultState.AUTO);
        _hold = true;
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
        return new SequentialCommandGroup(new DropIntake(_intake), new WaitCommand(0.5), command, new SetState(_catapult, Catapult.CatapultState.ZEROING));
    }

    public Command getAutonomousCommand() {
        // _driveTrain.resetOdometry(Constants.Auto.RobotPositions.THIRD);
        // return new ZeroBallAuto(_driveTrain, Constants.Auto.BallPositions.BALL_TWO, new Rotation2d());
        AutoChooser.Position autoPosition = Position.Fourth;//_autoChooser.getSelectedPosition()
        AutoChooser.Mode autoMode = Mode.ZeroBall;//_autoChooser.getSelectedMode();
        Pose2d[] destinationsZeroBall = { new Pose2d(), new Pose2d(), new Pose2d() };
        Pose2d[] destinationsOneBall = { new Pose2d() };
        Rotation2d[] rotationsZeroBall = { new Rotation2d() };
        Rotation2d[] rotationsOneBall = { new Rotation2d() };
        
        switch(autoPosition) {
            case First:
                _driveTrain.resetOdometry(Constants.Auto.RobotPositions.FIRST);
                destinationsZeroBall[0] = Constants.Auto.BallPositions.BALL_ONE;
                destinationsOneBall[0] = Constants.Auto.BallPositions.BALL_ONE;
                rotationsZeroBall[0] = new Rotation2d();
                rotationsOneBall[0] = new Rotation2d();
                break;
            case Second:
                _driveTrain.resetOdometry(Constants.Auto.RobotPositions.SECOND);
                destinationsZeroBall[0] = Constants.Auto.FieldPositions.ROBOT_POS_TWO_DEST;
                destinationsOneBall[0] = Constants.Auto.FieldPositions.ROBOT_POS_TWO_DEST;
                rotationsZeroBall[0] = new Rotation2d();
                rotationsOneBall[0] = new Rotation2d();
                break;
            case Third:
                _driveTrain.resetOdometry(Constants.Auto.RobotPositions.THIRD);
                destinationsZeroBall[0] = Constants.Auto.BallPositions.BALL_TWO;
                destinationsOneBall[0] = Constants.Auto.BallPositions.BALL_TWO;
                rotationsZeroBall[0] = new Rotation2d();
                rotationsOneBall[0] = new Rotation2d();
                break;
            case Fourth:
                _driveTrain.resetOdometry(Constants.Auto.RobotPositions.FOURTH);
                destinationsZeroBall[0] = Constants.Auto.FieldPositions.PARALLEL_PARK;
                destinationsOneBall[0] = Constants.Auto.FieldPositions.SAFE_BALL_THREE;
                rotationsZeroBall[0] = new Rotation2d();
                rotationsOneBall[0] = Constants.Auto.Rotations.BALL_THREE_FROM_FOURTH;
                break;
            default:
                return new Wait(15);
        }

        switch (autoMode) {
            case ZeroBall:
            return new ZeroBallAuto(_driveTrain, destinationsZeroBall[0], rotationsZeroBall[0]);
            case OneBall:
            return new OneBallAuto(_driveTrain, _catapult, destinationsOneBall[0], rotationsOneBall[0]);
            default:
                return new Wait(15);
        }

    //     switch(autoPosition) {
    //         case First:
    //             _driveTrain.resetOdometry(Constants.Auto.RobotPositions.FIRST);
    //             switch(autoMode) {
    //                 case ZeroBall:
    //                     return new ZeroBallAuto(_driveTrain, Constants.Auto.BallPositions.BALL_ONE, new Rotation2d());
    //                 case OneBall:
    //                     return new OneBallAuto(_driveTrain, Constants.Auto.BallPositions.BALL_ONE, new Rotation2d());
    //             }
    //         case Second:
    //             _driveTrain.resetOdometry(Constants.Auto.RobotPositions.SECOND);
    //             switch(autoMode) {
    //                 case ZeroBall:
    //                     return new ZeroBallAuto(_driveTrain, Constants.Auto.FieldPositions.ROBOT_POS_TWO_DEST, new Rotation2d());
    //                 case OneBall:
    //                     return new OneBallAuto(_driveTrain, Constants.Auto.FieldPositions.ROBOT_POS_TWO_DEST, new Rotation2d());
    //             }
    //         case Third:
    //             _driveTrain.resetOdometry(Constants.Auto.RobotPositions.THIRD);
    //             switch(autoMode) {
    //                 case ZeroBall:
    //                     return new ZeroBallAuto(_driveTrain, Constants.Auto.BallPositions.BALL_TWO, new Rotation2d());
    //                 case OneBall:
    //             }       return new OneBallAuto(_driveTrain, Constants.Auto.BallPositions.BALL_TWO, new Rotation2d());
    //         case Fourth:
    //             _driveTrain.resetOdometry(Constants.Auto.RobotPositions.FOURTH);
    //             switch(autoMode) {
    //                 case ZeroBall:
    //                     return new ZeroBallAuto(_driveTrain, Constants.Auto.FieldPositions.SAFE_BALL_THREE, new Rotation2d());
    //                 case OneBall:
    //                     return new OneBallAuto(_driveTrain, Constants.Auto.FieldPositions.SAFE_BALL_THREE, new Rotation2d());
    //             }
    //         default:
    //             info("Couldn't find position switch, waiting.");
    //             return new Wait(15);
                
    //     }
    }

    @Override
    public void updateDashboard() {
        //Updates the driver station
        _autoChooser.updateDashboard();
        _driveTrain.updateDashboard();
        //metric("Proxy/Millis", _proxy.getLatestFrame().getMillis());
//        _driveTrain.updateDashboard();
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
