/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.rapidreact.commands.*;
import org.frc5687.rapidreact.commands.Autos.DriveForTime;
import org.frc5687.rapidreact.commands.Autos.DropIntake;
import org.frc5687.rapidreact.commands.Autos.OneBallAuto;
import org.frc5687.rapidreact.commands.Autos.Wait;
import org.frc5687.rapidreact.commands.Autos.ZeroBallAuto;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.util.AutoChooser;
import org.frc5687.rapidreact.util.JetsonProxy;
import org.frc5687.rapidreact.util.OutliersContainer;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private JetsonProxy _proxy;

    private Robot _robot;
    private Catapult _catapult;
    private Intake _intake;
    private DriveTrain _driveTrain;
    private boolean _hold;
    private AutoChooser _autoChooser;


    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        // initialize peripherals. Do this before subsystems.
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _catapult = new Catapult(this);
        _driveTrain = new DriveTrain(this, _oi, _proxy, _imu);
        _intake = new Intake(this);
        _proxy = new JetsonProxy(10);
        _autoChooser = new AutoChooser();

        //The robots default command will run so long as another command isn't activated
        setDefaultCommand(_catapult, new Shoot(_catapult, _intake, _oi));
        setDefaultCommand(_intake, new IdleIntake(_intake, _oi));
        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));

        // initialize OI after subsystems.
        _oi.initializeButtons(_driveTrain, _catapult, _intake);
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

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    public Command wrapCommand(Command command) {
        return new SequentialCommandGroup(new DropIntake(_intake), command);
    }

    public Command getAutonomousCommand() {
        //_driveTrain.resetOdometry(Constants.Auto.RobotPositions.THIRD);
        //return new ZeroBallAuto(_driveTrain, Constants.Auto.BallPositions.BALL_TWO, new Rotation2d());
        AutoChooser.Position autoPosition = _autoChooser.getSelectedPosition();
        AutoChooser.Mode autoMode = _autoChooser.getSelectedMode();
        Pose2d[] destinationsZeroBall = {};
        Pose2d[] destinationsOneBall = {};
        
        switch(autoPosition) {
            case First:
                _driveTrain.resetOdometry(Constants.Auto.RobotPositions.FIRST);
                destinationsZeroBall[0] = Constants.Auto.BallPositions.BALL_ONE;
                break;
            case Second:
                _driveTrain.resetOdometry(Constants.Auto.RobotPositions.SECOND);
                destinationsZeroBall[0] = Constants.Auto.FieldPositions.ROBOT_POS_TWO_DEST;
                break;
            case Third:
                _driveTrain.resetOdometry(Constants.Auto.RobotPositions.THIRD);
                destinationsZeroBall[0] = Constants.Auto.BallPositions.BALL_TWO;
                break;
            case Fourth:
                _driveTrain.resetOdometry(Constants.Auto.RobotPositions.FOURTH);
                destinationsZeroBall[0] = Constants.Auto.FieldPositions.SAFE_BALL_THREE;
                break;
            default:
                return new Wait(15);
        }

        switch (autoMode) {
            case ZeroBall:
                return new ZeroBallAuto(_driveTrain, destinationsZeroBall[0], new Rotation2d());
            case OneBall:
                return new Wait(15);
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
        //TODO: uncomment this, or don't...
        // if (_proxy.getLatestFrame() != null) {
        //     metric("Millis", _proxy.getLatestFrame().getMillis());
        //     metric("Has goal", _proxy.getLatestFrame().hasTarget());
        //     metric("Object Distance", _proxy.getLatestFrame().getTargetDistance());
        //     metric("Object Angle", _proxy.getLatestFrame().getTargetAngle());
        // }
        //Updates the driver station
        _driveTrain.updateDashboard();
        //metric("Proxy/Millis", _proxy.getLatestFrame().getMillis());
        //_catapult.updateDashboard();
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
