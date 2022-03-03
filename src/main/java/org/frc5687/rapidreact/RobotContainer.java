/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frc5687.rapidreact.commands.Drive;
import org.frc5687.rapidreact.commands.IdleIntake;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.commands.Shoot;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.util.JetsonProxy;
import org.frc5687.rapidreact.util.OutliersContainer;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private JetsonProxy _proxy;

    private Robot _robot;
    private Catapult _catapult;
    private Intake _intake;
    private Climber _climber;
    private DriveTrain _driveTrain;
    private boolean _hold;


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
        _climber = new Climber(this);
        _proxy = new JetsonProxy(10);

        //The robots default command will run so long as another command isn't activated
        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));

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

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    public Command getAutonomousCommand() {
        //        return new StealBallAuto(
        //                _driveTrain, _shooter, _hood, _intake, _spindexer, _stealTenPrt1,
        // _stealExit, _oi);
        error("Start auto");
//        return null;
        // return wrapCommand(new OneBall(_driveTrain, _catapult, _intake, _oi));
               return null;
    }

    private Command wrapCommand(Command command) {
        // return new SequentialCommandGroup(new DropIntake(_intake), command);
        return null;
    }

    @Override
    public void updateDashboard() {
        if (_proxy.getLatestFrame() != null) {
            metric("Millis", _proxy.getLatestFrame().getMillis());
            metric("Has goal", _proxy.getLatestFrame().hasTarget());
            metric("Object Distance", _proxy.getLatestFrame().getTargetDistance());
            metric("Object Angle", _proxy.getLatestFrame().getTargetAngle());
        }
        //Updates the driver station
        //_driveTrain.updateDashboard();
        //metric("Proxy/Millis", _proxy.getLatestFrame().getMillis());
//        _driveTrain.updateDashboard();
        _catapult.updateDashboard();
        _climber.updateDashboard();
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
