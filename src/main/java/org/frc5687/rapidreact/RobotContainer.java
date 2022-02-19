/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.rapidreact.commands.*;
import org.frc5687.rapidreact.commands.Autos.DropIntake;
import org.frc5687.rapidreact.commands.Autos.OneBall;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.subsystems.ServoStop;
import org.frc5687.rapidreact.util.OutliersContainer;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private Robot _robot;
    private Catapult _catapult;
    private Intake _intake;
    private DriveTrain _driveTrain;
    private ServoStop _servoStop;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        // initialize peripherals. Do this before subsystems.
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _catapult = new Catapult(this);
        _driveTrain = new DriveTrain(this, _oi, _imu);
        _intake = new Intake(this);
        _servoStop = new ServoStop();

        //The robots default command will run so long as another command isn't activated
        setDefaultCommand(_catapult, new Shoot(_catapult, _intake, _oi));
        setDefaultCommand(_intake, new IdleIntake(_intake, _oi));
//        setDefaultCommand(_catapult, new IdleCatapult(_catapult, _oi));
        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));

        // initialize OI after subsystems.
        _oi.initializeButtons(_driveTrain, _catapult, _intake);
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
        _imu.reset();
    }

    public void periodic() {}

    public void disabledPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void autonomousInit() {}

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
        return wrapCommand(new OneBall(_driveTrain, _catapult, _intake, _oi));
        //        return null;
    }

    private Command wrapCommand(Command command) {
        return new SequentialCommandGroup(new DropIntake(_intake), command);
    }

    @Override
    public void updateDashboard() {
        _driveTrain.updateDashboard();
        _catapult.updateDashboard();
    }

    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.controllerPeriodic();
        }
    }
}
