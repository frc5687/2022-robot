/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.rapidreact.commands.*;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.util.OutliersContainer;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private Robot _robot;
    private Catapult _catapult;
    private Intake _intake;
    private DriveTrain _driveTrain;

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
