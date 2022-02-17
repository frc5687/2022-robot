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


    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        // initialize peripherals. Do this before subsystems.
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _proxy = new JetsonProxy(10);
//        _catapult = new Catapult(this);
        _driveTrain = new DriveTrain(this, _oi, _proxy, _imu);
        _intake = new Intake(this);
        //The robots default command will run so long as another command isn't activated
//        setDefaultCommand(_catapult, new Shoot(_catapult, _intake, _oi));
        setDefaultCommand(_intake, new IdleIntake(_intake));
        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));

        // initialize OI after subsystems.
        _oi.initializeButtons(_driveTrain, _catapult, _intake);
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
        _imu.reset();
    }

    public void periodic() {}

    public void disabledPeriodic() {
        _proxy.setData(
                new JetsonProxy.Data(0),
//                new JetsonProxy.Data(System.currentTimeMillis()),
                new JetsonProxy.Data(10),
                new JetsonProxy.Data(5),
                new JetsonProxy.Data(0));
        //Runs every 20ms during disabled
    }
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
//        _proxy.setData(
//                new JetsonProxy.Data(0),
//                new JetsonProxy.Data(System.currentTimeMillis()),
//                new JetsonProxy.Data(10),
//                new JetsonProxy.Data(5),
//                new JetsonProxy.Data(0));
        if (_proxy.getLatestFrame() != null) {
            metric("Millis", _proxy.getLatestFrame().getMillis());
            metric("Object Distance", _proxy.getLatestFrame().getGoalDistance());
        }
        //Updates the driver station
        //_driveTrain.updateDashboard();
        //metric("Proxy/Millis", _proxy.getLatestFrame().getMillis());
        _driveTrain.updateDashboard();
//        _catapult.updateDashboard();
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
