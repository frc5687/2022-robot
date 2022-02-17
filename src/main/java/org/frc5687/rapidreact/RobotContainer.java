/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.rapidreact.commands.Drive;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.util.JetsonProxy;
import org.frc5687.rapidreact.util.OutliersContainer;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private JetsonProxy _proxy;

    private Robot _robot;
    private DriveTrain _driveTrain;


    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        _oi = new OI();
        //Config the NavX
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _proxy = new JetsonProxy(10);
        //_driveTrain = new DriveTrain(this, _oi, _imu);
        //The robots default command will run so long as another command isn't activated
        //setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        //_robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
        _robot.addPeriodic(this::dataPeriodic, 0.05, 0.01);
        _imu.reset();
        //_oi.initializeButtons(_driveTrain);
    }

    public void periodic() {
        //Runs every 20ms
    }

    public void disabledPeriodic() {
        _proxy.setData(
                new JetsonProxy.Data(1),
                new JetsonProxy.Data(System.currentTimeMillis()),
                new JetsonProxy.Data(10),
                new JetsonProxy.Data(5),
                new JetsonProxy.Data(0));
        //Runs every 20ms during disabled
    }

    @Override
    public void disabledInit() {
        //Runs once during disabled
    }

    @Override
    public void teleopInit() {
        //Runs at the start of teleop
    }

    @Override
    public void autonomousInit() {
        //This is where autos go
        //Runs once during auto
    }

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
            metric("Has goal", _proxy.getLatestFrame().hasGoal());
            metric("Object Distance", _proxy.getLatestFrame().getGoalDistance());
            metric("Object Angle", _proxy.getLatestFrame().getGoalAngle());
        }
        //Updates the driver station
        //_driveTrain.updateDashboard();
        //metric("Proxy/Millis", _proxy.getLatestFrame().getMillis());
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
