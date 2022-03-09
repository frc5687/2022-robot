/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.rapidreact.commands.Drive;
import org.frc5687.rapidreact.commands.DriveCatapult;
import org.frc5687.rapidreact.commands.IdleIntake;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.commands.Climber.IdleClimber;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Lights;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.util.JetsonProxy;
import org.frc5687.rapidreact.util.OutliersContainer;

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
    private UsbCamera _cam;
    private Lights _lights;


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
        _lights = new Lights(this, 0);

        _catapult = new Catapult(this);
        _driveTrain = new DriveTrain(this, _oi, _proxy, _imu);
        _intake = new Intake(this, _lights);
        _climber = new Climber(this);
        initializeCamera();
        
        // The robots default command will run so long as another command isn't activated
        // setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        // setDefaultCommand(_intake, new IdleIntake(_intake, _oi));
        // setDefaultCommand(_catapult, new DriveCatapult(_catapult, _intake, _oi));
        // setDefaultCommand(_climber, new IdleClimber(_climber, _oi));

        // initialize OI after subsystems.
        // _oi.initializeButtons(_driveTrain, _catapult, _intake, _climber);
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
        _imu.reset();
    }

    public void periodic() {
    }

    public void disabledPeriodic() {
        //Runs every 20ms during disabled
    }

    @Override
    public void disabledInit() {
    }

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
        _lights.base();
        //Updates the driver station
        //_driveTrain.updateDashboard();
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
