package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.config.Auto;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultSetpoint;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultState;

import edu.wpi.first.wpilibj.DriverStation;

/** DriveCatapult is the default command for the catapult.  It runs when no other command
 *  that requires the catapult is scheduled.
 * 
 * <p> Note: Every command that wants to use the catapult must check the state of the intake.
 * If the intake has not been successfully deployed at least once, do not ask the catapult to
 * shoot.  Shooting when intake is in initial stowed state can break the robot.
 */
public class DriveCatapult extends OutliersCommand {

    private final Catapult _catapult;
    private final DriveTrain _driveTrain;
    private final Intake _intake;
    private final OI _oi;

    private Catapult.CatapultState _prevState;
    private boolean _isFirstShot = true;
    private CatapultState _lastLoggedState = null;

    public DriveCatapult(Catapult catapult, Intake intake, DriveTrain driveTrain, OI oi) {
        _catapult = catapult;
        _intake = intake;
        _driveTrain = driveTrain;
        _oi = oi;
        _prevState = _catapult.getState();
        addRequirements(catapult);
    }

    @Override
    public void initialize() {
        info("DriveCatapult initializing.");
        _catapult.setInitialized(true);
    }

    @Override
    public void execute() {
        metric("String from dist", _catapult.calculateIdealRope(_driveTrain.getDistanceToTarget()));
        metric("Spring from dist", _catapult.calculateIdealSpring(_driveTrain.getDistanceToTarget()));
        metric("Intake down", _intake.isIntakeUp());
        metric("Setpoint value", _catapult.getSetpoint().toString());
        CatapultState newState =  _catapult.getState();
        if (newState != _lastLoggedState) {
            info("State changed to " + newState);
            _lastLoggedState = newState;
        }

        // Failsafe kill switch
        checkKill();

        switch (newState) {
            case WAITING:
                if (_oi.exitKill()) {
                    _catapult.setState(_prevState);
                }
                break;
            case READY:
                if (isShootTriggered()) {
                    _catapult.releaseArm();
                }
                break;
            case DEBUG:
                if (_oi.releaseArm()) {
                    _catapult.releaseArm();
                }
                if (_oi.preloadCatapult()) {
                    _catapult.setState(Catapult.CatapultState.PRELOADED);
                }
                if (_oi.exitDebugCatapult()) {
                    _catapult.setState(Catapult.CatapultState.ZEROING_ARM);
                }
                break;
            default:
        }
    }

    boolean isShootTriggered() {
        if (DriverStation.isAutonomous()) {
            return _catapult.isAutoShoot();    
        } else {
            return _oi.isShootButtonPressed();
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    protected void checkLockOut() {
        if (_intake.isIntakeUp()) {
            _prevState = _catapult.getState();
            _catapult.setState(Catapult.CatapultState.LOCK_OUT);
        }
    }

    /** Stop catapult motors on kill button */
    protected void checkKill() {
        if (_oi.kill()) {
            _prevState = _catapult.getState();
            _catapult.stopMotors();
            _catapult.setState(Catapult.CatapultState.WAITING);
        }
    }
}



