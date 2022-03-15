package org.frc5687.rapidreact.commands.Catapult;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.config.Auto;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultSetpoint;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultState;

import static org.frc5687.rapidreact.Constants.Catapult.*;
import static org.frc5687.rapidreact.subsystems.Catapult.CatapultState.*;

public class DriveCatapult extends OutliersCommand {

    private final Catapult _catapult;
    private final DriveTrain _driveTrain;
    private final Intake _intake;
    private final OI _oi;

    private CatapultState _prevState;
    private CatapultState _lastLoggedState = null;
    private boolean _isFirstShot = true;
    private long _wait;

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
        metric("String from dist", _catapult.calculateIdealString(_driveTrain.getDistanceToTarget()));
        metric("Spring from dist", _catapult.calculateIdealSpring(_driveTrain.getDistanceToTarget()));
        metric("Intake down", _intake.isIntakeUp());
        metric("Setpoint value", _catapult.getSetpoint().toString());
        CatapultState newState =  _catapult.getState(); 
        if (newState != _lastLoggedState) {
            info("State changed to " + newState);
            _lastLoggedState = newState;
        } 
        switch (newState) {
            case ZEROING: {
                checkLockOut();
                checkKill();
                _catapult.setSpringMotorSpeed(SPRING_ZERO_SPEED);
                _catapult.setWinchMotorSpeed(LOWERING_SPEED);
                if (_catapult.isSpringHallTriggered()) {
                    _catapult.setSpringMotorSpeed(0.0);
                    _catapult.zeroSpringEncoder();
                }
                if (_catapult.isArmLowered()) {
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.zeroWinchEncoder();
                }
                if ((_catapult.isArmLowered() && _catapult.isWinchZeroed()) && (_catapult.isSpringHallTriggered() && _catapult.isSpringZeroed())) {
                    _catapult.setSpringMotorSpeed(0.0);
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.setWinchGoal(0.0);
                    _catapult.setSpringDistance(0.0);
                    _catapult.setState(LOWERING_ARM);
                }
            }
            break;
            case LOWERING_ARM: {
                checkLockOut();
                checkKill();
                _catapult.setWinchMotorSpeed(LOWERING_SPEED);
                if (_catapult.isArmLowered()) {
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.lockArm();
                    _catapult.setState(LOADING);
                }
            }
            break;
            case LOADING: {
                _catapult.raiseGate();
                checkLockOut();
                checkKill();
                if (_catapult.isBallDetected()) {
                    _catapult.lowerGate();
                    _catapult.setState(AIMING);
                }

            }
            break;
            case AIMING: {
                checkLockOut();
                checkKill();
                if (_driveTrain.hasTarget() && _catapult.getSetpoint() == CatapultSetpoint.NONE) {
                    _catapult.setWinchGoal(_catapult.calculateIdealString(_driveTrain.getDistanceToTarget()));
                    _catapult.setSpringDistance(_catapult.calculateIdealSpring(_driveTrain.getDistanceToTarget()));
                } else {
                    _catapult.setStaticGoals();
                }
//                _catapult.setStaticGoals();
                if (isShootTriggered() && (_isFirstShot ||  (_catapult.isWinchAtGoal() && _catapult.isSpringAtPosition()))) {
                    _catapult.setAutoshoot(false);
                    _catapult.setState(SHOOTING);
                }
//             check if we are in the correct position and aiming at the goal.
              _catapult.setWinchMotorSpeed(_catapult.getWinchControllerOutput());
            }
            break;
            case WRONG_BALL: {
                checkLockOut();
                checkKill();
                _catapult.setWinchGoal(REMOVE_BALL_WINCH_GOAL);
                _catapult.setSpringDistance(REMOVE_BALL_SPRING_GOAL);
                _catapult.setWinchMotorSpeed(_catapult.getWinchControllerOutput());
                if (_catapult.isWinchAtGoal()) {
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.releaseArm();
                    _wait = System.currentTimeMillis() + DELAY;
                    _catapult.setState(CatapultState.WAIT_SHOT);
                }
            }
            break;
            case SHOOTING: {
                checkLockOut();
                checkKill();
                _catapult.setSetpoint(CatapultSetpoint.NONE);
                _catapult.releaseArm();
                if (_isFirstShot) {
                    _catapult.setState(CatapultState.ZEROING);
                    _isFirstShot = false;
                } else {
                    _wait = System.currentTimeMillis() + DELAY;
                    _catapult.setState(Catapult.CatapultState.WAIT_SHOT);
                }
            } break;
            case WAIT_SHOT: {
                if (System.currentTimeMillis() > _wait) {
                    _catapult.setState(LOWERING_ARM);
                }
            } break;
            case LOCK_OUT: {
                _catapult.setWinchMotorSpeed(0.0);
                _catapult.setSpringMotorSpeed(0.0);
                if (!_intake.isIntakeUp()) {
                    _catapult.setState(_prevState);
                }
            } break;
            case PRELOAD: {
                checkLockOut();
                if (!_catapult.isSpringHallTriggered() && !_catapult.isSpringZeroed()) {
                    _catapult.setSpringMotorSpeed(SPRING_ZERO_SPEED);
                } else if (_catapult.isSpringHallTriggered()) {
                    _catapult.zeroSpringEncoder();
                    _catapult.setSpringDistance(Auto.StaticShots.TARMAC_SPRING);
                }
                if (!_catapult.isArmLowered() && !_catapult.isWinchZeroed()) {
                    _catapult.setWinchMotorSpeed(LOWERING_SPEED);
                } else if (_catapult.isArmLowered()) {
                    _catapult.zeroWinchEncoder();
                    _catapult.lockArm();
                }
                if ((_catapult.isArmLowered() && _catapult.isWinchZeroed()) && _catapult.isSpringZeroed()){
                    _catapult.setWinchGoal(Auto.StaticShots.TARMAC_WINCH);
                }
                if (_catapult.isWinchZeroed()){
                    _catapult.setWinchMotorSpeed(_catapult.getWinchControllerOutput());
                }
                if ((_catapult.isSpringAtPosition() && _catapult.isSpringZeroed()) && (_catapult.isWinchAtGoal() && _catapult.isWinchZeroed())) {
                    _catapult.setWinchMotorSpeed(0);
                    _catapult.setState(DEBUG);
                }
            } break;
            case DEBUG: {
                if (_oi.releaseArm()) {
                    _catapult.releaseArm();
                }
                if (_oi.preloadCatapult()) {
                    _catapult.setState(PRELOAD);
                }
                if (_oi.exitDebugCatapult()) {
                    _catapult.setState(ZEROING);
                }
            } break;
            case KILL: {
                _catapult.setSpringMotorSpeed(0);
                _catapult.setWinchMotorSpeed(0);
                if (_oi.exitKill()) {
                    _catapult.setState(_prevState);
                }
            }break;
            case AUTO:
                _catapult.setSpringMotorSpeed(0.0);
                _catapult.setWinchMotorSpeed(0.0);
                if(!_catapult.isArmLowered()){
                    _catapult.setState(ZEROING);
                }
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
            _catapult.setState(LOCK_OUT);
        }
    }
    protected void checkKill() {
        if (_oi.kill()) {
            _prevState = _catapult.getState();
            _catapult.setState(KILL);
        }
    }
}



