package org.frc5687.rapidreact.commands.catapult;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.config.Auto;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Indexer;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultSetpoint;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultState;

import static org.frc5687.rapidreact.Constants.Catapult.*;
import static org.frc5687.rapidreact.subsystems.Catapult.CatapultState.*;

public class DriveCatapult extends OutliersCommand {

    private final Catapult _catapult;
    private final DriveTrain _driveTrain;
    private final Intake _intake;
    private final Indexer _indexer;
    private final OI _oi;

    private CatapultState _prevState;
    private CatapultState _lastLoggedState = null;

    private double _springGoal;
    private double _winchGoal;

    private boolean _isFirstShot = true;
    private long _wait;
    private long _indexerWait;
    private long _loweringWait;


    public DriveCatapult(Catapult catapult, Intake intake, DriveTrain driveTrain, Indexer indexer, OI oi) {
        _catapult = catapult;
        _indexer = indexer;
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
        metric("Automatic Shooting (Teleop)", _catapult.getAutomatShoot());
        metric("Autonomous Shooting (Autonomous)", _catapult.getAutonomShoot());
        metric("String from dist", _catapult.calculateIdealString(_driveTrain.getDistanceToTarget()));
        metric("Spring from dist", _catapult.calculateIdealSpring(_driveTrain.getDistanceToTarget()));
        metric("Setpoint value", _catapult.getSetpoint().toString());

//        metric("Spring lead dispacement", _catapult.calculateLeadSpringDisplacement(
//                _driveTrain.getTargetPosition(),
//                _driveTrain.getTargetVelocity()
//        ));
//
//        metric("Winch Lead", _catapult.calculateLeadWinchString(
//                _driveTrain.getTargetPosition(),
//                _driveTrain.getTargetVelocity()
//        ));

        CatapultState newState =  _catapult.getState(); 
        if (newState != _lastLoggedState) {
            info("State changed to " + newState);
            _lastLoggedState = newState;
        }

        switch (newState) {
            case ZEROING: {
                checkLockOut();
                checkKill();
                // Try Zeroing the spring first, then the winch.
                if (zeroWinch()) {
                    if (zeroSpring()) {
                        _catapult.setState(LOWERING_ARM);
                    }
                }
            }
            break;
            case LOWERING_ARM: {
                checkLockOut();
                checkKill();
                if (_indexer.isBallDetected()) {
                    _catapult.setWinchMotorSpeed(0.0);
                } else {
                    _catapult.setWinchMotorSpeed(LOWERING_SPEED);
                }
                if (_catapult.isArmLowered()) {
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.lockArm();
                    _loweringWait = System.currentTimeMillis() + LOWERING_DELAY;
                    _catapult.setState(WAIT_LOADING);
                }
            }
            break;
            case WAIT_LOADING: {
                if (System.currentTimeMillis() > _loweringWait) {
                    _catapult.setState(LOADING);
                }
                break;
            }
            case LOADING: {
                _indexer.up();
                checkLockOut();
                checkKill();
                if (_driveTrain.hasTarget() && _catapult.getSetpoint() == CatapultSetpoint.NONE) {
                    _winchGoal = _catapult.calculateIdealString(_driveTrain.getDistanceToTarget());
                    _springGoal = _catapult.calculateIdealSpring(_driveTrain.getDistanceToTarget());
                    _catapult.setWinchGoal(_winchGoal);
                    _catapult.setSpringDistance(_springGoal);
                }
                if (_indexer.isBallDetected()) {
                    _indexer.down();
                    _catapult.setState(AIMING);
                    _indexerWait = System.currentTimeMillis() + Constants.Indexer.NO_BALL_DELAY;
                }
                _catapult.setWinchMotorSpeed(_catapult.getWinchControllerOutput());
            }
            break;
            case AIMING: {
                checkLockOut();
                checkKill();
                if(!_indexer.isBallDetected() && (System.currentTimeMillis() > _indexerWait)){
                    _catapult.setState(LOADING);
                }
                if (_driveTrain.hasTarget() && _catapult.getSetpoint() == CatapultSetpoint.NONE) {
                    _winchGoal = _catapult.calculateIdealString(_driveTrain.getDistanceToTarget());
                    _springGoal = _catapult.calculateIdealSpring(_driveTrain.getDistanceToTarget());
                    _catapult.setWinchGoal(_winchGoal);
                    _catapult.setSpringDistance(_springGoal);
                } else {
                    if (_catapult.getSetpoint() == CatapultSetpoint.NONE) {
                        _catapult.setWinchGoal(_winchGoal);
                        _catapult.setSpringDistance(_springGoal);
                    } else {
                        _catapult.setStaticGoals();
                    }
                }
                if (isShootTriggered() && ((_catapult.isWinchAtGoal() && _catapult.isSpringAtPosition()))) {
                    if (_indexer.isBallDetected() && (System.currentTimeMillis() > _indexerWait)) {
                        _catapult.setAutonomShoot(false);
                        _catapult.setState(SHOOTING);
                    }
                }
//             check if we are in the correct position and aiming at the goal.
              _catapult.setWinchMotorSpeed(_catapult.getWinchControllerOutput());
            }
            break;
            case SHOOTING: {
                checkLockOut();
                checkKill();
                _catapult.setSetpoint(CatapultSetpoint.NONE);
                _catapult.releaseArm();
                _wait = System.currentTimeMillis() + DELAY;
                _catapult.setState(Catapult.CatapultState.WAIT_SHOT);
            } break;
            case WAIT_SHOT: {
                if (System.currentTimeMillis() > _wait) {
                    if (_isFirstShot) {
                        _isFirstShot = false;
                        _catapult.setState(ZEROING);
                    } else {
                        _catapult.setState(LOWERING_ARM);
                    }
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
                if (zeroWinch()) {
                    if (zeroSpring()) {
                        _catapult.setState(DEBUG);
                        _catapult.setWinchMotorSpeed(0);
//                        _catapult.setWinchGoal(Auto.StaticShots.TARMAC_WINCH);
//                        _catapult.setSpringDistance(Auto.StaticShots.TARMAC_SPRING);
//                        _catapult.setWinchMotorSpeed(_catapult.getWinchControllerOutput());
//                        if (_catapult.isWinchAtGoal() && _catapult.isSpringAtPosition()) {
                   }
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
    protected boolean zeroWinch() {
        if (!_catapult.isWinchZeroed()) {
            _catapult.setWinchMotorSpeed(LOWERING_SPEED);
            if (_catapult.isArmLowered()) {
                _catapult.zeroWinchEncoder();
                _catapult.lockArm();
                _catapult.setWinchMotorSpeed(0);
                _catapult.setWinchGoal(0);
            }
        }
        return _catapult.isWinchZeroed();
    }

    protected boolean zeroSpring() {
        if (!_catapult.isSpringZeroed()) {
            _catapult.setSpringMotorSpeed(SPRING_ZERO_SPEED);
            if (_catapult.isSpringHallTriggered()) {
                _catapult.zeroSpringEncoder();
                _catapult.setSpringMotorSpeed(0);
                _catapult.setSpringDistance(0);
            }
        }
        return _catapult.isSpringZeroed();
    }

    boolean isShootTriggered() {
        metric("On target", _driveTrain.onTarget());
        metric("Is drivetrain moving", _driveTrain.isMoving());
        if(!_driveTrain.isMoving() && _driveTrain.onTarget() && _catapult.getAutomatShoot()){
            metric("Can shoot", true);
            return true;
        }
        if (_catapult.getAutonomShoot() && !_driveTrain.isMoving()) {
            return true;
        }
        return _oi.isShootButtonPressed();
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



