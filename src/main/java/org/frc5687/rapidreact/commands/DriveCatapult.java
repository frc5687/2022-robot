package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultState;

import edu.wpi.first.wpilibj.DriverStation;

public class DriveCatapult extends OutliersCommand {

    private final Catapult _catapult;
    private final DriveTrain _driveTrain;
    private final Intake _intake;
    private final OI _oi;

    private Catapult.CatapultState _prevState;

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
        switch (_catapult.getState()) {
            case ZEROING: {
                checkLockOut();
                checkKill();
                _catapult.setSpringMotorSpeed(Constants.Catapult.SPRING_ZERO_SPEED);
                _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                if (_catapult.isSpringHallTriggered()) {
                    _catapult.setSpringMotorSpeed(0.0);
                    _catapult.zeroSpringEncoder();
                    _catapult.setSpringDistance(0);
                }
                if (_catapult.isArmLowered()) {
                    _catapult.zeroWinchEncoder();
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.setWinchGoal(0.0);
                }
                if (_catapult.isArmLowered() && _catapult.isSpringHallTriggered()) {
                    _catapult.zeroSpringEncoder();
                    _catapult.zeroWinchEncoder();
                    _catapult.setSpringMotorSpeed(0.0);
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.setWinchGoal(0.0);
                    _catapult.setSpringDistance(0.0);
                    _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
                }
            }
            break;
            case LOWERING_ARM: {
                checkLockOut();
                checkKill();
                _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                if (_catapult.isArmLowered() && (Math.abs(_catapult.getWinchStringLength()) < 0.05)) {
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.lockArm();
//                error("Switching state Loading");
                    _catapult.setState(Catapult.CatapultState.LOADING);
                }
            }
            break;
            case LOADING: {
                _catapult.raiseGate();
                checkLockOut();
                checkKill();
                // in the future check if we have a ball and the ball color, REV Color Sensor
                // has a proximity sensor built it.
                if (_catapult.isRedAlliance() && _catapult.isRedBallDetected()) {
                    _catapult.setState(Catapult.CatapultState.AIMING);
                    _catapult.lowerGate();
                } else if (_catapult.isRedAlliance() && _catapult.isBlueBallDetected()) {
                    _catapult.setState(Catapult.CatapultState.WRONG_BALL);
                    _catapult.lowerGate();
                }
                if (!_catapult.isRedAlliance() && _catapult.isBlueBallDetected()) {
                    _catapult.setState(Catapult.CatapultState.AIMING);
                    _catapult.lowerGate();
                } else if (!_catapult.isRedAlliance() && _catapult.isRedBallDetected()) {
                    _catapult.setState(Catapult.CatapultState.WRONG_BALL);
                    _catapult.lowerGate();
                }
            }
            break;
            case AIMING: {
                checkLockOut();
                checkKill();
                if (_driveTrain.hasTarget()) {
                    _catapult.setWinchGoal(_catapult.calculateIdealString(_driveTrain.getDistanceToTarget()));
                    _catapult.setSpringDistance(_catapult.calculateIdealSpring(_driveTrain.getDistanceToTarget()));
                } else {
                    _catapult.setWinchGoal(0.325);
                    _catapult.setSpringDistance(0.0835);
                }
                if (isShootTriggered() && _catapult.isWinchAtGoal() && _catapult.isSpringAtPosition()) {
                    _catapult.setAutoshoot(false);
                    _catapult.setState(Catapult.CatapultState.SHOOTING);
                }
//             check if we are in the correct position and aiming at the goal.
              _catapult.setWinchMotorSpeed(_catapult.getWinchControllerOutput());
            }
            break;
            case WRONG_BALL: {
                checkLockOut();
                checkKill();
                _catapult.setWinchGoal(Constants.Catapult.REMOVE_BALL_WINCH_GOAL);
                _catapult.setSpringDistance(Constants.Catapult.REMOVE_BALL_SPRING_GOAL);
                _catapult.setWinchMotorSpeed(_catapult.getWinchControllerOutput());
                if (_catapult.isWinchAtGoal()) {
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.releaseArm();
                    _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
                }
            }
            break;
            case SHOOTING: {
                checkLockOut();
                checkKill();
                _catapult.releaseArm();
                _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
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
                    _catapult.setSpringMotorSpeed(Constants.Catapult.SPRING_ZERO_SPEED);
                } else {
                    _catapult.zeroSpringEncoder();
                    _catapult.setSpringDistance(Constants.Catapult.INITIAL_BALL_SPRING_GOAL);
                }
                if (!_catapult.isArmLowered() && !_catapult.isWinchZeroed()) {
                    _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                } else {
                    _catapult.zeroWinchEncoder();
                    _catapult.lockArm();
                    _catapult.setWinchGoal(Constants.Catapult.INITIAL_BALL_WINCH_GOAL);
                    _catapult.setWinchMotorSpeed(_catapult.getWinchControllerOutput());
                }
                if (_catapult.isSpringAtPosition() && _catapult.isWinchAtGoal() && _catapult.isWinchZeroed()) {
                    _catapult.setWinchMotorSpeed(0);
                    _catapult.setSpringMotorSpeed(0);
                    _catapult.setState(Catapult.CatapultState.DEBUG);
                }
            } break;
            case DEBUG: {
                if (_oi.releaseArm()) {
                    _catapult.releaseArm();
                }
                if (_oi.preloadCatapult()) {
                    _catapult.setState(Catapult.CatapultState.PRELOAD);
                }
                if (_oi.exitDebugCatapult()) {
                    _catapult.setState(Catapult.CatapultState.ZEROING);
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
                    _catapult.setState(CatapultState.ZEROING);
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
            _catapult.setState(Catapult.CatapultState.LOCK_OUT);
        }
    }
    protected void checkKill() {
        if (_oi.kill()) {
            _prevState = _catapult.getState();
            _catapult.setState(Catapult.CatapultState.KILL);
        }
    }
}



