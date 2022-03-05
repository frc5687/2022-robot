package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultState;

public class DriveCatapult extends OutliersCommand {

    private final Catapult _catapult;
    private final Intake _intake;
    private final OI _oi;

    private long _time;
    private boolean _shoot;
    private Catapult.CatapultState _prevState;

    public DriveCatapult(Catapult catapult, Intake intake, OI oi) {
        _catapult = catapult;
        _intake = intake;
        _oi = oi;
        _shoot = false;
        _prevState = _catapult.getState();
        addRequirements(catapult);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        metric("Intake down", _intake.isIntakeUp());
        switch (_catapult.getState()) {
            case ZEROING: {
                checkLockOut();
                checkKill();
                _catapult.setSpringMotorSpeed(Constants.Catapult.SPRING_ZERO_SPEED);
                _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                if (_catapult.isSpringHallTriggered()) {
                    _catapult.setSpringMotorSpeed(0.0);
                    _catapult.setSpringGoal(0.0);
                }
                if (_catapult.isArmLowered()) {
                    _catapult.zeroWinchEncoder();
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.setWinchGoal(0.0);
                }
                if (_catapult.isArmLowered() && _catapult.isSpringHallTriggered()) {
                    _catapult.setSpringMotorSpeed(0.0);
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.setWinchGoal(0.0);
                    _catapult.setSpringGoal(0.0);
                    _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
                }
            }
            break;
            case LOWERING_ARM: {
                checkLockOut();
                checkKill();
                _shoot = false;
                _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                _catapult.setSpringMotorSpeed(_catapult.getSpringControllerOutput());
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
//             check if we are in the correct position and aiming at the goal.
              _catapult.setSpringMotorSpeed(_catapult.getSpringControllerOutput());
//            error("Switching state Shooting");
              _catapult.setState(Catapult.CatapultState.SHOOTING);
            }
            break;
            case WRONG_BALL: {
                checkLockOut();
                checkKill();
                _catapult.setWinchGoal(Constants.Catapult.REMOVE_BALL_WINCH_GOAL);
                _catapult.setSpringGoal(Constants.Catapult.REMOVE_BALL_SPRING_GOAL);
                _catapult.setSpringMotorSpeed(_catapult.getSpringControllerOutput());
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
                // call OI button to shoot.
                _catapult.setWinchGoal(0.245);
                _catapult.setSpringGoal(0.105);
                _catapult.setSpringMotorSpeed(_catapult.getSpringControllerOutput());
                _catapult.setWinchMotorSpeed(_catapult.getWinchControllerOutput());
                if (_oi.isShootButtonPressed()) {
                    _shoot = true;
                }
                if (_shoot && _catapult.isWinchAtGoal() && _catapult.isSpringAtPosition()) {
                    _catapult.releaseArm();
                    _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
                    _shoot = false;
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
                    _catapult.setSpringMotorSpeed(Constants.Catapult.SPRING_ZERO_SPEED);
                } else {
                    _catapult.setSpringGoal(Constants.Catapult.INITIAL_BALL_SPRING_GOAL);
                    _catapult.setSpringMotorSpeed(_catapult.getSpringControllerOutput());
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



