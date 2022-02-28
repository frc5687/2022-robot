package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultState;

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

    }

    @Override
    public void execute() {
        metric("Intake down", _intake.isIntakeDown());
        metric("String from dist", _catapult.calculateIdealString(_driveTrain.getDistanceToTarget()));
        metric("Spring from dist", _catapult.calculateIdealSpring(_driveTrain.getDistanceToTarget()));
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
                _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                _catapult.runSpringController();
                if (_catapult.isArmLowered() && (Math.abs(_catapult.getWinchStringLength()) < 0.05)) {
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.lockArm();
//                error("Switching state Loading");
                    _catapult.setState(Catapult.CatapultState.LOADING);
                }
            }
            break;
            case LOADING: {
                checkLockOut();
                checkKill();
                _catapult.setState(Catapult.CatapultState.AIMING);
            }
            break;
            case AIMING: {
                checkLockOut();
                checkKill();
                if (_driveTrain.hasTarget()) {
                    _catapult.setWinchGoal(_catapult.calculateIdealString(_driveTrain.getDistanceToTarget()));
                    _catapult.setSpringGoal(_catapult.calculateIdealSpring(_driveTrain.getDistanceToTarget()));
                } else {
                    _catapult.setWinchGoal(0.245);
                    _catapult.setSpringGoal(0.105);
                }
                _catapult.runSpringController();
                _catapult.runWinchController();
                if (_oi.isShootButtonPressed() && _catapult.isWinchAtGoal() && _catapult.isSpringAtPosition()) {
                    _catapult.setState(Catapult.CatapultState.SHOOTING);
                }
            }
            break;
            case WRONG_BALL: {
                checkLockOut();
                checkKill();
                _catapult.setWinchGoal(Constants.Catapult.REMOVE_BALL_WINCH_GOAL);
                _catapult.setSpringGoal(Constants.Catapult.REMOVE_BALL_SPRING_GOAL);
                _catapult.runSpringController();
                _catapult.runWinchController();
                if (_catapult.isWinchAtGoal()) {
                    _catapult.releaseArm();
                    _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
                }
            }
            break;
            case SHOOTING: {
                _catapult.releaseArm();
                _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
            } break;
            case LOCK_OUT: {
                _catapult.setWinchMotorSpeed(0.0);
                _catapult.setSpringMotorSpeed(0.0);
                if (_intake.isIntakeDown()) {
                    _catapult.setState(_prevState);
                }
            } break;
            case PRELOAD: {
                checkLockOut();
                if (!_catapult.isSpringHallTriggered() && !_catapult.isSpringZeroed()) {
                    _catapult.setSpringMotorSpeed(Constants.Catapult.SPRING_ZERO_SPEED);
                } else {
                    _catapult.setSpringGoal(Constants.Catapult.INITIAL_BALL_SPRING_GOAL);
                    _catapult.runSpringController();
                }
                if (!_catapult.isArmLowered() && !_catapult.isWinchZeroed()) {
                    _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                } else {
                    _catapult.zeroWinchEncoder();
                    _catapult.lockArm();
                    _catapult.setWinchGoal(Constants.Catapult.INITIAL_BALL_WINCH_GOAL);
                    _catapult.runWinchController();
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
        if (!_intake.isIntakeDown()) {
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



