package org.frc5687.rapidreact.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Intake;

public class Shoot extends OutliersCommand {

    private final Catapult _catapult;
    private final Intake _intake;
    private final OI _oi;

    private long _time;
    private boolean _shoot;
    private Catapult.CatapultState _prevState;

    public Shoot(Catapult catapult, Intake intake, OI oi) {
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
        metric("Intake down", _intake.isIntakeDown());
        switch (_catapult.getState()) {
            case ZEROING: {
                if(!_intake.isIntakeDown()) {
                    _prevState = _catapult.getState();
                    _catapult.setState(Catapult.CatapultState.LOCK_OUT);
                }
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
                if(!_intake.isIntakeDown()) {
                    _prevState = _catapult.getState();
                    _catapult.setState(Catapult.CatapultState.LOCK_OUT);
                }
                _shoot = false;
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
                if(!_intake.isIntakeDown()) {
                    _prevState = _catapult.getState();
                    _catapult.setState(Catapult.CatapultState.LOCK_OUT);
                }
                // in the future check if we have a ball and the ball color, REV Color Sensor
                // has a proximity sensor built it.
//                if (!correctColor && hasBall) {
//                    _catapult.setState(Catapult.CatapultState.WRONG_BALL);
//                } else {
                    _catapult.setState(Catapult.CatapultState.AIMING);
//                }
            }
            break;
            case AIMING: {
                if(!_intake.isIntakeDown()) {
                    _prevState = _catapult.getState();
                    _catapult.setState(Catapult.CatapultState.LOCK_OUT);
                }
//             check if we are in the correct position and aiming at the goal.
                _catapult.runSpringController();
//            error("Switching state Shooting");
                _catapult.setState(Catapult.CatapultState.SHOOTING);
            }
            break;
            case WRONG_BALL: {
                if(!_intake.isIntakeDown()) {
                    _prevState = _catapult.getState();
                    _catapult.setState(Catapult.CatapultState.LOCK_OUT);
                }
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
                if(!_intake.isIntakeDown()) {
                    _prevState = _catapult.getState();
                    _catapult.setState(Catapult.CatapultState.LOCK_OUT);
                }
                // call OI button to shoot.
                _catapult.setWinchGoal(0.245);
                _catapult.setSpringGoal(0.105);
                _catapult.runSpringController();
                _catapult.runWinchController();
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
                if (_intake.isIntakeDown()) {
                    _catapult.setState(_prevState);
                }
            } break;
            case PRELOAD: {
                if (!_catapult.isSpringHallTriggered() && !_catapult.isSpringZeroed()) {
                    _catapult.setSpringMotorSpeed(Constants.Catapult.SPRING_ZERO_SPEED);
                } else {
                    _catapult.setSpringGoal(0.10);
                    _catapult.runSpringController();
                }
                if (!_catapult.isArmLowered() && !_catapult.isWinchZeroed()) {
                    _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                } else {
                    _catapult.zeroWinchEncoder();
                    _catapult.lockArm();
                    _catapult.setWinchGoal(0.235);
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
}



