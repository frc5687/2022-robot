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

    public Shoot(Catapult catapult, Intake intake, OI oi) {
        _catapult = catapult;
        _intake = intake;
        _oi = oi;
        _shoot = false;
        addRequirements(catapult);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        metric("intake down", _intake.isIntakeDown());
//        if (_intake.isIntakeDown()) {
            switch (_catapult.getState()) {
                case ZEROING: {
//                error("Settings state zeroing");
                    _catapult.setSpringMotorSpeed(Constants.Catapult.SPRING_ZERO_SPEED);
                    _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                    if (_catapult.isSpringHallTriggered()) {
                        _catapult.setSpringMotorSpeed(0.0);
                        _catapult.setSpringGoal(0.0);
                    }
                    if (_catapult.isArmLowered()) {
                        _catapult.setWinchMotorSpeed(0.0);
                        _catapult.setWinchGoal(0.0);
                    }
                    if (_catapult.isArmLowered() && _catapult.isSpringHallTriggered()) {
                        _catapult.setSpringMotorSpeed(0.0);
                        _catapult.setWinchMotorSpeed(0.0);
                        _catapult.setWinchGoal(0.0);
                        _catapult.setSpringGoal(0.0);
//                    error("Switching state Lowering arm");
                        _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
                    }
                }
                break;
                case LOWERING_ARM: {
                    _shoot = false;
                    _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                    _catapult.runSpringController();
                    if (_catapult.isArmLowered() && (Math.abs(_catapult.getWinchStringLength()) < 0.05)) {
                        _catapult.setWinchMotorSpeed(0.0);
                        _catapult.lockArm();
//                    error("Switching state Loading");
                        _catapult.setState(Catapult.CatapultState.LOADING);
                    }
                }
                break;
                case LOADING: {
                    // in the future check if we have a ball and the ball color, REV Color Sensor
                    // has a proximity sensor built it.
//                    if (!correctColor && hasBall) {
//                        _catapult.setState(Catapult.CatapultState.WRONG_BALL);
//                    } else {
                        _catapult.setState(Catapult.CatapultState.AIMING);
//                    }
                }
                break;
                case AIMING: {
//                 check if we are in the correct position and aiming at the goal.
                    _catapult.runSpringController();
//                error("Switching state Shooting");
                    _catapult.setState(Catapult.CatapultState.SHOOTING);
                }
                break;
                case WRONG_BALL: {
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
                case DEBUG: {
                    if (_oi.releaseArm()) {
                        _catapult.releaseArm();
                    }
                    if (_oi.exitDebugCatapult()) {
                        _catapult.setState(Catapult.CatapultState.ZEROING);
                    }
                }
            }
//        } else {
//            _catapult.setSpringMotorSpeed(0.0);
//            _catapult.setWinchMotorSpeed(0.0);
//        }
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



