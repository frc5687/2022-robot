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
    public void initialize() {
        _catapult.setState(Catapult.CatapultState.ZEROING);
    }

    @Override
    public void execute() {
//        metric("intake down", _intake.isIntakeDown());
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
//                    _catapult.setWinchGoal(0.0);
                    }
                    if (_catapult.isArmLowered() && _catapult.isSpringHallTriggered()) {
                        _catapult.setSpringMotorSpeed(0.0);
                        _catapult.setWinchMotorSpeed(0.0);
//                    _catapult.setWinchGoal(0.0);
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
                    // get Spring Goal and Winch Goal from an override or distance to goal measurement.
                    _catapult.setWinchGoal(0.25);
                    _catapult.setSpringGoal(0.11); // meters
                    _catapult.runSpringController();
                    _catapult.runWinchController();
                    if (_catapult.isWinchAtGoal()) {
//                    error("Switching state Aiming");
                        _catapult.setState(Catapult.CatapultState.AIMING);
                    }
                }
                break;
                case AIMING: {
//                 check if we are in the correct position and aiming at the goal.
                    _catapult.runSpringController();
                    _catapult.runWinchController();
//                error("Switching state Shooting");
                    _catapult.setState(Catapult.CatapultState.SHOOTING);
                }
                break;
                case SHOOTING: {
                    // call OI button to shoot.
                    _catapult.runSpringController();
                    _catapult.runWinchController();
                    if (_oi.isShootButtonPressed()) {
                        _shoot = true;
                    }
                    if (_shoot) {
                        _catapult.releaseArm();
                        _catapult.setState(Catapult.CatapultState.DELAY);
                        _time = System.currentTimeMillis() + Constants.Catapult.DELAY;
                        _shoot = false;
                    }
                } break;
                case DELAY: {
                    if (Math.abs(_time - System.currentTimeMillis()) < 0) {
                        _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
                    }
                } break;
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



