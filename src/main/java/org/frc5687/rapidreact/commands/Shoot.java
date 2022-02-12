package org.frc5687.rapidreact.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.subsystems.Catapult;

public class Shoot extends OutliersCommand {

    private final Catapult _catapult;
    private final OI _oi;

    public Shoot(Catapult catapult, OI oi) {
        _catapult = catapult;
        _oi = oi;
        addRequirements(catapult);
    }

    @Override
    public void initialize() {
        _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
    }

    @Override
    public void execute() {
        switch(_catapult.getState()) {
            case ZEROING:
                _catapult.setSpringMotorSpeed(Constants.Catapult.SPRING_ZERO_SPEED);
                _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                if (_catapult.isSpringHallTriggered()) {
                    _catapult.setSpringMotorSpeed(0.0);
                    _catapult.setSpringGoal(0.0);
                }
                if (_catapult.isArmLowered()) {
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.setWinchGoal(0.0);
                    _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
                }
            case LOWERING_ARM:
                _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                _catapult.runSpringController();
                if (_catapult.isArmLowered()) {
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.lockArm();
                    _catapult.setState(Catapult.CatapultState.LOADING);
                }
            case LOADING:
                // in the future check if we have a ball and the ball color, REV Color Sensor
                // has a proximity sensor built it.
                // get Spring Goal and Winch Goal from an override or distance to goal measurement.
                _catapult.setWinchGoal(0.2);
                _catapult.setSpringGoal(0.1); // meters
                _catapult.runSpringController();
                _catapult.runWinchController();
                if (_catapult.isWinchAtGoal()) {
                    _catapult.setState(Catapult.CatapultState.AIMING);
                }
            case AIMING:
                // check if we are in the correct position and aiming at the goal.
                _catapult.runSpringController();
//                _catapult.runWinchController();
                _catapult.setState(Catapult.CatapultState.SHOOTING);
            case SHOOTING:
                // call OI button to shoot.
                _catapult.runSpringController();
//                _catapult.runWinchController();
                if (_oi.isShootButtonPressed()) {
                    _catapult.releaseArm();
                    _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
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
}



