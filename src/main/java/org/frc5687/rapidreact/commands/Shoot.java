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
            case IDLE:
                _catapult.setState(Catapult.CatapultState.LOWERING_ARM);
            case LOWERING_ARM:
                if (!_catapult.isArmLowered()) {
                    _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                } else {
                    _catapult.setSpringMotorSpeed(0.0);
                    _catapult.setWinchMotorSpeed(0.0);
                    _catapult.lockArm();
                    _catapult.setState(Catapult.CatapultState.LOADING);
                }
            case LOADING:
                // in the future check if we have a ball and the ball color, REV Color Sensor
                // has a proximity sensor built it.
                // get Spring Goal and Winch Goal from an override or distance to goal measurement.
                _catapult.setWinchGoal(Units.degreesToRadians(90));
                _catapult.setSpringGoal(0.2); // meters
                if (_catapult.isWinchAtGoal()) {
                    _catapult.setState(Catapult.CatapultState.AIMING);
                }
            case AIMING:
                // check if we are in the correct position and aiming at the goal.
                _catapult.setState(Catapult.CatapultState.SHOOTING);
            case SHOOTING:
                // call OI button to shoot.
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



