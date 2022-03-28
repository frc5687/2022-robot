package org.frc5687.rapidreact.commands.catapult;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;

import static org.frc5687.rapidreact.Constants.DriveTrain.VISION_TOLERANCE;

public class AutoShoot extends OutliersCommand {
    private Catapult _catapult;
    private DriveTrain _driveTrain;

    public AutoShoot(Catapult catapult, DriveTrain driveTrain) {
        _catapult = catapult;
        _driveTrain = driveTrain;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (_driveTrain.hasTarget()) {
//            _catapult.setWinchGoal(); // TODO: distance calc from linear regression
//            _catapult.setSpringGoal(); // TODO: distance calc from linear regression
        }
    }

    @Override
    public boolean isFinished() {
        if (_catapult.isWinchAtGoal() &&
                _catapult.isSpringAtPosition() &&
                Math.abs(_driveTrain.getAngleToTarget()) < VISION_TOLERANCE
        ) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
