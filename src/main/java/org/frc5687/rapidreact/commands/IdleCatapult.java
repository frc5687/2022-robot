package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.subsystems.Catapult;

public class IdleCatapult extends OutliersCommand {

    private Catapult _catapult;
    private OI _oi;

    public IdleCatapult(Catapult catapult, OI oi) {
        _catapult = catapult;
        _oi = oi;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        double speed = _oi.getWinchMotorSpeed();
        metric("winch speed", speed);
        _catapult.setWinchMotorSpeed(0);
//        double speedSpring = _oi.getSpringMotorSpeed();
//        metric("spring speed", speedSpring);
//        _catapult.setSpringMotorSpeed(0.0);
        if (!_catapult.isSpringHallTriggered()) {
            _catapult.setSpringMotorSpeed(_catapult.getSpringControllerOutput());
        } else {
            _catapult.setSpringMotorSpeed(0.0);
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
