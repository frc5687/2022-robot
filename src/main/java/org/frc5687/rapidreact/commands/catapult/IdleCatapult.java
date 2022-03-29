package org.frc5687.rapidreact.commands.catapult;

import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.commands.OutliersCommand;
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
        _catapult.setWinchMotorSpeed(0);
        _catapult.setSpringMotorSpeed(_oi.getSpringMotorSpeed());
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
