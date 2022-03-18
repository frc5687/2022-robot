package org.frc5687.rapidreact.commands.catapult;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Catapult;

public class SetSetpoint extends OutliersCommand {
    private Catapult _catapult;
    private Catapult.CatapultSetpoint _setpoint;

    public SetSetpoint(Catapult catapult, Catapult.CatapultSetpoint setpoint) {
        _catapult = catapult;
        _setpoint = setpoint;
    }

    @Override
    public void initialize() {
        _catapult.setSetpoint(_setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);        
    }

    @Override
    public boolean isFinished() {
        info("SetSetpoint finished.");
        return true;
    }

}
