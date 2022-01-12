package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;

public class IdleCatapult extends OutliersCommand {
    private Catapult _catapult;

    public IdleCatapult(Catapult catapult) {
        _catapult = catapult;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        super.execute();
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
