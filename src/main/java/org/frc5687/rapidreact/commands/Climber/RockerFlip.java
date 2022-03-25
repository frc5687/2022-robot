package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

public class RockerFlip extends OutliersCommand{
    private Climber _climber;
    public RockerFlip(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void initialize() {
        super.initialize();
        _climber.rockerFlip();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
