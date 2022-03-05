package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

public class IdleClimber extends OutliersCommand {
    private Climber _climber;
    private OI _oi;
    public IdleClimber(Climber climber, OI oi) {
        _climber = climber;
        _oi = oi;
        addRequirements(_climber);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        _climber.setStaSpeed(_oi.getStationarySpeed());
        _climber.setRockSpeed(_oi.getRockerSpeed());
        if (_climber.isRockArmDown()) {
            _climber.zeroRockerArmEncoder();
        }
        if (_climber.isStaArmDown()) {
            _climber.zeroStationaryArmEncoder();
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

