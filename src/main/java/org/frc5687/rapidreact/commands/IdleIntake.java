package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.subsystems.Intake;

public class IdleIntake extends OutliersCommand{
    
    private final Intake _intake;
    private final OI _oi;
    private long _waitTime;

    public IdleIntake(Intake intake, OI oi){
        _intake = intake;
        _oi = oi;
        addRequirements(_intake);
    }

    @Override
    public void initialize(){
        super.initialize();
        _waitTime = System.currentTimeMillis() + Constants.Intake.ROLLER_DELAY;
    }

    @Override
    public void execute(){
        super.execute();
        if (System.currentTimeMillis() > _waitTime) {
            _intake.spinDownRoller();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
