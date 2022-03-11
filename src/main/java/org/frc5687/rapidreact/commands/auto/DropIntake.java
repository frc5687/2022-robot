package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Intake;

public class DropIntake extends OutliersCommand{

    private Intake _intake;

    public DropIntake(Intake intake){
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize(){
        super.initialize();
        _intake.deploy();
    }

    @Override
    public boolean isFinished(){
        info("Finishing");
        return !_intake.isIntakeUp();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
