package org.frc5687.rapidreact.commands.Autos;

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
        return _intake.isIntakeDown();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
