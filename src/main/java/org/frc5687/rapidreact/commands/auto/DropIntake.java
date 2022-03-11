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
        info("DropIntake initialized");
        _intake.deploy();
    }

    @Override
    public boolean isFinished(){
        if (!_intake.isIntakeUp()) {
            info("DropIntake finished");
            return true;
        }
        info("Intake is still up.");
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
