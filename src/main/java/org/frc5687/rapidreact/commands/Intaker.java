package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Intake;

public class Intaker extends OutliersCommand {

    Intake _intake;

    public Intaker(Intake intake){
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        super.execute();
        _intake.spinUpRoller();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.spinDownRoller();
    }
}