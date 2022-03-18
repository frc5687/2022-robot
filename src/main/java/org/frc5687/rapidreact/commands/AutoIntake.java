package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Intake;

public class AutoIntake extends OutliersCommand {

    private Intake _intake;

    public AutoIntake(Intake intake){
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize(){
        _intake.deploy();
    }

    @Override
    public void execute(){
        _intake.spinUpRoller();
    }

    @Override
    public void end(boolean interrupted) {
//        _intake.spinDownRoller();
        _intake.stowe();
        super.end(interrupted);
    }
}