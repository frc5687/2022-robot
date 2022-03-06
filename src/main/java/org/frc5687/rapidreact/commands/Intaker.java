package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Intake;

public class Intaker extends OutliersCommand {

    private Intake _intake;
    private boolean _reversed;

    public Intaker(Intake intake, boolean reversed){
        _intake = intake;
        _reversed = reversed;
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
        _intake.spinDownRoller();
        _intake.stowe();
        super.end(interrupted);
    }
}