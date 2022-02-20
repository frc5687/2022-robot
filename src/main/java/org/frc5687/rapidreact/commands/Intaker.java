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
    }

    @Override
    public void execute(){
        super.execute();
        if (_reversed) {
            _intake.reversed();
        } else {
            _intake.spinUpRoller();
        }
    }

    @Override
    public void end(boolean interrupted) {
        _intake.spinDownRoller();
        super.end(interrupted);
    }
}