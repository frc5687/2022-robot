package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.subsystems.Intake;

public class IdleIntake extends OutliersCommand{
    
    private final Intake _intake;
    private final OI _oi;

    public IdleIntake(Intake intake, OI oi){
        _intake = intake;
        _oi = oi;
        addRequirements(_intake);
    }

    @Override
    public void initialize(){
        super.initialize();
        _intake.deploy();
    }

    @Override
    public void execute(){
        super.execute();
        // if (_oi.intakeDeployRetract() && _intake.isIntake()) {
        //     _intake.stowe();
        // } else if (_oi.intakeDeployRetract() && _intake.isIntakeSolenoidStowed()) {
        //     _intake.deploy();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
