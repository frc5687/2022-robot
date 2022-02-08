package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Intake;

import edu.wpi.first.wpilibj.DriverStation;

public class Intaker extends OutliersCommand {

    Intake _intake;

    public Intaker(Intake intake){
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void execute(){
        super.execute();
        _intake.deploy();
    }

    @Override
    public void initialize(){
        _intake.deploy();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.stowe();
    }
}