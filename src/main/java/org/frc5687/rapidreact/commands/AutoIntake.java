package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Intake;

public class AutoIntake extends OutliersCommand {

    private Intake _intake;
    private Catapult _catapult;

    public AutoIntake(Intake intake, Catapult catapult){
        _intake = intake;
        _catapult = catapult;
        addRequirements(_intake);
    }

    @Override
    public void initialize(){
        _intake.deploy();
    }

    @Override
    public void execute(){
        if(_intake.isBallInCradle()) {
            if (_intake.isBallInItake()) {
                _intake.spinDownRoller();
            } else {
                _intake.spinRollerSlow();
            }
        } else if (_catapult.isArmLowered()) {
            _intake.spinUpRoller();
        }
    }

    @Override
    public void end(boolean interrupted) {
//        _intake.spinDownRoller();
        _intake.stowe();
        super.end(interrupted);
    }
}