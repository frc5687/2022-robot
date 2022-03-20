package org.frc5687.rapidreact.commands;

//import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.subsystems.Catapult;

public class SetState extends OutliersCommand {
    private Catapult _catapult;
    private Catapult.CatapultState _state;
    public SetState(Catapult catapult, Catapult.CatapultState state) {
        _catapult = catapult;
        _state = state;
    }

    @Override
    public void initialize() {
        _catapult.setState(_state);
        info("SetState initialized.");
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);        
    }

    @Override
    public boolean isFinished() {
        info("SetState finished.");
        return true;
    }

}
