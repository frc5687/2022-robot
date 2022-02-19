package org.frc5687.rapidreact.commands.Autos;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Catapult;

public class ReleaseArm extends OutliersCommand{

    private Catapult _catapult;

    public ReleaseArm(Catapult catapult){
        _catapult = catapult;
        addRequirements(_catapult);
    }

    @Override
    public void initialize(){
        super.initialize();
        _catapult.releaseArm();
    }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
