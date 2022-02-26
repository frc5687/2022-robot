package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

public class Stow extends OutliersCommand{

    private Climber _climber;
    
    public Stow(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void initialize(){
        super.initialize();
        _climber.rockerOut();
    }

    @Override
    public void execute(){
        super.execute();
        if (_climber.isRockArmDown()) {
            _climber.setRockSpeed(0.0);
        } else {
            _climber.setRockSpeed(Constants.Climber.ARM_STOW_SPEED);
        }
        if (!_climber.isStaArmDown()) {
            _climber.setStaSpeed(Constants.Climber.ARM_STOW_SPEED);
        }
    }


    @Override
    public boolean isFinished(){
        return _climber.isStaArmDown() && _climber.isRockArmDown();
    }

    @Override
    public void end(boolean interrupted) {
        _climber.stop();
        _climber.zeroRockerArmEncoder();
        _climber.zeroStationaryArmEncoder();
    }
}
