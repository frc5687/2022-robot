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
        info("Stow initialized.");
        _climber.enableMetrics();
    }

    @Override
    public void execute(){
        super.execute();
        if (_climber.isRockArmDown()) {
            _climber.stopRockerArm();
        } else {
            _climber.setRockSpeed(Constants.Climber.ARM_STOW_SPEED);
        }
        if (_climber.isStaArmDown()) {
            _climber.stopStationaryArm();
        } else {
            _climber.setStaSpeed(Constants.Climber.ARM_STOW_SPEED);
        }
    }


    @Override
    public boolean isFinished(){
        if  (_climber.isRockArmDown()) {
            info("Stow finished.");
            return true;
        };
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        info("Stow ended.");
        _climber.stop();
        _climber.zeroRockerArmEncoder();
        _climber.zeroStationaryArmEncoder();
        _climber.disableMetrics();
    }
}
