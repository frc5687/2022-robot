package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.Climber.ClimberStep;

import edu.wpi.first.wpilibj.DriverStation;

public class Stow extends OutliersCommand{

    private Climber _climber;
    
    public Stow(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void initialize(){
        super.initialize();
        _climber.setStep(ClimberStep.STOW);
        _climber.rockerOut();
        info("Stow initialized.");
        _climber.enableMetrics();
    }

    @Override
    public void execute(){
        super.execute();
        error("Execute");
        if (_climber.isRockArmDown()) {
            _climber.stopRockerArm();
            error("Not Moving");
        } else {
            _climber.setRockSpeed(Constants.Climber.ARM_STOW_SPEED);
            error("Moving");
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
            error("Stow finished");
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
