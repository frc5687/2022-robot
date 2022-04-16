package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.subsystems.Lights;
import org.frc5687.rapidreact.util.OutliersContainer;

public class ChangeColor extends OutliersCommand{

    private Lights _lights;

    public ChangeColor(Lights lights){
        _lights = lights;
    }

    @Override
    public void execute(){
        super.execute();
        _lights.setColor(Constants.CANdle.GREEN);
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return true;
    }
}
