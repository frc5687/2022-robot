package org.frc5687.rapidreact.commands.auto;

import java.security.DrbgParameters.Reseed;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.DriveTrain;

public class DriveForTime extends OutliersCommand{

    private DriveTrain _driveTrain;
    private long _timeMs;
    private boolean _reversed;
    private long _endMillis;
    private double _pow;
    
    public DriveForTime(DriveTrain driveTrain, long timeMs, boolean reversed){
        _driveTrain = driveTrain;
        _timeMs = timeMs;
        _reversed = reversed;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize(){
        super.initialize();
        _endMillis = System.currentTimeMillis() + _timeMs;
        _pow = (_reversed ? -1.0 : 1.0) * Constants.Auto.DRIVETRAIN_POWER;
    }

    @Override
    public void execute(){
        super.execute();
        if (_endMillis > System.currentTimeMillis()) {
            _driveTrain.drive(_pow, 0.0, 0.0, false);
        } else {
            _driveTrain.drive(0.0, 0.0, 0.0, false);
        }
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return System.currentTimeMillis() > _endMillis;
    }
}
