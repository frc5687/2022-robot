package org.frc5687.rapidreact.commands.catapult;


import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;

public class AutoShoot extends OutliersCommand{

    private Catapult _catapult;
    private DriveTrain _driveTrain;
    private OI _oi;
    private boolean shotComplete;

    public AutoShoot(Catapult catapult, DriveTrain driveTrain, OI oi){
        _catapult = catapult;
        _driveTrain = driveTrain;
        _oi = oi;
        metric("Auto shoot running", false);
    }

    @Override
    public void initialize(){
        super.initialize();
    }

    @Override
    public void execute(){
        super.execute();
        if(_oi.getDriveX() < 0.4 && _oi.getDriveY() < 0.4 && _driveTrain.onTarget() == true){
            metric("Clear to shoot", true);
        }
        metric("Not clear to shoot", false);
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        if(shotComplete){
            return true;
        }
        return false;
    }
}
