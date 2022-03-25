package org.frc5687.rapidreact.subsystems;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.util.ColorSensor;
import org.frc5687.rapidreact.util.OutliersContainer;
import org.frc5687.rapidreact.util.ProximitySensor;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;

public class Indexer extends OutliersSubsystem{

    private DoubleSolenoid _indexerArm;
    private ColorSensor _colorSensor;
    private ProximitySensor _proximitySensor;
    
    public Indexer(OutliersContainer container){
        super(container);
        _indexerArm = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PCH.INDEXER_IN, RobotMap.PCH.INDEXER_OUT);
        _colorSensor = new ColorSensor(Port.kMXP);
        _proximitySensor = new ProximitySensor(RobotMap.DIO.PROXIMITY_SENSOR);
    }

    
    public boolean isDown(){
        return _indexerArm.get() == Value.kReverse;
    }

    /**
     * Is the arm up
     * @return if true is up
     */
    public boolean isUp(){
        return _indexerArm.get() == Value.kForward;
    }

    /**
     * Set the indexer down
     */
    public void down(){
        _indexerArm.set(Value.kForward);
    }

    /**
     * Set the indexer up
     */
    public void up(){
        _indexerArm.set(Value.kReverse);
    }

    public boolean isBallDetected() {
        return _proximitySensor.get();
    }

    public boolean isBlueBallDetected() {
        return _colorSensor.isBlue() && isBallDetected();
    }

    public boolean isRedBallDetected() {
        return _colorSensor.isRed() && isBallDetected();
    }

    @Override
    public void updateDashboard() {
        metric("Prox dected", isBallDetected());
    }
}
