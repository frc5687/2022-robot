/* Team 5687 (C)5687-2022 */
package org.frc5687.rapidreact.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.Robot;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.OutliersContainer;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Climber extends OutliersSubsystem{
    
    private CANSparkMax _stationaryArm;
    private CANSparkMax _rockerArm;
    private DoubleSolenoid _rocker;

    public Climber(OutliersContainer container) {
        super(container);
        _stationaryArm = new CANSparkMax(RobotMap.CAN.SPARKMAX.STATIONARY_CLIMBER, CANSparkMax.MotorType.kBrushless);
        _rockerArm = new CANSparkMax(RobotMap.CAN.SPARKMAX.ROCKER_CLIMBER, CANSparkMax.MotorType.kBrushless);
        _rocker = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.PCM.CLIMBER_IN, RobotMap.PCM.CLIMBER_OUT);

        _stationaryArm.setIdleMode(IdleMode.kCoast);
        _stationaryArm.setInverted(false);
    }

    public void raiseSationaryArm(){
        //Raise right arm
        _stationaryArm.set(0.1);
    }

    public void rock(){
        //Use the rocker
        _rocker.set(Value.kForward);
    }

    @Override
    public void updateDashboard() {
        metric("Stationary climber motor encoder", _stationaryArm.getEncoder().toString());
        metric("Stationary climber motor temp", _stationaryArm.getMotorTemperature());
    }
}