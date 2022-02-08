package org.frc5687.rapidreact.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.OutliersContainer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Intake extends OutliersSubsystem{

    private CANSparkMax _roller;
    private DoubleSolenoid _solenoid;
    
    public Intake(OutliersContainer container) {
        //Construct the roller and solenoids
        super(container);
        _roller = new CANSparkMax(RobotMap.CAN.SPARKMAX.INTAKE_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _roller.restoreFactoryDefaults();
        _roller.setInverted(Constants.Intake.INVERTED);
        _roller.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _roller.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        _roller.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 1000);
        _roller.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 1000);
        _solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PCM.INTAKE_HIGH, RobotMap.PCM.INTAKE_LOW);
    }

    private void spinDownRoller(){
        //Set the intake roller to idle
        _roller.set(Constants.Intake.ROLLER_IDLE_SPEED);
    }

    private void spinUpRoller(){
        _roller.set(Constants.Intake.ROLLER_INTAKE_SPEED);
    }

    public void stowe(){
        //Stowe the intake
        _solenoid.set(Value.kReverse);
    }

    public void deploy(){
        //Deploy the intake
        _solenoid.set(Value.kForward);
        spinUpRoller();
    }

    @Override
    public void updateDashboard() {
        
    }
}
