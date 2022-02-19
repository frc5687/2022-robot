package org.frc5687.rapidreact.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.HallEffect;
import org.frc5687.rapidreact.util.OutliersContainer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import static org.frc5687.rapidreact.Constants.Intake.ROLLER_INTAKE_SPEED;


public class Intake extends OutliersSubsystem{

    private CANSparkMax _roller;
    private DoubleSolenoid _solenoid;
    private HallEffect _intakeHall;
    
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
        _solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PCH.INTAKE_HIGH, RobotMap.PCH.INTAKE_LOW);
        _intakeHall = new HallEffect(RobotMap.DIO.INTAKE_HALL_EFFECT);
    }

    public void spinDownRoller(){
        //Set the intake roller to idle
        _roller.set(Constants.Intake.ROLLER_IDLE_SPEED);
    }

    public void reversed() {
        _roller.set(-ROLLER_INTAKE_SPEED);
    }

    public void spinUpRoller(){
        _roller.set(ROLLER_INTAKE_SPEED);
    }


    public void stowe(){
        //Stowe the intake
        _solenoid.set(Value.kReverse);
    }

    public void deploy(){
        //Deploy the intake
        _solenoid.set(Value.kForward);
    }

    public boolean isIntakeDown() {
        return _intakeHall.get();
    }

    public boolean isIntakeSolenoidDeployed() {
        return _solenoid.get() == Value.kForward;
    }
    public boolean isIntakeSolenoidStowed() {
        return _solenoid.get() == Value.kReverse;
    }

    @Override
    public void updateDashboard() {
        
    }
}
