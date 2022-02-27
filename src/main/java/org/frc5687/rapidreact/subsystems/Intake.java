package org.frc5687.rapidreact.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.OutliersContainer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Intake extends OutliersSubsystem{

    private TalonFX _roller;
    private DoubleSolenoid _solenoid;
    private boolean _deployed;
    
    public Intake(OutliersContainer container) {
        //Construct the roller and solenoids
        super(container);
        _deployed = false;
        _roller = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_ROLLER);
        _solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PCM.INTAKE_HIGH, RobotMap.PCM.INTAKE_LOW);
    }

    /**
     * Spin down the intake
     */
    private void spinDownRoller(){
        _roller.set(TalonFXControlMode.PercentOutput, Constants.Intake.ROLLER_IDLE_SPEED);
    }

    /**
     * Spin up the intake
     */
    private void spinUpRoller(){
        _roller.set(TalonFXControlMode.PercentOutput, Constants.Intake.THE_BEANS);
    }

    /**
     * Stowe the intake
     */
    public void stowe(){
        _solenoid.set(Value.kReverse);
        spinDownRoller();
        _deployed = false;
    }

    /**
     * Deploy the intake
     */
    public void deploy(){
        _solenoid.set(Value.kForward);
        spinUpRoller();
        _deployed = true;
    }

    /**
     * Get the velocity of the roller
     * @return
     */
    public double getVelocity(){
        return _roller.getSelectedSensorVelocity();
    }

    /**
     * Get the RPM of the roller
     * @return
     */
    public double getRPM(){
        return getVelocity() / Constants.Intake.TICKS_TO_ROTATIONS * 600 * Constants.Intake.GEAR_RATIO;
    }

    @Override
    public void updateDashboard() {
        metric("RPM", getRPM());
        metric("Intake deployed", _deployed);
    }
}
