package org.frc5687.rapidreact.subsystems;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.HallEffect;
import org.frc5687.rapidreact.util.OutliersContainer;
import org.frc5687.rapidreact.util.ProximitySensor;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Intake extends OutliersSubsystem{

    private TalonFX _roller;
    private DoubleSolenoid _solenoid;
    private boolean _deployed;
    private HallEffect _intakeHall;
    private Indexer _indexer;
    private ProximitySensor _sensor;
    private Lights _lights;
    
    public Intake(OutliersContainer container, Indexer indexer, Lights lights) {
        super(container);
        _deployed = false;
        _roller = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_ROLLER);
        _solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PCH.INTAKE_HIGH, RobotMap.PCH.INTAKE_LOW);
        _intakeHall = new HallEffect(RobotMap.DIO.INTAKE_HALL_EFFECT);
        _indexer = indexer;
        _sensor = new ProximitySensor(RobotMap.DIO.INTAKE_PROXIMITY_SENSOR);
        _lights = lights;
    }

    @Override
    public void periodic(){
        if(isBallInCradle()){
            if(isBallInItake()){
                _lights.setGreen();
            }else{
                _lights.setBlue();
            }
        }else if(isBallInItake()){
            _lights.setYellow();
        } else {
            _lights.rainbow();
        }
    }

    /**
     * Spin down the intake
     */
    public void spinDownRoller(){
        _roller.set(TalonFXControlMode.PercentOutput, Constants.Intake.ROLLER_IDLE_SPEED);
    }

    /**
     * Spin up the intake
     */
    public void spinUpRoller(){
        _roller.set(TalonFXControlMode.PercentOutput, Constants.Intake.THE_BEANS);
    }

    /**
     * Spin up the intake at slower sleep
     */
    public void spinRollerSlow(){
        _roller.set(TalonFXControlMode.PercentOutput, Constants.Intake.ROLLER_SLOW_SPEED);
    }

    /**
     * Check for ball in intake
     * @return is there a ball in the intake
     */
    public boolean isBallInCradle(){
        return _indexer.isBallDetected();
    }
    /**
     * Stowe the intake
     */
    public void stowe(){
        _solenoid.set(Value.kReverse);
        _deployed = false;
        _lights.setWhite();
    }

    /**
     * Deploy the intake
     */
    public void deploy(){
        _solenoid.set(Value.kForward);
        _deployed = true;
    }

    /**
     * Returns true if there is a ball in the intake
     * @return returns boolean
     */
    public boolean isBallInItake(){
        return _sensor.get();
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

    public boolean isIntakeUp() {
        return _intakeHall.get();
    }

    @Override
    public void updateDashboard() {
        metric("RPM", getRPM());
        metric("Intake deployed", _deployed);
        metric("Ball in intake", isBallInItake());
        metric("Ball in cradle", _indexer.isBallDetected());
    }
}