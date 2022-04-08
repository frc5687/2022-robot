/* Team 5687 (C)2022 */
package org.frc5687.rapidreact.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.HallEffect;
import org.frc5687.rapidreact.util.OutliersContainer;

public class Intake extends OutliersSubsystem {

    private TalonFX _roller;
    private DoubleSolenoid _solenoid;
    private boolean _deployed;
    private HallEffect _intakeHall;

    public Intake(OutliersContainer container) {
        super(container);
        _deployed = false;
        _roller = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_ROLLER);
        _solenoid =
                new DoubleSolenoid(
                        PneumaticsModuleType.REVPH,
                        RobotMap.PCH.INTAKE_HIGH,
                        RobotMap.PCH.INTAKE_LOW);
        _intakeHall = new HallEffect(RobotMap.DIO.INTAKE_HALL_EFFECT);
    }

    /** Spin down the intake */
    public void spinDownRoller() {
        _roller.set(TalonFXControlMode.PercentOutput, Constants.Intake.ROLLER_IDLE_SPEED);
    }

    /** Spin up the intake */
    public void spinUpRoller() {
        _roller.set(TalonFXControlMode.PercentOutput, Constants.Intake.THE_BEANS);
    }

    /** Stowe the intake */
    public void stowe() {
        _solenoid.set(Value.kReverse);
        _deployed = false;
    }

    /** Deploy the intake */
    public void deploy() {
        _solenoid.set(Value.kForward);
        _deployed = true;
    }

    /**
     * Get the velocity of the roller
     *
     * @return
     */
    public double getVelocity() {
        return _roller.getSelectedSensorVelocity();
    }

    /**
     * Get the RPM of the roller
     *
     * @return
     */
    public double getRPM() {
        return getVelocity()
                / Constants.Intake.TICKS_TO_ROTATIONS
                * 600
                * Constants.Intake.GEAR_RATIO;
    }

    public boolean isIntakeUp() {
        return _intakeHall.get();
    }

    @Override
    public void updateDashboard() {
        //        metric("RPM", getRPM());
        //        metric("Intake deployed", _deployed);
    }
}
