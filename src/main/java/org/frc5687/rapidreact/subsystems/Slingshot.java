package org.frc5687.rapidreact.subsystems;

import java.lang.invoke.ConstantCallSite;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.HallEffect;
import org.frc5687.rapidreact.util.OutliersContainer;

import edu.wpi.first.wpilibj.Servo;

public class Slingshot extends OutliersSubsystem {
    private TalonFX _tensionMotor;
    private Servo _releaseServo;

    private HallEffect _primedSensor;
    

    public Slingshot(OutliersContainer container) {
        super(container);

        _tensionMotor = new TalonFX(RobotMap.CAN.TALONFX.SLINGSHOT_TENSIONER);
        _releaseServo = new Servo(RobotMap.PWM.SLINGSHOT_RELEASE);

        _primedSensor = new HallEffect(RobotMap.DIO.SLINGSHOT_PRIMED);
    }



    @Override
    public void updateDashboard() {
        // TODO Auto-generated method stub
        
    }



    public void runTensioner() {
        _tensionMotor.set(ControlMode.PercentOutput, Constants.Slingshot.TENSIONER_SPEED);
    }

    public void stopTensioner() {
        _tensionMotor.set(ControlMode.PercentOutput, 0);
    }



    public boolean isPrimed() {
        return _primedSensor.get();
    }


    
}
