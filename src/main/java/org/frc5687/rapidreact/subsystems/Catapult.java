package org.frc5687.rapidreact.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.ADXL345_SPI;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.OutliersContainer;

public class Catapult extends OutliersSubsystem {

    private CANSparkMax _springMotor;
    private CANSparkMax _winchMotor;

    private RelativeEncoder _springEncoder;
    private RelativeEncoder _winchEncoder;

    private ProfiledPIDController _springController;

    public Catapult(OutliersContainer container) {
        super(container);

        // Create Motor controllers.
        _springMotor = new CANSparkMax(RobotMap.CAN.SPARKMAX.SPRING_BABY_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _winchMotor = new CANSparkMax(RobotMap.CAN.SPARKMAX.WINCH_BABY_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);

        // setup controllers
        _springMotor.restoreFactoryDefaults();
        _winchMotor.restoreFactoryDefaults();
        _springMotor.setInverted(Constants.Catapult.SPRING_MOTOR_INVERTED);
        _winchMotor.setInverted(Constants.Catapult.WINCH_MOTOR_INVERTED);

        // setup encoder.
        _springEncoder = _springMotor.getEncoder();
        _winchEncoder = _winchMotor.getEncoder();

        _springController = new ProfiledPIDController(
                Constants.Catapult.kP,
                Constants.Catapult.kI,
                Constants.Catapult.kD,
                new TrapezoidProfile.Constraints(
                        Constants.Catapult.MAX_SPRING_VELOCITY,
                        Constants.Catapult.MAX_SPRING_ACCELERATION
                )
        );
        _springController.setTolerance(Constants.Catapult.SPRING_TOLERANCE);
    }


    @Override
    public void periodic() {
        super.periodic();
    }

    public void setSpringMotorSpeed(double speed) {
        _springMotor.set(speed);
    }

    public void setWinchMotorSpeed(double speed) {
        _winchMotor.set(speed);
    }

    public double getSpringEncoderRotations() {
        return _springEncoder.getPosition();
    }

    // meters
    public double getSpringRailPosition() {
        return (getSpringEncoderRotations() / Constants.Catapult.GEAR_REDUCTION_SPRING)
                * Constants.Catapult.ROTATIONS_TO_POSITION;
    }

    public void setSpringPosition(double position) {
        setSpringMotorSpeed(_springController.calculate(getSpringRailPosition(), position));
    }

    public boolean isSpringAtPosition() {
        return _springController.atGoal();
    }


    @Override
    public void updateDashboard() {
        metric("Spring Encoder Rail Rotations", getSpringRailPosition());
    }

}
