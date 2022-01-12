package org.frc5687.rapidreact.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.Helpers;
import org.frc5687.rapidreact.util.OutliersContainer;

public class Catapult extends OutliersSubsystem {

    private TalonFX _arm;
    private DutyCycleEncoder _boreEncoder;

    private LinearSystem<N2, N1, N1> _catapultPlant;
    private KalmanFilter<N2, N1, N1> _observer;
    private LinearQuadraticRegulator<N2, N1, N1> _controller;
    private LinearSystemLoop<N2, N1, N1> _loop;

    private Matrix<N2, N1> _reference;

    public Catapult(OutliersContainer container) {
        super(container);

        _boreEncoder = new DutyCycleEncoder(RobotMap.DIO.ENCODER_ARM);
        _boreEncoder.setDistancePerRotation(2 * Math.PI);
        _arm = new TalonFX(RobotMap.CAN.TALONFX.ARM_MOTOR);

        // configure the motor.
        _arm.setInverted(Constants.Catapult.INVERTED);
        _arm.setNeutralMode(NeutralMode.Brake);
        _arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 200);
        // set up for voltage as input
        _arm.configVoltageCompSaturation(Constants.Catapult.CONTROL_EFFORT, 200);
        _arm.enableVoltageCompensation(Constants.Catapult.VOLTAGE_COMPENSATION);

        // setup state space
        _catapultPlant = LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getFalcon500(1),
                Constants.Catapult.MOMENT_OF_INERTIA,
                Constants.Catapult.GEAR_RATIO
        );

        // setup observer
        _observer = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                _catapultPlant,
                VecBuilder.fill(
                        Constants.Catapult.MODEL_ANGLE_NOISE,
                        Constants.Catapult.MODEL_ANGULAR_VELOCITY_NOISE
                ),
                VecBuilder.fill(Constants.Catapult.SENSOR_ANGLE_NOISE),
                Constants.Catapult.kDt
        );

        _controller = new LinearQuadraticRegulator<>(
                _catapultPlant,
                VecBuilder.fill(
                        Constants.Catapult.ANGLE_TOLERANCE,
                        Constants.Catapult.ANGULAR_VELOCITY_TOLERANCE
                ),
                VecBuilder.fill(Constants.Catapult.CONTROL_EFFORT),
                Constants.Catapult.kDt
        );
        _loop.reset(VecBuilder.fill(getAngle(), _arm.getSelectedSensorVelocity())); // placeholder, needs real values.
        _reference = VecBuilder.fill(getAngle(), 0);
    }

    @Override
    public void periodic() {
        super.periodic();
        _loop.setNextR(_reference);
        _loop.correct(VecBuilder.fill(getAngle()));
        _loop.predict(Constants.Catapult.kDt);
    }

    public void setSpeed(double speed) {
        _arm.set(ControlMode.PercentOutput, speed);
    }

    public void setVoltage(double voltage) {
        double limVoltage = Helpers.limit(
                voltage,
                -Constants.Catapult.CONTROL_EFFORT,
                Constants.Catapult.CONTROL_EFFORT
        );
        _arm.set(TalonFXControlMode.PercentOutput, limVoltage / Constants.Catapult.CONTROL_EFFORT);
    }

    public void setReference(Matrix<N2, N1> reference) {
        _reference = reference;
    }

    // we are saying we want to get to the velocity we want then stop abruptly so the ball keeps its velocity:
    // This is reversing the motor fast could damage with the amount of torque we are using.
    public void heuristicControl(Matrix<N2, N1> reference) {
        if (Math.abs(reference.get(0, 0) - getAngle()) > Constants.Catapult.CONTROL_TOLERANCE) { // take absolute to not worry about negative
            setReference(reference);
        } else {
            setReference(VecBuilder.fill(reference.get(0,0), 0));
        }
    }

    public double getNextVoltage() {
        return _loop.getU(0);
    }

    public double getAngle() {
        return _boreEncoder.getDistance();
    }


    @Override
    public void updateDashboard() {

    }
}
