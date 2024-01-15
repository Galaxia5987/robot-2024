package frc.robot.subsystems.example;

import static frc.robot.subsystems.example.ExampleSubsystemConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ExampleSubsystemIOReal implements ExampleSubsystemIO {

    private final TalonFX motor = new TalonFX(ExampleSubsystemConstants.MOTOR_PORT);

    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public ExampleSubsystemIOReal() {
        CONFIG.Slot0.withKP(POSITION_P.get())
                .withKI(POSITION_I.get())
                .withKD(POSITION_D.get())
                .withKV(POSITION_V.get());
        CONFIG.CurrentLimits.withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(40)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        CONFIG.Feedback.withSensorToMechanismRatio(GEAR_RATIO);

        while (motor.getConfigurator().apply(CONFIG) != StatusCode.OK) {
            System.out.println("Configuring motor " + motor.getDeviceID());
        }
    }

    @Override
    public void setAngle(Rotation2d angle) {
        positionRequest.withPosition(angle.getRotations()).withEnableFOC(true);
        motor.setControl(positionRequest);
    }

    @Override
    public void setPower(double power) {
        dutyCycleRequest.withOutput(power).withEnableFOC(true);
        motor.setControl(dutyCycleRequest);
    }

    @Override
    public void updateInputs() {
        inputs.angle = Rotation2d.fromRotations(motor.getPosition().getValue());
        inputs.tipPosition =
                new Translation2d(inputs.angle.getCos(), inputs.angle.getSin())
                        .times(ExampleSubsystemConstants.LENGTH);
        inputs.velocity = Rotation2d.fromRotations(motor.getVelocity().getValue());
    }
}
