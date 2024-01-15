package frc.robot.subsystems.example;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import lib.motors.TalonFXSim;
import lib.units.Units;

public class ExampleSubsystemIOSim implements ExampleSubsystemIO {

    private final TalonFXSim motor;

    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final DutyCycleOut powerRequest = new DutyCycleOut(0);

    public ExampleSubsystemIOSim() {
        motor =
                new TalonFXSim(
                        1,
                        ExampleSubsystemConstants.GEAR_RATIO,
                        ExampleSubsystemConstants.MOMENT_OF_INERTIA);

        motor.setController(
                new PIDController(
                        ExampleSubsystemConstants.POSITION_P.get(),
                        ExampleSubsystemConstants.POSITION_I.get(),
                        ExampleSubsystemConstants.POSITION_D.get()));
    }

    @Override
    public void setAngle(Rotation2d angle) {
        motor.setControl(positionRequest.withPosition(angle.getRotations()));
    }

    @Override
    public void setPower(double power) {
        motor.setControl(powerRequest.withOutput(power));
    }

    @Override
    public void updateInputs() {
        motor.update(Timer.getFPGATimestamp());

        inputs.angle = Rotation2d.fromRotations(motor.getPosition());
        inputs.tipPosition =
                new Translation2d(inputs.angle.getCos(), inputs.angle.getSin())
                        .times(ExampleSubsystemConstants.LENGTH);
        inputs.velocity = Rotation2d.fromRotations(Units.rpmToRps(motor.getVelocity()));
    }
}
