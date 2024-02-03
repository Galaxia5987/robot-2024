package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import lib.motors.TalonFXSim;

public class ShooterIOSim implements ShooterIO {
    private final TalonFXSim topMotor;
    private final TalonFXSim bottomMotor;

    private final VelocityVoltage topControl = new VelocityVoltage(0);
    private final SimpleMotorFeedforward topFeedForward;

    private final VelocityVoltage bottomControl = new VelocityVoltage(0);
    private final SimpleMotorFeedforward bottomFeedForward;

    private final DutyCycleOut stop = new DutyCycleOut(0);

    public ShooterIOSim() {
        topMotor =
                new TalonFXSim(
                        1,
                        ShooterConstants.GEAR_RATIO_TOP,
                        ShooterConstants.MOMENT_OF_INERTIA_TOP,
                        1);
        bottomMotor =
                new TalonFXSim(
                        1,
                        ShooterConstants.GEAR_RATIO_BOTTOM,
                        ShooterConstants.MOMENT_OF_INERTIA_BOTTOM,
                        1);

        topMotor.setController(
                new PIDController(
                        ShooterConstants.TOP_kP.get(),
                        ShooterConstants.TOP_kI.get(),
                        ShooterConstants.TOP_kD.get()));
        bottomMotor.setController(
                new PIDController(
                        ShooterConstants.BOTTOM_kP.get(),
                        ShooterConstants.BOTTOM_kI.get(),
                        ShooterConstants.BOTTOM_kD.get()));

        topFeedForward =
                new SimpleMotorFeedforward(
                        ShooterConstants.TOP_kS.get(),
                        ShooterConstants.TOP_kV.get(),
                        ShooterConstants.TOP_kA.get());
        bottomFeedForward =
                new SimpleMotorFeedforward(
                        ShooterConstants.BOTTOM_kS.get(),
                        ShooterConstants.BOTTOM_kV.get(),
                        ShooterConstants.BOTTOM_kA.get());
    }

    @Override
    public void setTopVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        topRollerInputs.velocitySetpoint.mut_replace(velocity);
        topMotor.setControl(
                topControl
                        .withVelocity(velocity.in(Units.RotationsPerSecond))
                        .withFeedForward(
                                topFeedForward.calculate(velocity.in(Units.RotationsPerSecond))));
    }

    @Override
    public void setBottomVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        bottomRollerInputs.velocitySetpoint.mut_replace(velocity);
        bottomMotor.setControl(
                bottomControl
                        .withVelocity(velocity.in(Units.RotationsPerSecond))
                        .withFeedForward(
                                bottomFeedForward.calculate(
                                        velocity.in(Units.RotationsPerSecond))));
    }

    @Override
    public void stop() {
        bottomRollerInputs.velocitySetpoint.mut_replace(0, Units.RotationsPerSecond);
        topRollerInputs.velocitySetpoint.mut_replace(0, Units.RotationsPerSecond);
        bottomMotor.setControl(stop);
        topMotor.setControl(stop);
    }

    @Override
    public void updateInputs() {
        topMotor.update(Timer.getFPGATimestamp());
        bottomMotor.update(Timer.getFPGATimestamp());

        topRollerInputs.velocity.mut_replace(topMotor.getVelocity(), Units.RotationsPerSecond);
        topRollerInputs.voltage.mut_replace(topMotor.getAppliedVoltage(), Units.Volts);

        bottomRollerInputs.velocity.mut_replace(
                bottomMotor.getVelocity(), Units.RotationsPerSecond);
        bottomRollerInputs.voltage.mut_replace(bottomMotor.getAppliedVoltage(), Units.Volts);
    }
}
