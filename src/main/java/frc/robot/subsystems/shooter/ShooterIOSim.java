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

    private final DutyCycleOut stop =
            new DutyCycleOut(
                    ShooterConstants.STOP_POWER.in(Units.RotationsPerSecond),
                    false,
                    true,
                    false,
                    false);

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
        inputs.topVelocitySetpoint.mut_replace(velocity);
        topMotor.setControl(
                topControl
                        .withVelocity(velocity.in(Units.RotationsPerSecond))
                        .withFeedForward(
                                topFeedForward.calculate(velocity.in(Units.RotationsPerSecond))));
    }

    @Override
    public void setBottomVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        inputs.bottomVelocitySetpoint.mut_replace(velocity);
        bottomMotor.setControl(
                bottomControl
                        .withVelocity(velocity.in(Units.RotationsPerSecond))
                        .withFeedForward(
                                bottomFeedForward.calculate(
                                        velocity.in(Units.RotationsPerSecond))));
    }

    @Override
    public void updateInputs() {
        topMotor.update(Timer.getFPGATimestamp());
        bottomMotor.update(Timer.getFPGATimestamp());

        inputs.topVelocity.mut_replace(topMotor.getVelocity(), Units.RotationsPerSecond);
        inputs.topVoltage.mut_replace(topMotor.getAppliedVoltage(), Units.Volts);

        inputs.bottomVelocity.mut_replace(bottomMotor.getVelocity(), Units.RotationsPerSecond);
        inputs.bottomVoltage.mut_replace(bottomMotor.getAppliedVoltage(), Units.Volts);
    }
}
