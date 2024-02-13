package frc.robot.subsystems.hood;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.motors.TalonFXSim;

public class HoodIOSim implements HoodIO {
    private final TalonFXSim motor;

    private final MotionMagicDutyCycle control = new MotionMagicDutyCycle(0);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public HoodIOSim() {
        motor =
                new TalonFXSim(
                        1,
                        HoodConstants.GEAR_RATIO,
                        HoodConstants.MOMENT_OF_INERTIA.in(
                                Units.Kilograms.mult(Units.Meters).mult(Units.Meters)),
                        HoodConstants.GEAR_RATIO);

        motor.setProfiledController(
                new ProfiledPIDController(
                        HoodConstants.kP.get(),
                        HoodConstants.kI.get(),
                        HoodConstants.kD.get(),
                        new TrapezoidProfile.Constraints(
                                HoodConstants.MAX_VELOCITY, HoodConstants.MAX_ACCELERATION)));
    }

    @Override
    public void updateInternalEncoder() {}

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        inputs.controlMode = Mode.ANGLE;
        inputs.angleSetpoint.mut_replace(angle);
        motor.setControl(control.withPosition(angle.in(Units.Rotations)));
    }

    @Override
    public void setPower(double power) {
        inputs.controlMode = Mode.POWER;
        inputs.powerSetpoint = power;
        motor.setControl(dutyCycleOut.withOutput(power));
    }

    @Override
    public void updateInputs() {
        motor.update(Timer.getFPGATimestamp());

        inputs.angle.mut_replace(motor.getPosition(), Units.Rotations);
        inputs.voltage.mut_replace(motor.getAppliedVoltage(), Units.Volts);
    }
}
