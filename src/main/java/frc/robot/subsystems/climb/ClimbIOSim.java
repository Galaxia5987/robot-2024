package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.motors.TalonFXSim;

public class ClimbIOSim implements ClimbIO {
    private final TalonFXSim motor;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final DutyCycleOut powerRequest = new DutyCycleOut(0);

    public ClimbIOSim() {
        motor =
                new TalonFXSim(
                        2,
                        ClimbConstants.GEAR_RATIO,
                        0.000_01,
                        ClimbConstants.GEAR_RATIO
                                * (2 * Math.PI * ClimbConstants.DRUM_RADIUS));

        motor.setProfiledController(
                new ProfiledPIDController(
                        ClimbConstants.KP.get(),
                        ClimbConstants.KI.get(),
                        ClimbConstants.KD.get(),
                        ClimbConstants.TRAPEZOID_PROFILE));
    }

    @Override
    public void setPower(double power) {
        motor.setControl(powerRequest.withOutput(power));
    }

    @Override
    public void setHeight(MutableMeasure<Distance> height) {
        motor.setControl(positionRequest.withPosition(height.in(Meters)));
    }

    @Override
    public void openStopper() {}

    @Override
    public void closeStopper() {}

    @Override
    public void manualReset() {}

    @Override
    public void updateInputs(ClimbInputs inputs) {
        motor.update(Timer.getFPGATimestamp());

        inputs.hooksHeight = Meters.of(motor.getPosition()).mutableCopy();
        inputs.carriageHeight.mut_replace(
                inputs.hooksHeight.gt(ClimbConstants.GRIPPER_TO_HOOKS)
                        ? inputs.hooksHeight.mut_minus(ClimbConstants.GRIPPER_TO_HOOKS)
                        : Meters.zero());
    }
}
