package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.motors.TalonFXSim;

public class ClimbIOSim implements ClimbIO {
    private final TalonFXSim motor;

    private final DutyCycleOut powerRequest = new DutyCycleOut(0);

    public ClimbIOSim() {
        motor = new TalonFXSim(2, ClimbConstants.GEAR_RATIO, 0.000_01, 1);
    }

    @Override
    public void setPower(double power) {
        motor.setControl(powerRequest.withOutput(power));
    }

    @Override
    public void openStopper() {}

    @Override
    public void closeStopper() {}

    @Override
    public void updateInputs(ClimbInputs inputs) {
        motor.update(Timer.getFPGATimestamp());
    }
}
