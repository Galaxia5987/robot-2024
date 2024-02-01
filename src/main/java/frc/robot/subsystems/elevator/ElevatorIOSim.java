package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.Timer;
import lib.motors.TalonFXSim;

public class ElevatorIOSim implements ElevatorIO {
    private final TalonFXSim motor;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final DutyCycleOut powerRequest = new DutyCycleOut(0);

    public ElevatorIOSim() {
        motor = new TalonFXSim(2, ElevatorConstants.GEAR_RATIO, 0.5, 1);

        motor.setProfiledController(
                new ProfiledPIDController(
                        ElevatorConstants.KP.get(),
                        ElevatorConstants.KI.get(),
                        ElevatorConstants.KD.get(),
                        ElevatorConstants.));
    }

    @Override
    public void setPower(double power) {
        powerRequest.withOutput(power);
        motor.setControl(powerRequest);
    }

    @Override
    public void setHeight(MutableMeasure<Distance> height) {
        inputs.heightSetpoint = height;
        positionRequest.withPosition(height.in(Meters));
        motor.setControl(positionRequest);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        motor.update(Timer.getFPGATimestamp());
        inputs.currentHeight =
                Meters.of(
                                lib.units.Units.rpsToMetersPerSecond(
                                        motor.getPosition(), ElevatorConstants.DRUM_RADIUS))
                        .mutableCopy();
    }
}
