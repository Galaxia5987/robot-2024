package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Ports;

public class ClimbIOReal implements ClimbIO {
    private final TalonFX mainMotor;
    private final Timer timer = new Timer();

    private final DutyCycleOut powerControl = new DutyCycleOut(0).withEnableFOC(true);

    public ClimbIOReal() {
        mainMotor = new TalonFX(Ports.Elevator.MAIN_ID);
        mainMotor.getConfigurator().apply(ClimbConstants.MAIN_MOTOR_CONFIGURATION);

        timer.reset();
        timer.start();
    }

    @Override
    public void setPower(double power) {
        mainMotor.setControl(powerControl.withOutput(power-0.035));
    }

    public void stopMotor() {
        mainMotor.stopMotor();
    }

    public void manualReset() {
        mainMotor.setPosition(0);
    }

    @Override
    public void updateInputs(ClimbInputs inputs) {
        inputs.appliedVoltage.mut_replace(mainMotor.getMotorVoltage().getValue(), Volts);
    }
}
