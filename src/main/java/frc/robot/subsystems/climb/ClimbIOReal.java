package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Ports;
import frc.robot.lib.math.differential.BooleanTrigger;

public class ClimbIOReal implements ClimbIO {
    private final TalonFX mainMotor;
    private final TalonFX auxMotor;

    private final DutyCycleOut powerControl = new DutyCycleOut(0).withEnableFOC(true);

    private final DigitalInput sensor = new DigitalInput(1);
    private final BooleanTrigger sensorTrigger = new BooleanTrigger();
    private final BooleanTrigger movingWeightTrigger = new BooleanTrigger();

    private double kS = ClimbConstants.KS_FIRST_STAGE.get();
    private double kG = ClimbConstants.KG_FIRST_STAGE.get();

    public ClimbIOReal() {
        mainMotor = new TalonFX(Ports.Elevator.MAIN_ID);
        auxMotor = new TalonFX(Ports.Elevator.AUX_ID);

        mainMotor.getConfigurator().apply(ClimbConstants.MAIN_MOTOR_CONFIGURATION);
        auxMotor.getConfigurator().apply(ClimbConstants.AUX_MOTOR_CONFIGURATION);

        auxMotor.setControl(new StrictFollower(mainMotor.getDeviceID()));
    }

    @Override
    public void setPower(double power) {
        mainMotor.setControl(powerControl.withOutput(power));
    }

    public void stopMotor() {
        mainMotor.stopMotor();
    }

    public void manualReset() {
        mainMotor.setPosition(0);
    }

    @Override
    public void updateInputs(ClimbInputs inputs) {
        inputs.isBottom = !sensor.get();
        sensorTrigger.update(inputs.isBottom);

        if (movingWeightTrigger.triggered()) {
            kG = ClimbConstants.KG_SECOND_STAGE.get();
            kS = ClimbConstants.KS_SECOND_STAGE.get();
        } else if (movingWeightTrigger.released()) {
            kG = ClimbConstants.KG_FIRST_STAGE.get();
            kS = ClimbConstants.KS_FIRST_STAGE.get();
        }

        if (sensorTrigger.triggered()) {
            mainMotor.setPosition(0);
            auxMotor.setPosition(0);
        }
    }
}
