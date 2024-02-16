package frc.robot.subsystems.conveyor;

import static frc.robot.subsystems.conveyor.ConveyorConstants.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants;
import frc.robot.Ports;

public class ConveyorIOReal implements ConveyorIO {

    private CANSparkMax roller =
            new CANSparkMax(Ports.Conveyor.MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    public static SimpleMotorFeedforward feed =
            new SimpleMotorFeedforward(KS.get(), KV.get(), KA.get());

    public ConveyorIOReal() {
        roller.restoreFactoryDefaults();
        roller.setIdleMode(CANSparkMax.IdleMode.kCoast);
        roller.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE.in(Units.Volts));
        roller.setInverted(true);
        roller.getPIDController().setP(KP.get());
        roller.getPIDController().setI(KI.get());
        roller.getPIDController().setD(KD.get());
        roller.getEncoder()
                .setVelocityConversionFactor(ConveyorConstants.VELOCITY_CONVERSION_FACTOR);
        roller.burnFlash();
    }

    @Override
    public void setVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        roller.getPIDController()
                .setReference(
                        velocity.in(Units.RotationsPerSecond),
                        CANSparkBase.ControlType.kVelocity,
                        0,
                        feed.calculate(velocity.in(Units.RotationsPerSecond)));
    }

    @Override
    public void updateInputs() {
        inputs.currentVelocity.mut_replace(
                (roller.getEncoder().getVelocity()), Units.RotationsPerSecond);
        inputs.appliedCurrent.mut_replace((roller.getOutputCurrent()), Units.Amps);
        inputs.appliedVoltage.mut_replace((roller.getBusVoltage()), Units.Volts);
    }
}
