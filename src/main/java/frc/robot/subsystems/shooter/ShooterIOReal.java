package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    private final TalonFXConfigurator topMotorConfigurator;
    private final TalonFXConfigurator bottomMotorConfigurator;
    private final TalonFXConfiguration topMotorConfiguration = new TalonFXConfiguration();
    private final TalonFXConfiguration bottomMotorConfiguration = new TalonFXConfiguration();
    private MotionMagicVelocityTorqueCurrentFOC topControl =
            new MotionMagicVelocityTorqueCurrentFOC(0);
    private MotionMagicVelocityTorqueCurrentFOC bottomControl =
            new MotionMagicVelocityTorqueCurrentFOC(0);

    public ShooterIOReal() {
        topMotor = new TalonFX(3); // TODO: to be changed later
        bottomMotor = new TalonFX(5); // TODO: to be changed later

        topMotor.setNeutralMode(NeutralModeValue.Coast);
        bottomMotor.setNeutralMode(NeutralModeValue.Coast);

        topMotorConfiguration.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO_TOP;
        bottomMotorConfiguration.Feedback.SensorToMechanismRatio =
                ShooterConstants.GEAR_RATIO_BOTTOM;

        topMotorConfigurator = topMotor.getConfigurator();
        topMotorConfiguration.Slot0.kP = ShooterConstants.TOP_kP.get();
        topMotorConfiguration.Slot0.kI = ShooterConstants.TOP_kI.get();
        topMotorConfiguration.Slot0.kD = ShooterConstants.TOP_kD.get();
        topMotorConfiguration.Slot0.kS = ShooterConstants.TOP_kS.get();
        topMotorConfiguration.Slot0.kV = ShooterConstants.TOP_kV.get();
        topMotorConfiguration.Slot0.kA = ShooterConstants.TOP_kA.get();
        topMotorConfiguration.Slot0.kG = ShooterConstants.TOP_kG.get();
        topMotorConfigurator.apply(topMotorConfiguration.Slot0);

        bottomMotorConfigurator = bottomMotor.getConfigurator();
        bottomMotorConfiguration.Slot0.kP = ShooterConstants.TOP_kP.get();
        bottomMotorConfiguration.Slot0.kI = ShooterConstants.TOP_kI.get();
        bottomMotorConfiguration.Slot0.kD = ShooterConstants.TOP_kD.get();
        bottomMotorConfiguration.Slot0.kS = ShooterConstants.TOP_kS.get();
        bottomMotorConfiguration.Slot0.kV = ShooterConstants.TOP_kV.get();
        bottomMotorConfiguration.Slot0.kA = ShooterConstants.TOP_kA.get();
        bottomMotorConfiguration.Slot0.kG = ShooterConstants.TOP_kG.get();
        bottomMotorConfigurator.apply(bottomMotorConfiguration.Slot0);
    }

    @Override
    public void setTopVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        topRollerInputs.velocitySetpoint.mut_replace(velocity);
        topMotor.setControl(topControl.withVelocity(velocity.in(Units.RotationsPerSecond)));
    }

    @Override
    public void setBottomVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        bottomRollerInputs.velocitySetpoint.mut_replace(velocity);
        bottomMotor.setControl(bottomControl.withVelocity(velocity.in(Units.RotationsPerSecond)));
    }

    @Override
    public void stop() {
        bottomRollerInputs.velocitySetpoint = Units.RotationsPerSecond.zero().mutableCopy();
        topRollerInputs.velocitySetpoint = Units.RotationsPerSecond.zero().mutableCopy();
        bottomMotor.stopMotor();
        topMotor.stopMotor();
    }

    @Override
    public void updateInputs() {
        topRollerInputs.velocity.mut_replace(
                topMotor.getVelocity().getValue(), Units.RotationsPerSecond);
        topRollerInputs.voltage.mut_replace(topMotor.getMotorVoltage().getValue(), Units.Volts);

        bottomRollerInputs.velocity.mut_replace(
                bottomMotor.getVelocity().getValue(), Units.RotationsPerSecond);
        bottomRollerInputs.voltage.mut_replace(
                bottomMotor.getMotorVoltage().getValue(), Units.Volts);
    }
}
