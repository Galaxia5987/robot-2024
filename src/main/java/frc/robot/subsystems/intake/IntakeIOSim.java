package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import lib.motors.SparkMaxSim;
import lib.motors.TalonFXSim;

public class IntakeIOSim implements IntakeIO {
    private final TalonFXSim angleMotor;
    private final SparkMaxSim frontRoller;
    private final SparkMaxSim centerRoller;
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    public static PIDController angleController =
            new PIDController(ANGLE_KP.get(), ANGLE_KI.get(), ANGLE_KD.get());
    public SimpleMotorFeedforward spinFeedForward = new SimpleMotorFeedforward(SPIN_KS.get(), SPIN_KV.get(), SPIN_KA.get() );

    public IntakeIOSim() {
        angleMotor =
                new TalonFXSim(
                        1, IntakeConstants.GEAR_RATIO, 0.003, 360 * IntakeConstants.GEAR_RATIO);
        frontRoller = new SparkMaxSim(1, 1, 0.5, 1);
        frontRoller.setController(new PIDController(SPIN_KP.get(), SPIN_KI.get(), SPIN_KD.get()));
        centerRoller = new SparkMaxSim(1, 1, 0.5, 1);
        angleMotor.setController(angleController);
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        inputs.angleSetPoint = angle;
        angleMotor.setControl(positionRequest.withPosition(angle.in(Units.Degrees)));
    }

    @Override
    public void setRollerSpeed(MutableMeasure<Velocity<Angle>> speed) {
        frontRoller.setReference(speed.in(Units.RotationsPerSecond), CANSparkBase.ControlType.kVelocity, spinFeedForward.calculate(speed.in(Units.RotationsPerSecond)));
    }

    @Override
    public void setCenterRollerSpeed(double speed) {
        centerRoller.set(speed);
    }

    @Override
    public void updateInputs() {
        angleMotor.update(Timer.getFPGATimestamp());
        centerRoller.update(Timer.getFPGATimestamp());
        frontRoller.update(Timer.getFPGATimestamp());
        inputs.currentAngle.mut_replace((angleMotor.getPosition()), Units.Degrees);
        inputs.currentRollerSpeed.mut_replace(frontRoller.getVelocity(), Units.RotationsPerSecond);
        inputs.currentCenterRollerSpeed.mut_replace(
                centerRoller.getVelocity(), Units.RotationsPerSecond);
        inputs.angleMotorVoltage.mut_replace(angleMotor.getAppliedVoltage(), Units.Volts);
        inputs.spinMotorVoltage.mut_replace(frontRoller.getBusVoltage(), Units.Volts);
        inputs.centerMotorVoltage.mut_replace(centerRoller.getBusVoltage(), Units.Volts);
    }
}
