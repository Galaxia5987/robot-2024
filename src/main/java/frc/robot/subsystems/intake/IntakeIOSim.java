package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import lib.motors.SparkMaxSim;
import lib.motors.TalonFXSim;

public class IntakeIOSim implements IntakeIO {
    TalonFXSim angleMotor;
    SparkMaxSim frontRoller;
    SparkMaxSim centerRoller;
    PositionVoltage positionRequest = new PositionVoltage(0);
    PIDController angleController =
            new PIDController(
                    IntakeConstants.ANGLE_KP, IntakeConstants.ANGLE_KI, IntakeConstants.ANGLE_KD);

    private IntakeIOSim() {
        angleMotor = new TalonFXSim(1, IntakeConstants.GEAR_RATIO, 0.3);
        angleMotor.setController(angleController);
        frontRoller = new SparkMaxSim(1, 1, 0);
        centerRoller = new SparkMaxSim(1, 1, 0);
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        angleMotor.setControl(positionRequest.withPosition(angle.baseUnitMagnitude()));
    }

    @Override
    public void setRollerSpeed(MutableMeasure<Velocity<Angle>> speed) {
        frontRoller.set(speed.baseUnitMagnitude());
    }

    @Override
    public void setCenterRollerSpeed(MutableMeasure<Velocity<Angle>> speed) {
        centerRoller.set(speed.baseUnitMagnitude());
    }

    @Override
    public void updateInputs() {
        inputs.currentAngle.mut_replace((angleMotor.getRotorPosition() * 360), Units.Degrees);
        inputs.rollerSpeed.mut_replace(frontRoller.getVelocity(), Units.RotationsPerSecond);
        inputs.centerRollerSpeed.mut_replace(centerRoller.getVelocity(), Units.RotationsPerSecond);
        inputs.angleMotorVoltage.mut_replace(angleMotor.getAppliedVoltage(), Units.Volts);
        inputs.spinMotorVoltage.mut_replace(frontRoller.getBusVoltage(), Units.Volts);
        inputs.centerMotorVoltage.mut_replace(centerRoller.getBusVoltage(), Units.Volts);
    }
}
