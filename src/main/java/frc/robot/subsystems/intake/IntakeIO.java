package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    void updateInput();

    void setAngle(double angle);

    void setVoltage(double voltage);

    @AutoLog
    class IntakeInputs {
        public double angle = 0.0;
        public double angleMotorSetPoint = 0.0;
        public double angleMotorVoltage = 0.0;
        public double spinMotorSetPoint = 0.0;
        public double spinMotorVoltage = 0.0;
    }
}
