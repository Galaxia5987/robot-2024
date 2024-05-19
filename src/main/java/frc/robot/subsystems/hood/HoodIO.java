package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();
    void updateInputs();

    void setAngle(double angle);
    void updateInternalEncoder();

    @AutoLog
    class HoodInputs {
        public double angle = 0.0;
        public double angleSetPoint = 0.0;
        public double voltage = 0.0;
        public double absoluteEncoderAngle = .0;
    }
}
