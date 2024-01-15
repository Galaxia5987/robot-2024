package frc.robot.subsystems.example;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ExampleSubsystemIO {

    // The inputs of the subsystem. This is to be used only in the subsystem classes.
    ExampleSubsystemInputsAutoLogged inputs = new ExampleSubsystemInputsAutoLogged();

    /**
     * Set the position of the subsystem
     *
     * @param angle The position of the subsystem.
     */
    void setAngle(Rotation2d angle);

    /**
     * Set the power of the subsystem
     *
     * @param power The power of the subsystem. [-1, 1]
     */
    void setPower(double power);

    /** Update the inputs of the subsystem */
    void updateInputs();

    @AutoLog
    class ExampleSubsystemInputs {
        public Rotation2d angle = new Rotation2d();
        public Translation2d tipPosition = new Translation2d();
        public Rotation2d velocity = new Rotation2d();

        public Rotation2d setpointAngle = new Rotation2d();
        public double setpointPower = 0;
    }
}
