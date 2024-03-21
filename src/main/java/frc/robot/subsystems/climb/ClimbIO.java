package frc.robot.subsystems.climb;

public interface ClimbIO {
    ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();

    void setPower(double power);

    void openStopper();

    void closeStopper();

    default void disableServo() {}

    void updateInputs(ClimbInputs inputs);
}
