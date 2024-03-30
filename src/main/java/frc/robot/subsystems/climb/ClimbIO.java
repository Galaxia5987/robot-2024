package frc.robot.subsystems.climb;

public interface ClimbIO {
    ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();

    default void setPower(double power) {}

    default void openStopper() {}

    default void closeStopper() {}

    default void disableStopper() {}

    default void updateInputs(ClimbInputs inputs) {}
}
