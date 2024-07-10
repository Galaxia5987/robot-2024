package frc.robot.subsystems.climb;

public interface ClimbIO {
    ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();

    default void setPower(double power) {}

    default void setPosition(double position){}

    default void openStopper() {}

    default void closeStopper() {}

    default void disableStopper() {}

    default void reset(){}

    default void updateInputs(ClimbInputs inputs) {}
}
