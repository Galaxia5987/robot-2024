package frc.robot.subsystems.climb;

public interface ClimbIO {
        ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();

    void setPower(double power);

    void manualReset();

    void updateInputs(ClimbInputs inputs);

    enum ControlMode {
        PERCENT_OUTPUT,
        POSITION
    }
}
