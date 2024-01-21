package frc.robot.subsystems.elevator;

public interface ElevatorIO {

    default void getInstance() {
    }

    default void setPower(double power) {
    }

    default void getPower() {
    }

    default void setHeight(double height) {
    }

    default void getHeight() {
    }

    default void getTopSensor() {
    }

    default void getBottomSensor() {
    }

    default void getControlMode(){
    }

    enum ControlMode {
        PERCENT_OUTPUT,
        POSITION
    }

}

