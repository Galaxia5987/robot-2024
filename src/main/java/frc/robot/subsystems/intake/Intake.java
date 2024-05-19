package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakeInputsAutoLogged inputs = IntakeIO.inputs;
    private static Intake INSTANCE = null;
    private final IntakeIO io;

    private Intake(IntakeIO intakeIO) {
        this.io = intakeIO;
    }

    public static Intake getInstance(IntakeIO io) {
        if (INSTANCE == null) {
            INSTANCE = new Intake(io);
        }
        return INSTANCE;
    }

    // setAngle
    // setSpin
    // intake -> intake angle down && intake spin intake
    // stop
    // outtake

    public Command setAngle(double angle) {
        return Commands.run(
                        () -> {
                            inputs.angleMotorSetPoint = angle;
                            io.setAngle(angle);
                        })
                .withName("set intake angle");
    }

    public Command setSpinVoltage(double voltage) {
        return Commands.run(
                        () -> {
                            inputs.spinMotorSetPoint = voltage;
                            io.setVoltage(voltage);
                        })
                .withName("set spin voltage");
    }

    public Command stop() {
        return Commands.run(
                        () -> {
                            setAngle(IntakeConstants.openingAngle);
                            setSpinVoltage(0);
                        })
                .withName("stop intake");
    }

    public Command intake() {
        return Commands.run(
                        () -> {
                            setAngle(IntakeConstants.closingAngle);
                            setSpinVoltage(IntakeConstants.intakeSpinVoltage);
                        })
                .withName("intake");
    }

    public Command outtake() {
        return Commands.run(
                        () -> {
                            setAngle(IntakeConstants.closingAngle);
                            setSpinVoltage(IntakeConstants.outtakeSpinVoltage);
                        })
                .withName("outtake");
    }

    @Override
    public void periodic() {
        io.updateInput();
    }
}
