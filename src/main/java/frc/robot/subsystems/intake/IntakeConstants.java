package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import lib.webconstants.LoggedTunableNumber;

public class IntakeConstants {
    public static final double GEAR_RATIO = 45.62;
    public static final LoggedTunableNumber ANGLE_KP =
            new LoggedTunableNumber("IntakeSim/kP", 10.0 / 360.0);
    public static final LoggedTunableNumber ANGLE_KI = new LoggedTunableNumber("IntakeSim/kI", 0);
    public static final LoggedTunableNumber ANGLE_KD = new LoggedTunableNumber("IntakeSim/kD", 0);
    public static PIDController angleController =
            new PIDController(ANGLE_KP.get(), ANGLE_KI.get(), ANGLE_KD.get());

    public enum IntakePose {
        UP(MutableMeasure.zero(Degrees)),
        DOWN(MutableMeasure.zero(Degrees));
        final MutableMeasure<Angle> intakePose;

        IntakePose(MutableMeasure<Angle> intakePose) {
            this.intakePose = intakePose;
        }
    }
}
