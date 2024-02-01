package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import frc.robot.Constants;
import lib.webconstants.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Degrees;

public class IntakeConstants {
    public static final double GEAR_RATIO = 45.62;
    public static final LoggedTunableNumber ANGLE_KP =
            new LoggedTunableNumber("IntakeSim/kP");
    public static final LoggedTunableNumber ANGLE_KI = new LoggedTunableNumber("IntakeSim/kI");
    public static final LoggedTunableNumber ANGLE_KD = new LoggedTunableNumber("IntakeSim/kD");

    public enum IntakePose {
        UP(MutableMeasure.zero(Degrees)),
        DOWN(MutableMeasure.zero(Degrees));
        final MutableMeasure<Angle> intakePose;

        IntakePose(MutableMeasure<Angle> intakePose) {
            this.intakePose = intakePose;
        }


        public void InitConstants() {
            switch (Constants.CURRENT_MODE) {


                case REAL:

                    ANGLE_KP.initDefault(10.0 / 360.0);

                    ANGLE_KI.initDefault(0);

                    ANGLE_KD.initDefault(0);


                    break;

                case SIM:

                case REPLAY:
                    ANGLE_KP.initDefault(10.0 / 360.0);


                    ANGLE_KI.initDefault(0);


                    ANGLE_KD.initDefault(0);

                default:
                    ANGLE_KP.initDefault(10.0 / 360.0);


                    ANGLE_KI.initDefault(0);


                    ANGLE_KD.initDefault(0);

                    break;


            }

        }

    }
}
