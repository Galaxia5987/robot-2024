package frc.robot;

public final class Ports {

    public static final class Shooter {
        public static final int TOP_MOTOR_ID = 11;
        public static final int BOTTOM_MOTOR_ID = 10;
    }

    public static final class Hood {
        public static final int MOTOR_ID = 9;
        public static final int ENCODER_ID = 6;
    }

    public static final class Conveyor {
        public static final int MOTOR_ID = 8;
    }

    public static final class SwerveDrive {
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 2;
        public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 1;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 4;
        public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 3;
        public static final int REAR_LEFT_DRIVE_MOTOR_ID = 6;
        public static final int REAR_LEFT_ANGLE_MOTOR_ID = 5;
        public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 8;
        public static final int REAR_RIGHT_ANGLE_MOTOR_ID = 7;

        public static final int[] DRIVE_IDS = {
            FRONT_LEFT_DRIVE_MOTOR_ID,
            FRONT_RIGHT_DRIVE_MOTOR_ID,
            REAR_LEFT_DRIVE_MOTOR_ID,
            REAR_RIGHT_DRIVE_MOTOR_ID
        };
        public static final int[] ANGLE_IDS = {
            FRONT_LEFT_ANGLE_MOTOR_ID,
            FRONT_RIGHT_ANGLE_MOTOR_ID,
            REAR_LEFT_ANGLE_MOTOR_ID,
            REAR_RIGHT_ANGLE_MOTOR_ID
        };

        public static final int FRONT_LEFT_ENCODER_ID = 9;
        public static final int FRONT_RIGHT_ENCODER_ID = 0;
        public static final int REAR_LEFT_ENCODER_ID = 8;
        public static final int REAR_RIGHT_ENCODER_ID = 1;

        public static final int[] ENCODER_IDS = {
            FRONT_LEFT_ENCODER_ID,
            FRONT_RIGHT_ENCODER_ID,
            REAR_LEFT_ENCODER_ID,
            REAR_RIGHT_ENCODER_ID
        };

        public static boolean FRONT_LEFT_DRIVE_INVERTED = true;
        public static boolean FRONT_LEFT_ANGLE_INVERTED = true;
        public static boolean FRONT_RIGHT_DRIVE_INVERTED = true;
        public static boolean FRONT_RIGHT_ANGLE_INVERTED = true;
        public static boolean REAR_LEFT_DRIVE_INVERTED = true;
        public static boolean REAR_LEFT_ANGLE_INVERTED = true;
        public static boolean REAR_RIGHT_DRIVE_INVERTED = true;
        public static final boolean[] DRIVE_INVERTED = {
            FRONT_LEFT_DRIVE_INVERTED,
            FRONT_RIGHT_DRIVE_INVERTED,
            REAR_LEFT_DRIVE_INVERTED,
            REAR_RIGHT_DRIVE_INVERTED
        };
        public static boolean REAR_RIGHT_ANGLE_INVERTED = true;
        public static final boolean[] ANGLE_INVERTED = {
            FRONT_LEFT_ANGLE_INVERTED,
            FRONT_RIGHT_ANGLE_INVERTED,
            REAR_LEFT_ANGLE_INVERTED,
            REAR_RIGHT_ANGLE_INVERTED
        };
    }

    public static class UI {
        public static final int JOYSTICK_TRIGGER = 1;
        public static final int JOYSTICK_TOP_BOTTOM_BUTTON = 2;
        public static final int JOYSTICK_TOP_LEFT_BUTTON = 3;
        public static final int JOYSTICK_TOP_RIGHT_BUTTON = 4;
        public static final int JOYSTICK_RIGHT_BIG_BUTTON = 16;
    }
}
