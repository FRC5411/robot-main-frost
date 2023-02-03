package frc.robot;
public final class Constants {
    public class CAN {
        //CAN Bus IDs
        public static final int PIGEON_ID = 0;

        public static final int FL_CANCODER_ID = 5;
        public static final int FR_CANCODER_ID = 6;
        public static final int BL_CANCODER_ID = 7;
        public static final int BR_CANCODER_ID = 8;

        public static final int FL_DRIVE_ID = 11;
        public static final int FR_DRIVE_ID = 12;
        public static final int BL_DRIVE_ID = 13;
        public static final int BR_DRIVE_ID = 14;

        public static final int FL_AZIMUTH_ID = 21;
        public static final int FR_AZIMUTH_ID = 22;
        public static final int BL_AZIMUTH_ID = 23;
        public static final int BR_AZIMUTH_ID = 24;
    }

    public class DRIVETRAIN {
        // robot width (meters)
        public static final double ROBOT_WIDTH = 0.6858;
        // wheel diameter (meters)
        public static final double WHEEL_DIAMETER = 0.1016;
        // drive gear ratio
        public static final double DRIVE_GEAR_RATIO = 6.75;

        // encoder offsets (degrees)
        public static final double FL_ECODER_OFFSET = -313.682;
        public static final double FR_ECODER_OFFSET = -166.553;
        public static final double BL_ECODER_OFFSET = -246.006;
        public static final double BR_ECODER_OFFSET = -204.258;
        
        /** maximum strafe speed (meters per second) */
        public static final double MAX_LINEAR_SPEED = 5.4;
        /** maximum rotation speed (radians per second) */
        public static final double MAX_ROTATION_SPEED = Math.PI*2;
            
        // pid values
        public static final double AZIMUTH_kP = 0.2;
        public static final double AZIMUTH_kD = 0.1;

        // calculated via JVN calculator
        public static final double DRIVE_kP = 0.044057;
        public static final double DRIVE_kF = 0.028998;
    }
        public class ARM {        
           /* public static final double LOW_DIST = 0;
            public static final double LOW_X = 0;
            public static final double LOW_Y = 0;    
            public static final double HIGH_DIST = 0;
            public static final double HIGH_X = 0;
            public static final double HIGH_Y = 0;*/        
            public static final double BISCEP_LENGTH = 20;        
            public static final double ELBOW_LENGTH = 30;
            public static final double LOW_ARM_ANG = 152.8;
            public static final double LOW_ELBOW_ANG = 168.9;
            public static final double LOW_CLAW_ANG = -14.5;
            public static final double HIGH_ARM_ANG = 83;
            public static final double HIGH_ELBOW_ANG = 168.9;
            public static final double HIGH_CLAW_ANG = -142.5;
            public static final double IDLE_ARM_ANG = 63.8;
            public static final double IDLE_ELBOW_ANG = -38.4;
            public static final double IDLE_CLAW_ANG = 96.9;
    }

        public class POP {
            public static final double F = 0;
            public static final double R = 0;
            public static final double SPEED = 0.5;
        }
}