package frc.robot;

import frc.lib.ArmPosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.commands.HolonomicController.HolonomicConstraints;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;

public final class Constants {
    public class PWM {
        public static final int BLINKIN_ID = 0;
    }

    public class DIO {
        public static final int ARM_STAGE_1_ENCODER_ID = 7;
        public static final int ARM_STAGE_2_ENCODER_ID = 8;
        public static final int ARM_STAGE_3_ENCODER_ID = 9;
        public static final int GRIP_LIMIT_SWITCH = 1;
    }

    public class CAN {
        // CAN Bus IDs
        public static final int PDH_ID = 1;
        public static final int PCH_ID = 2;
        public static final int PIGEON_ID = 3;

        public static final int FL_CANCODER_ID = 4;
        public static final int FR_CANCODER_ID = 5;
        public static final int BL_CANCODER_ID = 6;
        public static final int BR_CANCODER_ID = 7;

        public static final int FL_DRIVE_ID = 11;
        public static final int FR_DRIVE_ID = 12;
        public static final int BL_DRIVE_ID = 13;
        public static final int BR_DRIVE_ID = 14;

        public static final int SHWERVE_DRIVE_ID = 15;

        public static final int FL_AZIMUTH_ID = 21;
        public static final int FR_AZIMUTH_ID = 22;
        public static final int BL_AZIMUTH_ID = 23;
        public static final int BR_AZIMUTH_ID = 24;

        public static final int ARM_STAGE_1_ID = 31;
        public static final int ARM_STAGE_1_FOLLOWER_ID = 32;
        public static final int ARM_STAGE_2_ID = 33;
        public static final int ARM_STAGE_3_ID = 34;

        public static final int GRIP_LEFT_ID = 41;
        public static final int GRIP_RIGHT_ID = 42;
    }

    public static class DRIVETRAIN {
        // robot width (meters)
        public static final double ROBOT_WIDTH_METERS = 0.6858;
        // wheel diameter (meters)
        public static final double WHEEL_DIAMETER_METERS = 0.1016;
        public static final double WHEEL_PERIMETER_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        
        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double AZIMUTH_GEAR_RATIO = 12.8;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(  ROBOT_WIDTH_METERS / 2,  ROBOT_WIDTH_METERS / 2 ),
            new Translation2d(  ROBOT_WIDTH_METERS / 2, -ROBOT_WIDTH_METERS / 2 ),
            new Translation2d( -ROBOT_WIDTH_METERS / 2,  ROBOT_WIDTH_METERS / 2 ),
            new Translation2d( -ROBOT_WIDTH_METERS / 2, -ROBOT_WIDTH_METERS / 2 ) );

        // encoder offsets (degrees)
        public static final double FL_ECODER_OFFSET = -313.506 + 0.5;
        public static final double FR_ECODER_OFFSET = -69.082 + 0.5;
        public static final double BL_ECODER_OFFSET = -45.791 + 180;
        public static final double BR_ECODER_OFFSET = -257.783;

        /** maximum strafe speed (meters per second) */
        public static final double MAX_LINEAR_SPEED = 5.4;
        /** maximum rotation speed (radians per second) */
        public static final double MAX_ROTATION_SPEED = Math.PI * 2;
        public static final double SWERVE_SLOW_SPEED_PERCENTAGE = 0.1;
        public static final double ROTATION_SCALE_FACTOR = 0.65;

        // pid values
        public static final double AZIMUTH_kP = 0.0105;// 0.0115//0.0125;//0.025 //0.05//0.1 //0.01 //0.0053 sds: 0.2;
                                                       // rylan: 0.65
        public static final double AZIMUTH_kD = 0.000265;// 0.000275;//0.0003;//0.0004;//0.0005;//0.0006;//0.0006125;//0.0006125//0.000625//0.00065//0.0006;//0.00055//0.0005;//0.002//0.001//0.00075
                                                         // //0.0005;//0.00025
        public static final double AZIMUTH_kF = 0.04;
        
        public static final double FL_kF = 0.047;
        public static final double FR_kF = 0.04; // 0.05
        public static final double BL_kF = 0.04; // 0.05
        public static final double BR_kF = 0.05;

        public static final double AZIMUTH_DEADBAND = 0.06;// 0.1;//0.06;//0.075over slop;//0.1Over slop//0.05 under
                                                           // slop
        public static final PIDController FL_PID = new PIDController(0.0094, 0, 0.000277); // 0.105
        public static final PIDController FR_PID = new PIDController(0.0095, 0, 0.000270);
        public static final PIDController BL_PID = new PIDController(0.0096, 0, 0.000270);
        public static final PIDController BR_PID = new PIDController(0.0093, 0, 0.000270);

        // calculated via JVN calculator
        public static final double DRIVE_kP = 0.088062; // 0.04;//0.07;//0.06; //0.044057
        public static final double DRIVE_kF = 0.028998;// 0.04//0.06; //0.028998

        /* Maximum distance for a valid waypoint (meters) */
        public static final double MAX_WAYPOINT_DISTANCE = 0.5;

        public static final double SHWERVE_DRIVE_Kp = 0.044057;
        public static final double SHWERVE_DRIVE_Kd = 0;

        public static final double AUTO_BALANCE_Kp = 0.1;
        public static final double AUTO_BALANCE_Kd = 0;

        public static final double _translationKp = 2.40;// 2.35//1.8;//3.25;//2.75;//2.5;//2.1;//2;//0.018;//0.03;//0.004 0.001
        public static final double _translationKi = 0;
        public static final double _translationKd = 0;
        public static final double _rotationKp = 1.83;//1.83;// 2.5//12.5;//15;//0.00005
        public static final double _rotationKi = 0;
        public static final double _rotationKd = 0.085; // 0.1

        public static final PIDConstants _translationPID = 
            new PIDConstants(_translationKp, _translationKi, _translationKd);
        public static final PIDConstants _rotationPID = 
            new PIDConstants(_rotationKp,    _rotationKi,    _rotationKd);
      
        public static final double _alignXKp = 3.0;//5.5;//5; //5.5;
        public static final double _alignXKi = 0.0;//0.1;//0.;
        public static final double _alignXKd = 0.03;//0.05;
      
        public static final double _alignYKp = 2.5;//2.2;//3.1; //5.5;
        public static final double _alignYKi = 0.00; //0.01;//0.;
        public static final double _alignYKd = 0.02; //0.03;
      
        public static final double _alignRotationKp = 6.2;//2.5;
        public static final double _alignRotationKi = 0.0;// 0.03; //.42;
        public static final double _alignRotationKd = 0;//.0;

        public static final Constraints _YConstraints = new Constraints(4, 6);
        public static final Constraints _XConstraints = new Constraints(3, 5);
        public static final Constraints _rotConstraints = new Constraints(360, 240);

        public static final HolonomicConstraints _holonomicConstraints = 
          new HolonomicConstraints(_XConstraints, _YConstraints, _rotConstraints);

        public static final Pose2d defaultTolerance = new Pose2d(
            0.1, 
            0.1, 
            Rotation2d.fromDegrees(3) );

        public static final double downChargeLine = 1.0;
        public static final double upChargeLine = 4.5;
        public static final double rightChargeLine = 14.05;
        public static final double leftChargeLine = 11.2;

        public static final Field2d field2d = new Field2d();
    }

    public static class LL {
        public static final double translationGains = 3.1;
        public static final double rotationGains = 1000000;
    }

    public static final class ARM {
        public static final double JOINT_ANGLE_DEADZONE = 20;
        public static final double JOINT_COORDINATE_DEADZONE = 0;

        public static enum positions {
            ScoreHighCone,
            DipHighCone,
            ScoreHighCube,
            ScoreMidCone,
            DipMidCone,
            ScoreMidCube,
            ScoreHighPlace,
            ScoreMidPlace,
            ScoreLow,
            Floor,
            AutonFloor,
            FloorAlt,
            Substation,
            Idle,
            IdleShootPosition
        };

        public static final double STAGE_1_OFFSET = 203;
        public static final double STAGE_2_OFFSET = 270;
        public static final double STAGE_3_OFFSET = 210;
        public static ArmPosition scoreHighConePosition = new ArmPosition(207, changeScope(130), 145);
        public static ArmPosition dipHighConePosition = new ArmPosition(198, changeScope(108), 136);
        public static ArmPosition scoreHighCubePosition = new ArmPosition(185, changeScope(122), 127);
        public static ArmPosition scoreMidConePosition = new ArmPosition(133.5, changeScope(160), 160);
        public static ArmPosition dipMidConePosition = new ArmPosition(133.5, changeScope(158), 140);
        public static ArmPosition scoreMidCubePosition = new ArmPosition(133.5, changeScope(165), 97);
        public static ArmPosition idlePosition = new ArmPosition(133.5, 45, 36);// 131 217 50
        public static ArmPosition scoreLowPosition = new ArmPosition(133.5, changeScope(165), 52);
        public static ArmPosition floorPosition = new ArmPosition(133.5, changeScope(88), 142);
        public static ArmPosition autonFloorPosition = new ArmPosition(133.5, changeScope(83), 132);
        public static ArmPosition floorAltPosition = new ArmPosition(133.5, changeScope(120), 43);
        // public static ArmPosition floorAltCubePosition = new ArmPosition(133.5, 91, 43);
        public static ArmPosition substationPosition = new ArmPosition(133.5, changeScope(182), 111);

        public static final double THETA_SPEED = 1;
        public static final double X_SPEED = 0.5;
        public static final double Y_SPEED = 0.5;

        // inches
        public static final double STAGE_1_LENGTH = 20.0;
        public static final double STAGE_2_LENGTH = 29.5;

        public static final double STAGE_1_Kp = 0.0450;// kp=.008, 0.025
        public static final double STAGE_1_Ki = 0.001;
        public static final double STAGE_1_Kd = 0.0003;// 0.0005
        public static final double STAGE_1_Ks = 0.04;// 0.04;//0.0005
        public static final double STAGE_1_Kg = 1.1;// 1.3 causes tiny pulse width modulation; 0.5
        public static final double STAGE_1_MAX_SPEED = 800;
        public static final double STAGE_1_MAX_ACCEL = 240;// 180;//200//170;//160;//140;//130

        public static final double STAGE_2_Kp = 0.0135;// 0.035//0.04//0.0325//0.008
        public static final double STAGE_2_Ki = 0.00;// 0.0025 //0.0065
        public static final double STAGE_2_Kd = 0.0044;// 0.0041 // 0.0051;//0.0042//0.00420//0.035 0.0043
        public static final double STAGE_2_Ks = 0.025;/// 0.03;//0.05
        public static final double STAGE_2_Kg = 0.82;//0.72;// 0.92//1.4, 1.0
        public static final double STAGE_2_MAX_SPEED = 800;// 250;
        public static final double STAGE_2_MAX_ACCEL = 275;// 210;//200//190;//180;//160;//170//150

        public static final double STAGE_3_Kp = 0.0175;// 0.0255//0.0155;//0.005
        public static final double STAGE_3_Ki = 0;
        public static final double STAGE_3_Kd = 0.000001;// 0.000
        public static final double STAGE_3_Ks = 0.02;
        public static final double STAGE_3_Kg = 0.42;
        public static final double STAGE_3_MAX_SPEED = 800; // 500
        public static final double STAGE_3_MAX_ACCEL = 600;// 280;//250;//230;//300
    }

    public static double changeScope(double angle) {
        double newAngle = angle - 167;
        if(newAngle < 0) return 360 + newAngle;
        return newAngle;
    }

    public class POP {
        public static final double F = 0;
        public static final double R = 0;
        public static final double SPEEDOUT = 0.8;
        public static final double SPEEDIN = 0.50;

        public static final int FORWARD_PNEUMATIC_CHANNEL = 14;
        public static final int BACKWARD_PNEUMATIC_CHANNEL = 15;
    }

    public class LED {
        // public static final double YELLOW = 0.69;
        // public static final double PURPLE = 0.91;
        // public static final double RED = 0.61;
        // public static final double GREEN = 0.77;
        // public static final double RAINBOW = -0.99;
        public final static int YELLOWR = 255;
        public final static int YELLOWG = 255;
        public final static int YELLOWB = 0;
        public final static int PURPLER = 255;
        public final static int PURPLEG = 0;
        public final static int PURPLEB = 255;
        public final static int REDR = 255;
        public final static int REDG = 0;
        public final static int REDB = 0;
        public final static int GREENR = 0;
        public final static int GREENG = 255;
        public final static int GREENB = 0;
    }
}
