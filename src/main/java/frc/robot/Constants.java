// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.ArmPosition;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class PWM {
        public static final int BLINKIN_ID = 0;
    }

    public static class DIO {
        public static final int ARM_STAGE_1_ENCODER_ID = 7;
        public static final int ARM_STAGE_2_ENCODER_ID = 8;
        public static final int ARM_STAGE_3_ENCODER_ID = 9;
        public static final int GRIP_LIMIT_SWITCH = 1;
    }

    public static class CAN {
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
        public static final double AZIMUTH_kF = 0.04;// 0.05
        public static final double AZIMUTH_DEADBAND = 0.06;// 0.1;//0.06;//0.075over slop;//0.1Over slop//0.05 under
                                                           // slop

        // calculated via JVN calculator
        public static final double DRIVE_kP = 0.088062; // 0.04;//0.07;//0.06; //0.044057
        public static final double DRIVE_kF = 0.028998;// 0.04//0.06; //0.028998

        /* Maximum distance for a valid waypoint (meters) */
        public static final double MAX_WAYPOINT_DISTANCE = 0.5;

        public static final double SHWERVE_DRIVE_Kp = 0.044057;
        public static final double SHWERVE_DRIVE_Kd = 0;

        public static final double AUTO_BALANCE_Kp = 0.1;
        public static final double AUTO_BALANCE_Kd = 0;

        public static final StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = 
            new StatorCurrentLimitConfiguration(
          true, 
          60, 
          60, 
          0);
    
        public static final StatorCurrentLimitConfiguration AZIMUTH_CURRENT_LIMIT = 
            new StatorCurrentLimitConfiguration(
          true, 
          30, 
          40, 
          0.2);
    }

    public static class LL {
        public static final String centerLLNT = "limelight-limeone";
        public static final String   leftLLNT = "limelight-left";
        public static final String  rightLLNT = "limelight-right";
        
        public static final Pose3d rightLLOffsetMeters = new Pose3d( 
            new Translation3d(0.13, 0.09, 0.0),
            new Rotation3d(Math.toRadians(43), 0, 0));
        public static final Pose3d leftLLOffsetMeters = new Pose3d( 
            new Translation3d(-0.13, 0.09, 0.0),
            new Rotation3d(Math.toRadians(43), 0, 0));

        public static final int apriltagPipelineIndex = 0;
        public static final int gamePiecePipelineIndex = 1;
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
        public static ArmPosition scoreHighConePosition = new ArmPosition(207, changeScopeAgain(changeScope(130)), 145);
        public static ArmPosition dipHighConePosition = new ArmPosition(198, changeScopeAgain(changeScope(108)), 136);
        public static ArmPosition scoreHighCubePosition = new ArmPosition(185, changeScopeAgain(changeScope(122)), 127);
        public static ArmPosition scoreMidConePosition = new ArmPosition(133.5, changeScopeAgain(changeScope(160)), 160);
        public static ArmPosition dipMidConePosition = new ArmPosition(133.5, changeScopeAgain(changeScope(158)), 140);
        public static ArmPosition scoreMidCubePosition = new ArmPosition(133.5, changeScopeAgain(changeScope(165)), 97);
        public static ArmPosition idlePosition = new ArmPosition(133.5, changeScopeAgain(45), 36);// 131 217 50
        public static ArmPosition scoreLowPosition = new ArmPosition(133.5, changeScopeAgain(changeScope(165)), 52);
        public static ArmPosition floorPosition = new ArmPosition(133.5, changeScopeAgain(changeScope(88)), 142);
        public static ArmPosition autonFloorPosition = new ArmPosition(133.5, changeScopeAgain(changeScope(83)), 132);
        public static ArmPosition floorAltPosition = new ArmPosition(133.5, changeScopeAgain(changeScope(120)), 43);
        // public static ArmPosition floorAltCubePosition = new ArmPosition(133.5, 91, 43);
        public static ArmPosition substationPosition = new ArmPosition(133.5, changeScopeAgain(changeScope(182)), 111);

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

    public static double changeScopeAgain(double angle) {
        double offset = 45 - 318;
        double offsetAngle = angle + offset;
        if(angle < 0) return 360 + offsetAngle;
        return offsetAngle;
    }

    public static class POP {
        public static final double F = 0;
        public static final double R = 0;
        public static final double SPEEDOUT = 0.8;
        public static final double SPEEDIN = 0.50;

        public static final int FORWARD_PNEUMATIC_CHANNEL = 14;
        public static final int BACKWARD_PNEUMATIC_CHANNEL = 15;
    }

    public static class LED {
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
