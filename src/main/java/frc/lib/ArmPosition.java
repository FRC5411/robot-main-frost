package frc.lib;

import static frc.robot.Constants.ARM.STAGE_1_LENGTH;
import static frc.robot.Constants.ARM.STAGE_2_LENGTH;

import java.util.function.Supplier;

public class ArmPosition {
    private double stage1Angle = 0;
    private double stage2Angle = 0;
    private double stage3Angle = 0;

    public ArmPosition (double stage1Angle, double stage2Angle, double stage3Angle) {
        this.stage1Angle = stage1Angle;
        this.stage2Angle = stage2Angle;
        this.stage3Angle = stage3Angle;
    }

    public double getStage1Angle () {
        return stage1Angle;
    }

    public double getStage2Angle () {
        return stage2Angle;
    }

    public double getStage3Angle () {
        return stage3Angle;
    }

    public double getXPosition() {
        return STAGE_1_LENGTH * Math.cos(Math.toRadians(stage1Angle)) + STAGE_2_LENGTH * Math.cos(Math.toRadians(stage2Angle));
    }

    public double getYPosition() {
        return -(STAGE_1_LENGTH * Math.sin(Math.toRadians(stage1Angle)) + STAGE_2_LENGTH * Math.sin(Math.toRadians(stage2Angle)));
    }

    public static double[] inverseKinematics (double x, double y, double theta, Supplier<double[]> currentpoint) {
        double[] output = new double[3];

        double l1 = STAGE_1_LENGTH;
        double l2 = STAGE_2_LENGTH;
        double l3 = Math.hypot(x, y);
        double thetaA = Math.toDegrees(Math.acos((l2*l2-l1*l1-l3*l3)/(-2*l1*l3)));
        double thetaB = Math.toDegrees(Math.acos((l3*l3-l1*l1-l2*l2)/(-2*l1*l2)));

        output[0] = (360 + Math.toDegrees(Math.atan2(y, x)) + thetaA) % 360;
        output[1] = (360 + output[0] + thetaB + 180) % 360;
        output[2] = (360 + theta) % 360;

        if ( y >= 20 || x > 49.0 || Double.isNaN(output[0]) || 
            Double.isNaN(output[1]) || Double.isNaN(output[2])) output = currentpoint.get();

        return output;
    }

    public static double[] forwardKinematics ( double stage1Degrees, double stage2Degrees, double stage3Degrees ) {
        double[] output = new double[3];
        output[0] = Math.cos(Math.toRadians(stage1Degrees)) * STAGE_1_LENGTH + Math.cos(Math.toRadians(stage2Degrees)) * (STAGE_2_LENGTH);
        output[1] = Math.sin(Math.toRadians(stage1Degrees)) * STAGE_1_LENGTH + Math.sin(Math.toRadians(stage2Degrees)) * (STAGE_2_LENGTH);
        output[2] = (360 + stage3Degrees) % 360;
        return output;
    }
}
