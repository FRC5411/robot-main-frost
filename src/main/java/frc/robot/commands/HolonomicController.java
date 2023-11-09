package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.Telemetry;
import frc.lib.SProfile;
import frc.lib.SProfilePIDController;
import frc.lib.SProfile.State;
import frc.lib.SProfile.Constraints;

public class HolonomicController {
    private SProfilePIDController xController;
    private SProfilePIDController yController;
    private SProfilePIDController thetaController;

    public HolonomicController(
        SProfilePIDController xController, SProfilePIDController yController, SProfilePIDController thetaController) {
        thetaController.enableContinuousInput( -Math.PI, Math.PI );

        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;

        setTolerance( new Pose2d() );
    }

    public void reset(Pose2d startPose) {
        reset( startPose, new ChassisSpeeds() );
    }

    public void reset(Pose2d startPose, ChassisSpeeds targetChassisSpeeds) {
        xController.reset( 
            new State(
                startPose.getX(),
                targetChassisSpeeds.vxMetersPerSecond, 0, 0 ) );

        yController.reset( 
            new State(
                startPose.getY(),
                targetChassisSpeeds.vyMetersPerSecond, 0 ,0 ) );

        if(thetaController.getPositionError() > 7.5) {
            thetaController.reset(
                new State(
                    startPose.getRotation().getRadians(),
                    targetChassisSpeeds.omegaRadiansPerSecond, 0, 0 ) );
        }
    }

    public boolean atGoal() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }

    public Pose2d getPositionGoal() {
        return new Pose2d(
            new Translation2d(
                xController.getGoal().position, 
                yController.getGoal().position), 
            new Rotation2d(
                thetaController.getGoal().position
            ));
    }

    public Pose2d getPositionSetpoint() {
        return new Pose2d(
            new Translation2d(
                xController.getSetpoint().position, 
                yController.getSetpoint().position ), 
            new Rotation2d(
                thetaController.getSetpoint().position ) );
    }

    public ChassisSpeeds getVelocityGoal() {
        return new ChassisSpeeds(
            xController.getGoal().velocity,
            yController.getGoal().velocity,
            thetaController.getGoal().velocity );
    }

    public ChassisSpeeds getVelocitySetpoint() {
        return new ChassisSpeeds(
            xController.getSetpoint().velocity,
            yController.getSetpoint().velocity,
            thetaController.getSetpoint().velocity );
    }

    public ChassisSpeeds calculate(Pose2d goalPose, Pose2d currentPose) {
        return calculate(goalPose, new ChassisSpeeds(), currentPose);
    }

    public ChassisSpeeds calculate(Pose2d goalPose, ChassisSpeeds goalSpeed, Pose2d currentPose) {
        return new ChassisSpeeds(
            xController.calculate( 
                currentPose.getX(), 
                new State(
                    goalPose.getX(),
                    goalSpeed.vxMetersPerSecond, 0, 0) ),

            yController.calculate( 
                currentPose.getY(), 
                new State(
                    goalPose.getY(),
                    goalSpeed.vyMetersPerSecond, 0, 0) ),

            thetaController.calculate( 
                currentPose.getRotation().getDegrees(), 
                new State(
                    goalPose.getRotation().getDegrees(),
                    goalSpeed.vxMetersPerSecond, 0, 0) ) );
    }

    public ChassisSpeeds calculate(Pose2d robotPose) {
        return new ChassisSpeeds(
            xController.calculate( robotPose.getX() ),
            yController.calculate( robotPose.getY() ),
            thetaController.calculate( robotPose.getRotation().getRadians() )
        );
    }

    public ChassisSpeeds calculateWithFF(Pose2d goalPose, Pose2d currentPose) {
        return calculateWithFF(goalPose, new ChassisSpeeds(), currentPose);
    }

    public ChassisSpeeds calculateWithFF(Pose2d goalPose, ChassisSpeeds goalSpeed, Pose2d currentPose) {
        return new ChassisSpeeds(
            xController.calculate( 
                currentPose.getX(), 
                new State(
                    goalPose.getX(),
                    goalSpeed.vxMetersPerSecond, 0, 0) )
            + xController.getSetpoint().velocity,

            yController.calculate( 
                currentPose.getY(), 
                new State(
                    goalPose.getY(),
                    goalSpeed.vyMetersPerSecond, 0, 0) )
            + yController.getSetpoint().velocity,

            thetaController.calculate( 
                currentPose.getRotation().getDegrees(), 
                new State(
                    goalPose.getRotation().getDegrees(),
                    goalSpeed.omegaRadiansPerSecond, 0, 0) )
            + thetaController.getSetpoint().velocity
            );
    }

    public ChassisSpeeds calculateWithFF(Pose2d robotPose, double xkS, double xkV, double ykS, double ykV, double thetakS) {

        ChassisSpeeds speeds = new ChassisSpeeds(
            xController.calculate( robotPose.getX() )
            + xController.getSetpoint().velocity * xkV + xkS * Math.signum(xController.getSetpoint().velocity),
            yController.calculate( robotPose.getY() )
            + yController.getSetpoint().velocity * ykV + ykS * Math.signum(yController.getSetpoint().velocity),
            thetaController.calculate( robotPose.getRotation().getRadians()
            + thetaController.getSetpoint().velocity + thetakS * Math.signum(thetaController.getSetpoint().velocity) ) );
        
        Telemetry.setValue("ALIGNMENT/XOUTPUT", speeds.vxMetersPerSecond);
        Telemetry.setValue("ALIGNMENT/YOUTPUT", speeds.vyMetersPerSecond);
        Telemetry.setValue("ALIGNMENT/OMEGAOUTPUT", speeds.omegaRadiansPerSecond);

        Telemetry.setValue("ALIGNMENT/XPIDOUTPUT", speeds.vxMetersPerSecond - xController.getSetpoint().velocity);
        Telemetry.setValue("ALIGNMENT/YPIDOUTPUT", speeds.vyMetersPerSecond - yController.getSetpoint().velocity);
        Telemetry.setValue("ALIGNMENT/OMEGAPIDOUTPUT", speeds.omegaRadiansPerSecond - thetaController.getSetpoint().velocity);

        return speeds;
    }

    public void setGoal(Pose2d goalPose) {
        setGoal(goalPose, new ChassisSpeeds());
    }

    public void setGoal(Pose2d goalPose, ChassisSpeeds goalSpeed) {
        xController.setGoal( 
            new State(
                goalPose.getX(),
                goalSpeed.vxMetersPerSecond, 0, 0) );
        yController.setGoal( 
            new State(
                goalPose.getY(),
                goalSpeed.vyMetersPerSecond, 0, 0) );
        thetaController.setGoal( 
            new State(
                goalPose.getRotation().getRadians(),
                goalSpeed.omegaRadiansPerSecond, 0, 0) );
    }

    public void setTolerance(Pose2d tolerance) {
        xController.setTolerance( tolerance.getX() );
        yController.setTolerance( tolerance.getY() );
        thetaController.setTolerance( tolerance.getRotation().getRadians() );
    }

    public SProfilePIDController getXController() {
        return xController;
    }

    public SProfilePIDController getYController() {
        return yController;
    }

    public SProfilePIDController getThetaController() {
        return thetaController;
    }

    public void xControllerIRange(double range) {
        xControllerIRange( -range, range );
    }

    public void yControllerIRange(double range) {
        yControllerIRange( -range, range );
    }

    public void thetaControllerIRange(double range) {
        thetaControllerIRange( -range, range );
    }

    public void xControllerIRange(double lowerBound, double higherBound) {
        xController.setIntegratorRange( lowerBound, higherBound );
    }

    public void yControllerIRange(double lowerBound, double higherBound) {
        yController.setIntegratorRange( lowerBound, higherBound );
    }

    public void thetaControllerIRange(double lowerBound, double higherBound) {
        thetaController.setIntegratorRange( lowerBound, higherBound );
    }

    public void xIZone(double kI, double min, double max) {
        if((xController.getPositionError() < max) || (xController.getPositionError() > min)) xController.setI(kI);
        else xController.setI(0);
    }

    public void yIZone(double kI, double min, double max) {
        if((yController.getPositionError() < max) || (yController.getPositionError() > min)) xController.setI(kI);
        else yController.setI(0);
    }

    public void thetaIZone(double kI, double min, double max) {
        if((thetaController.getPositionError() < max) || (thetaController.getPositionError() > min)) xController.setI(kI);
        else thetaController.setI(0);
    }

    public Pose2d getPoseError() {
        return new Pose2d(
            xController.getPositionError(),
            yController.getPositionError(),
            new Rotation2d(thetaController.getPositionError())
        );
    }

    public static class HolonomicConstraints {
        Constraints xConstraints;
        Constraints yConstraints;
        Constraints thetaConstraints;

        public HolonomicConstraints(Constraints xConstraints, Constraints yConstraints, Constraints thetaConstraints) {
                this.xConstraints = xConstraints;
                this.yConstraints = yConstraints;
                this.thetaConstraints = thetaConstraints;
        }

        public double getLongestTime(State xGoal, State yGoal, State thetaGoal) {
            SProfile xProfile = new SProfile(xConstraints, xGoal);
            SProfile yProfile = new SProfile(yConstraints, yGoal);
            SProfile thetaProfile = new SProfile(thetaConstraints, thetaGoal);

            return Math.max(
                Math.max(xProfile.totalTime(), yProfile.totalTime()), thetaProfile.totalTime());
        }

        public double getLongestTime(Pose2d posGoal, ChassisSpeeds velGoal) {
            return getLongestTime(
                new State(
                    posGoal.getX(), 
                    velGoal.vxMetersPerSecond, 0, 0 ),
                new State(
                    posGoal.getY(),
                    velGoal.vyMetersPerSecond, 0, 0 ), 
                new State(
                    posGoal.getRotation().getDegrees(),
                    velGoal.omegaRadiansPerSecond, 0, 0 ) );
        }

        public double getLongestTime(Pose2d posGoal) {
            return getLongestTime(posGoal, new ChassisSpeeds());
        }
    }
}