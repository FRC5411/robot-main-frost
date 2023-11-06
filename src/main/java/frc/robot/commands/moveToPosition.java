package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.Telemetry;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.commands.HolonomicController.HolonomicConstraints;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import frc.lib.SimpleUtils;

public class moveToPosition {
    private Supplier<Pose2d> currentPose;
    private Supplier<ChassisSpeeds> currentChassisSpeeds;
    private Consumer<ChassisSpeeds> setDesiredStates;
    private Drivetrain requirements;
    private Pose2d target = new Pose2d();

    public moveToPosition( 
        Supplier<Pose2d> curPoseSupplier, Supplier<ChassisSpeeds> curSpeedSupplier, 
        Consumer<ChassisSpeeds> setDesiredStates, Drivetrain requirements) {
        currentPose = curPoseSupplier;
        currentChassisSpeeds = curSpeedSupplier;
        this.setDesiredStates = setDesiredStates;
        this.requirements = requirements;
    }

    public Command generateMoveToPositionCommand( 
        Pose2d targetPose, Pose2d tolerance, HolonomicController controller ) {
        return generateMoveToPositionCommand( 
            targetPose, 
            new ChassisSpeeds(), 
            tolerance, 
            controller );
    }

    public Command generateMoveToPositionCommand(
        Pose2d targetPose, ChassisSpeeds targetChassisSpeeds, 
        Pose2d tolerance, HolonomicController controller ) {
        final double xKi = controller.getXController().getI();
        final double yKi = controller.getYController().getI();
        final double thetaKi = controller.getThetaController().getI();

        return new FunctionalCommand(
            () -> {
                this.target = targetPose;
                controller.setTolerance( tolerance );
                controller.reset( currentPose.get(), currentChassisSpeeds.get() );
                controller.setGoal(targetPose, targetChassisSpeeds);
                
                DRIVETRAIN.field2d.getObject( "Goal" ).setPose( controller.getPositionGoal() );
            },
            () -> {
                controller.xIZone(xKi, controller.getPoseError().getX(), -0.5, 0.5);
                controller.yIZone(yKi, controller.getPoseError().getY(), -0.5, 0.5);
                controller.thetaIZone(thetaKi, controller.getPoseError().getRotation().getDegrees(), -5, 5);

                setDesiredStates.accept(
                    SimpleUtils.discretize( 
                        controller.calculateWithFF( currentPose.get() ) ) );

                DRIVETRAIN.field2d.getObject( "Setpoint" ).setPose( controller.getPositionSetpoint() );
                Telemetry.getValue("PathPlanner/AtGoal", controller.atGoal() );
            }, 
            (interrupted) -> { requirements.joystickDrive(0, 0, 0); }, 
            () -> controller.atGoal(),
            requirements) ;
    }

    public Command generateMoveToPositionCommandTimed(
        Pose2d targetPose, Pose2d tolerance, 
        HolonomicConstraints profiles, HolonomicController controller ) {
        return generateMoveToPositionCommandTimed(targetPose, new ChassisSpeeds(), tolerance, profiles, controller);
    }

    public Command generateMoveToPositionCommandTimed(
        Pose2d targetPose, ChassisSpeeds targetChassisSpeeds, 
        Pose2d tolerance,  HolonomicConstraints profiles, HolonomicController controller) {
        Telemetry.setValue("Alignment/MOM", profiles.getLongestTime(targetPose, targetChassisSpeeds));
        return generateMoveToPositionCommand(targetPose, targetChassisSpeeds, tolerance, controller)
            .withTimeout(profiles.getLongestTime(targetPose, targetChassisSpeeds));
    }

    public Pose2d getTarget() {
        return target;
    }

    public List<Pose2d> optimizeWaypoints(Pose2d target) {
        List<Pose2d> waypoints = new ArrayList<Pose2d>();
        if(currentPose.get().getY() < DRIVETRAIN.downChargeLine) 
            waypoints.add(linearOptimize( currentPose.get(), target, DRIVETRAIN.downChargeLine) );
        else if(currentPose.get().getY() > DRIVETRAIN.upChargeLine) 
            waypoints.add(linearOptimize(currentPose.get(), target,  DRIVETRAIN.upChargeLine  ) );
        else if(currentPose.get().getY() > DRIVETRAIN.downChargeLine && currentPose.get().getY() < DRIVETRAIN.upChargeLine) {
            rectangularOptimize( currentPose.get(), target, waypoints) ;
            waypoints.add(target);
        }
        return waypoints;
    }

      public Pose2d linearOptimize(Pose2d robotPose, Pose2d target, double avoidanceLine) {
        // Uses point slope form to find the equation of the line between the robot and the target
        double a = robotPose.getY();
        double b = robotPose.getX();
        double slope = (a - target.getY()) / (b - target.getX());
        double intersection = slope * (avoidanceLine - b) + a;
    
        if(intersection > 14.05) return (new Pose2d(14.05, DRIVETRAIN.downChargeLine, new Rotation2d(0)));
        return new Pose2d(14.05, target.getY(), target.getRotation());
    }

      public void rectangularOptimize(Pose2d robotPose, Pose2d target, List<Pose2d> waypoints) {
        List<Pose2d> onTheWay = new ArrayList<Pose2d>();
        onTheWay.add( new Pose2d( robotPose.getX(),   DRIVETRAIN.upChargeLine, new Rotation2d() ) );
        onTheWay.add( new Pose2d( robotPose.getX(), DRIVETRAIN.downChargeLine, new Rotation2d() ) );
        Pose2d nearest = robotPose.nearest( onTheWay );
    
        // Slope point form
        double a = robotPose.getY();
        double b = robotPose.getX();
        double m = (a - target.getY()) / (b - target.getX());
        double xIntersection = m * (nearest.getY() - b) + a;
        double yIntersection = ( (DRIVETRAIN.rightChargeLine - a) / m ) + b;
    
        if(target.getY() > DRIVETRAIN.downChargeLine && target.getY() < DRIVETRAIN.upChargeLine) {
            waypoints.add( nearest );
            waypoints.add( new Pose2d(14.05, nearest.getY(), new Rotation2d() ) );
            waypoints.add( new Pose2d(14.05, target.getY(), new Rotation2d() ) );
        } else if((xIntersection > DRIVETRAIN.leftChargeLine && xIntersection < DRIVETRAIN.rightChargeLine) ||
                  (yIntersection > DRIVETRAIN.downChargeLine && yIntersection < DRIVETRAIN.upChargeLine   )) {
            waypoints.add( nearest );
            waypoints.add( new Pose2d(14.05, target.getY(), new Rotation2d() ) );
        } else waypoints.add( new Pose2d(14.05, target.getY(), new Rotation2d() ) );
    }
}