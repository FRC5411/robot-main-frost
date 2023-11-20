package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ARM.positions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class moveToObject{
    BooleanSupplier hasObject;
    VisionSubsystem vision;
    Drivetrain drivetrain;
    Arm arm;
    
    public moveToObject(VisionSubsystem vision, BooleanSupplier hasObject, Drivetrain drivetrain, Arm arm) {
        this.vision = vision;
        this.hasObject = hasObject;
        this.drivetrain = drivetrain;
        this.arm = arm;
    }

    public Command toFloor() {
        if(arm.target != positions.Floor || 
           arm.target != positions.AutonFloor) return arm.moveToPositionCommand(positions.Floor);
        else return new InstantCommand();
    }

    public Command toAngle(double tolerance) {
        ProfiledPIDController controller = 
            new ProfiledPIDController(
                6.2, 0, 0, 
                new TrapezoidProfile.Constraints(180, 90) );
        controller.enableContinuousInput( -180, 180 );
        controller.setTolerance( tolerance );

        return new FunctionalCommand(
            () -> {
                vision.getCenterLimelight().setPipelineIndex(1);
                controller.setGoal(0);
                controller.reset( vision.getCenterLimelight().getYaw() );
            }, 
            () -> {
                drivetrain.driveFromChassisSpeeds( 
                    new ChassisSpeeds( 
                        0, 
                        0, 
                        Math.toRadians( controller.calculate( vision.getCenterLimelight().getYaw() ) ) ) );
            }, 
            (interrupted) -> {},
            () -> ( controller.atGoal() ), 
            drivetrain);
    }

    public Command toGamePiece() {
        ProfiledPIDController controller = 
        new ProfiledPIDController(
            6.2, 0, 0, 
            new TrapezoidProfile.Constraints(180, 90) );
        controller.enableContinuousInput( -180, 180 );
        controller.setTolerance( 1 );

        return new FunctionalCommand(
            () -> {
                vision.getCenterLimelight().setPipelineIndex(1);
                controller.setGoal(0);
                controller.reset( vision.getCenterLimelight().getYaw() );
            },
            () -> 
                drivetrain.driveFromChassisSpeeds( 
                    new ChassisSpeeds( 
                        Math.cos( drivetrain.getPose().getRotation().getRadians() ), 
                        Math.sin( drivetrain.getPose().getRotation().getRadians() ), 
                        Math.toRadians( controller.calculate( vision.getCenterLimelight().getYaw() ) ) ) ), 
            (interrupted) -> {}, 
            () -> ( hasObject.getAsBoolean() ), 
            drivetrain);
    }

    public Command collectGamepiece() {
        return new SequentialCommandGroup(
            toAngle(3),
            toFloor(),
            toGamePiece()
        );
    }
}