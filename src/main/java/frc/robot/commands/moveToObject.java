package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ARM.positions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PinchersofPower;
import frc.robot.subsystems.VisionSubsystem;

public class moveToObject{
    BooleanSupplier hasObject;
    VisionSubsystem vision;
    Drivetrain drivetrain;
    Arm arm;
    PinchersofPower intake;
    
    public moveToObject(VisionSubsystem vision, BooleanSupplier hasObject, Drivetrain drivetrain, Arm arm, PinchersofPower intake) {
        this.vision = vision;
        this.hasObject = hasObject;
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
    }

    public Command toSubstation() {
        if(arm.target != positions.Substation) return arm.moveToPositionTerminatingCommand(positions.Substation);
        else return new InstantCommand();
    }

    public Command toAngle(double tolerance) {
        ProfiledPIDController controller = 
            new ProfiledPIDController(
                6.2, 0, 0, 
                new TrapezoidProfile.Constraints(180, 90) );
        controller.enableContinuousInput( -180, 180 );
        controller.setTolerance( tolerance );

        Debouncer db = new Debouncer(0.2);

        return new FunctionalCommand(
            () -> {
                vision.getCenterLimelight().setPipelineIndex(1);
                controller.setGoal(0);
                controller.reset( vision.getCenterLimelight().getYaw() );
            }, 
            () -> {
                System.out.println(" \n \n \n \n RUNN ANGLE \n \n \n \n ");
                System.out.println(" \n \n \n \n"+hasObject.getAsBoolean()+"\n \n \n \n ");
                drivetrain.driveFromChassisSpeedsLocked ( 
                    new ChassisSpeeds( 
                        0, 
                        0, 
                        Math.toRadians( controller.calculate( vision.getCenterLimelight().getYaw() ) ) ) );
                
            }, 
            (interrupted) -> {
                System.out.println(" \n \n \n \n END ANGLE \n \n \n \n ");
            },
            () -> ( db.calculate(controller.atGoal()) ), 
            drivetrain);
    }

    public Command toGamePiece() {
        ProfiledPIDController controller = 
        new ProfiledPIDController(
            6.2, 0, 0, 
            new TrapezoidProfile.Constraints(180, 90) );
        controller.enableContinuousInput( -180, 180 );
        controller.setTolerance( 1 );
        controller.setGoal(0);

        Debouncer db = new Debouncer(0.1);

        return new FunctionalCommand(
            () -> {
                System.out.println(" \n \n \n \n RUN MOVE \n \n \n \n ");
                vision.getCenterLimelight().setPipelineIndex(1);
                controller.setGoal(0);
                controller.reset( vision.getCenterLimelight().getYaw() );
                drivetrain.setRobotOriented(false);
            },
            () -> {
                System.out.println(" \n \n \n \n RUN MOVE \n \n \n \n ");
                drivetrain.driveFromChassisSpeeds( 
                    new ChassisSpeeds( 
                        0.5, 0, 
                        Math.toRadians( controller.calculate( vision.getCenterLimelight().getYaw() ) ) ) ); }, 
            (interrupted) -> {
                System.out.println(" \n \n \n \n END MOVE \n \n \n \n ");
                drivetrain.setRobotOriented(true);
            }, 
            () -> ( db.calculate(hasObject.getAsBoolean()) ), 
            drivetrain);
    }

    public Command collectGamepiece() {
        return new SequentialCommandGroup(
            toAngle(3),
            toSubstation(),
            toGamePiece(),
            new InstantCommand( () -> intake.closeGrip() )
        );
    }
}