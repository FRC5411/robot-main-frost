package frc.robot.commands;

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
import frc.robot.subsystems.PinchersofPower.GamePieces;

public class moveToObject{
    VisionSubsystem vision;
    Drivetrain drivetrain;
    Arm arm;
    PinchersofPower intake;
    ProfiledPIDController angleController;
    
    public moveToObject(VisionSubsystem vision, Drivetrain drivetrain, Arm arm, PinchersofPower intake) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;

        angleController = 
            new ProfiledPIDController(
                6.2, 0, 0, 
                new TrapezoidProfile.Constraints(180, 90) );
        angleController.enableContinuousInput( -180, 180 );
        angleController.setTolerance( 0.0 );
        angleController.setGoal(0);
    }

    public Command toAngle(double tolerance) {
        Debouncer db = new Debouncer(0.2);

        return new FunctionalCommand(
            () -> initSystemsAndController(tolerance), 
            () -> {
                System.out.println(" \n \n \n \n RUNN ANGLE \n \n \n \n ");
                System.out.println(" \n \n \n \n"+intake.getPiece()+"\n \n \n \n ");
                drivetrain.driveFromChassisSpeedsLocked ( 
                    new ChassisSpeeds( 
                        0, 0, 
                        getControllerOutput() ) );
            }, 
            (interrupted) -> {
                System.out.println(" \n \n \n \n END ANGLE \n \n \n \n ");
            },
            () -> ( db.calculate(angleController.atGoal()) ), 
            drivetrain);
    }

    public Command toGamePiece() {
        Debouncer db = new Debouncer(0.1);

        return new FunctionalCommand(
            () -> {
                initSystemsAndController(0);
                drivetrain.setRobotOriented(false);
            },
            () -> {
                System.out.println(" \n \n \n \n RUN MOVE \n \n \n \n ");
                drivetrain.driveFromChassisSpeeds( 
                    new ChassisSpeeds( 
                        0.5, 0, 
                        getControllerOutput() ) ); }, 
            (interrupted) -> {
                System.out.println(" \n \n END MOVE \n \n ");
                drivetrain.setRobotOriented(true);
            }, 
            () -> ( db.calculate( intake.getPiece() ) ), 
            drivetrain);
    }

    public Command collectGamepiece() {
        return new SequentialCommandGroup(
            toAngle(3),
            arm.moveToPositionTerminatingCommand(positions.Substation),
            toGamePiece(),
            arm.moveToPositionCommand(positions.Idle)
        );
    }

    public void setMode() {
        if(vision.getCenterLimelight().getObjectType().equals("cone")) intake.setMode(GamePieces.Cone);
        else if( vision.getCenterLimelight().getObjectType().equals("cube")) intake.setMode(GamePieces.Cube);

    }

    public void initSystemsAndController(double tolerance) {
        vision.getCenterLimelight().setPipelineIndex(1);
        setMode();
        angleController.reset( vision.getCenterLimelight().getYaw() );
        angleController.setTolerance(tolerance);
    }

    public double getControllerOutput() {
        return Math.toRadians( angleController.calculate( vision.getCenterLimelight().getYaw() ) );
    }
}