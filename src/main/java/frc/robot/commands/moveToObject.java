package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.Telemetry;
import frc.robot.Constants.LL;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PinchersofPower;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.PinchersofPower.GamePieces;

public class moveToObject{
    VisionSubsystem vision;
    Drivetrain drivetrain;
    Arm arm;
    PinchersofPower intake;
    LEDs leds;
    ProfiledPIDController angleController;
    
    public moveToObject(VisionSubsystem vision, Drivetrain drivetrain, Arm arm, PinchersofPower intake,LEDs leds) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.leds = leds;

        angleController = 
            new ProfiledPIDController(
                6.2, 0, 0, 
                new TrapezoidProfile.Constraints(360, 240) );
        angleController.enableContinuousInput( -180, 180 );
        angleController.setTolerance( 0.0 );
        angleController.setGoal(0);
    }

    public Command collectGamepiece() {
        return new SequentialCommandGroup(
            angleToGamePiece(1.5).withTimeout(3),
            // arm.moveToPositionTerminatingCommand(positions.Substation).withTimeout(3),
            moveToGamePiece().withTimeout(3)
            //arm.moveToPositionCommand(positions.Idle)
        );
    }

    public Command angleToGamePiece(double tolerance) {
            return moveToObjectCommand(
                0, tolerance, true, 0.2, 
                () -> angleController.atGoal());
    }

    public Command moveToGamePiece() {
        return moveToObjectCommand(
            0.5, 0, false, 0.1, 
            () -> intake.getPiece() );
    }

    public Command moveToObjectCommand(
        double forwardSpeed, double tolerance, boolean shouldReset, 
        double debounceTime, BooleanSupplier endCondition) {  
            Debouncer db = new Debouncer(debounceTime);
            
            return new FunctionalCommand(
            () -> initSystemsAndController(tolerance, shouldReset),
            () -> drivetrain.driveFromChassisSpeeds( 
                    new ChassisSpeeds( 
                        forwardSpeed, 0.0, 
                        getControllerOutput() ) ), 
            (interrupted) -> {
                System.out.println(" \n \n END MOVE \n \n ");
                drivetrain.setRobotOriented(false);
            }, 
            () -> db.calculate( endCondition.getAsBoolean() ), 
            drivetrain);
    }

    public void initSystemsAndController(double tolerance, boolean reset) {
        vision.setPipelineIndices(LL.gamePiecePipelineIndex);

        if(vision.getCenterLimelight().getObjectType().equals("cone")) {
            intake.setMode(GamePieces.Cone);
            leds.turnYellow().initialize();
        }
        else if( vision.getCenterLimelight().getObjectType().equals("cube")) {
            intake.setMode(GamePieces.Cube);
            leds.turnPurple().initialize();
        }

        if(reset) angleController.reset( vision.getCenterLimelight().getYaw() );
        angleController.setTolerance(tolerance);

        drivetrain.setRobotOriented(true);
    }

    public double getControllerOutput() {
        double output = Math.toRadians( angleController.calculate( vision.getCenterLimelight().getYaw() ) )
                        + Math.toRadians( Math.signum( angleController.getPositionError() ) * 5 );

        Telemetry.setValue("RLimelight/output", output);
        return output;
    }
}