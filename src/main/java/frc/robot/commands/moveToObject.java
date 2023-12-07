package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.Telemetry;
import frc.robot.Constants.LL;
import frc.robot.Constants.ARM.positions;
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
            toAngle(1.5),
            arm.moveToPositionTerminatingCommand(positions.Substation),
            toGamePiece(),
            arm.moveToPositionCommand(positions.Idle)
        );
    }

    public Command toAngle(double tolerance) {
        Debouncer db = new Debouncer(0.2);

        return new FunctionalCommand(
            () -> initSystemsAndController(tolerance, true), 
            () -> {
                System.out.println(" \n \n \n \n RUNN ANGLE \n \n \n \n ");
                System.out.println(" \n \n \n \n"+intake.getPiece()+"\n \n \n \n ");
                double output = getControllerOutput();
                Telemetry.setValue("RLimelight/Output", output);

                drivetrain.driveFromChassisSpeedsLocked ( 
                    new ChassisSpeeds( 
                        0, 0, 
                        output ) );

            }, 
            (interrupted) -> {
                System.out.println(" \n \n \n \n END ANGLE \n \n \n \n ");
            },
            () -> db.calculate(angleController.atGoal()), 
            drivetrain);
    }

    public Command toGamePiece() {
        Debouncer db = new Debouncer(0.1);

        return new FunctionalCommand(
            () -> {
                initSystemsAndController(0, false);
                drivetrain.setRobotOriented(true);
            },
            () -> {
                System.out.println(" \n \n \n \n RUN MOVE \n \n \n \n ");
                drivetrain.driveFromChassisSpeeds( 
                    new ChassisSpeeds( 
                        0.5, 0.0, 
                        getControllerOutput() ) ); }, 
            (interrupted) -> {
                System.out.println(" \n \n END MOVE \n \n ");
                drivetrain.setRobotOriented(false);
            }, 
            () -> ( db.calculate( intake.getPiece() ) ), 
            drivetrain);
    }

    public void initSystemsAndController(double tolerance, boolean reset) {
        vision.setPipelineIndices(LL.gamePiecePipelineIndex);
        setMode();
        if(reset) angleController.reset( vision.getCenterLimelight().getYaw() );
        angleController.setTolerance(tolerance);
    }

    public void setMode() {
        if(vision.getCenterLimelight().getObjectType().equals("cone")) {
            intake.setMode(GamePieces.Cone);
            leds.turnYellow().initialize();
        }
        else if( vision.getCenterLimelight().getObjectType().equals("cube")) {
            intake.setMode(GamePieces.Cube);
            leds.turnPurple().initialize();
        }
    }

    public double getControllerOutput() {
        return Math.toRadians( angleController.calculate( vision.getCenterLimelight().getYaw() ) )
        + Math.toRadians( Math.signum( getControllerOutput() ) * 5 );
    }
}