package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveToPointCommand;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();;
    private final XboxController driverOne = new XboxController(0);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveCommand(swerveSubsystem, driverOne));
        configureButtonBindings();
        // System.out.println(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    }

    private void configureButtonBindings() {

        
        
    }

    public SwerveSubsystem getSwerve() {
        return swerveSubsystem;
    }

    public Command getAutonomousCommand() {

        // Create a voltage constraint to ensure we don't accelerate too fast

        // SwerveDriveKinematicsConstraint autoKinematicsConstraint = new SwerveDriveKinematicsConstraint(swerveSubsystem.getKinematics(), Constants.AutoConstants.kMaxSpeedMetersPerSecond);

        // TrajectoryConfig config = new TrajectoryConfig(
        //     Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //     Constants.AutoConstants.kMaxAngularAccelerationDegreesPerSecondSquared)
        //     .setKinematics(swerveSubsystem.getKinematics())
        //     .addConstraint(autoKinematicsConstraint);

        // Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(

        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //     new Pose2d(3, 0, new Rotation2d(0)),
        //     config);

        // RamseteCommand ramseteCommand =

        // new RamseteCommand(

        //     testTrajectory,
        //     swerveSubsystem::getPose,
        //     new RamseteController(Constants.PIDConstants.kRamseteB, Constants.PIDConstants.kRamseteZeta),
        //     new SimpleMotorFeedforward(
        //         Constants.PIDConstants.ksVolts,
        //         Constants.PIDConstants.kvVoltSecondsPerMeter,
        //         Constants.PIDConstants.kaVoldSecondsSquaredPerMeter),
        //     DriveConstants.kDriveKinematics,
        //     swerveSubsystem::getWheelSpeeds,
        //     new PIDController(Constants.PIDConstants.kPDriveVel, 0, 0),
        //     new PIDController(Constants.PIDConstants.kPDriveVel, 0, 0),
            
        //     // These two PID controllers were added because I was under the impression that we need one for each wheel.
            
        //     new PIDController(Constants.PIDConstants.kPDriveVel, 0, 0),
        //     new PIDController(Constants.PIDConstants.kPDriveVel, 0, 0),
        //     // RamseteCommand passes volts to the callback
        //     m_robotDrive::tankDriveVolts,
        //     swerveSubsystem
        //     );

        // return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
        // .andThen(ramseteCommand)
        // .andThen(Commands.runOnce(() -> swerveSubsystem.stopModules()));

        return Commands.runOnce( () -> swerveSubsystem.zeroHeading())
        .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
            new Rotation2d(0), 
            new Rotation2d(0),
            new Rotation2d(0),
            new Rotation2d(0)}))
        .andThen( () -> swerveSubsystem.resetOdometry())
        .andThen(new DriveToPointCommand(swerveSubsystem, new Transform2d(new Translation2d(-3 * Constants.DriveConstants.kNavxUnitsToMetersConversion, 0 * Constants.DriveConstants.kNavxUnitsToMetersConversion), new Rotation2d(0)), 0.05))
        .andThen( () -> {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        })
        .andThen(new DriveToPointCommand(swerveSubsystem, new Transform2d(new Translation2d( 0 * Constants.DriveConstants.kNavxUnitsToMetersConversion, 1 * Constants.DriveConstants.kNavxUnitsToMetersConversion), new Rotation2d(0)), 0.05));

        
    }
}