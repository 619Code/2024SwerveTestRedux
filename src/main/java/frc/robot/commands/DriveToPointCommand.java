package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPointCommand extends Command {

    private SwerveSubsystem swerve;
    private Translation2d firstPos, secondPos, ΔPos;
    private Rotation2d firstRot, secondRot, ΔRot;
    private double baseMaxSpeed, rotationMaxSpeed; 
    private double calculatedXSpeed, calculatedYSpeed, calculatedRotSpeed; 

    public DriveToPointCommand(SwerveSubsystem subsystem, Translation2d changeinPosMeters, Rotation2d changeinHeadingRads, double percentageOfMaxSpeed) {

        swerve = subsystem;
        ΔPos = changeinPosMeters;
        ΔRot = changeinHeadingRads;

        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        firstPos = swerve.getPose().getTranslation();
        firstRot = swerve.getRotation2d();

        secondPos = firstPos.plus(ΔPos);
        secondRot = firstRot.plus(ΔRot);

        double dx, dy, dθ;


        // Get the time it would take for each item to reach the destination if it is at the max speed

        dx = secondPos.getX() - firstPos.getX();
        dy = secondPos.getY() - firstPos.getY();
        dθ = secondRot.getRadians() - firstRot.getRadians();

        // The times it would take for each item of interest to reach it's destination if it moves at it's max speed

        double xSecMax, ySecMax, tSecMax;

        xSecMax = dx / baseMaxSpeed;
        ySecMax = dy / baseMaxSpeed;
        tSecMax = dθ / baseMaxSpeed;

        //  Get the shortest of these times. In other words, the operation should be performed as fast as the slowest movement can go

        double bottleneckTime = Math.max(Math.max(xSecMax, ySecMax), tSecMax);
        calculatedXSpeed = dx * bottleneckTime; 
        calculatedYSpeed = dy * bottleneckTime; 
        calculatedRotSpeed = dθ * bottleneckTime;


        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(calculatedXSpeed, calculatedYSpeed, calculatedRotSpeed, swerve.getRotation2d());
        swerve.setModuleStates(
            DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds)
        );

    }

    @Override
    public void execute() {
        System.out.println(":3c");
    }

    @Override
    public boolean isFinished() {

        return (swerve.getPose().getTranslation().getDistance(secondPos) <= 0.1);

    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }
}
