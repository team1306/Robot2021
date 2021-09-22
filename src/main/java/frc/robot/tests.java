package frc.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class tests {
    static Translation2d frontLeftWheel = new Translation2d(Constants.ROBOT_DISTANCE_BETWEEN_WHEELS, Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2);
    static Translation2d frontRightWheel = new Translation2d(Constants.ROBOT_DISTANCE_BETWEEN_WHEELS, -Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2);
    static Translation2d backLeftWheel = new Translation2d(-Constants.ROBOT_DISTANCE_BETWEEN_WHEELS, Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2);
    static Translation2d backRightWheel = new Translation2d(-Constants.ROBOT_DISTANCE_BETWEEN_WHEELS, -Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2);
    public static void main(String[] args) {
        System.out.println( chasisSpeedsPrintOut(chasisSpeedsTest(.05,-.05,0)));
    }


    public static double[] chasisSpeedsTest(double xSpeed, double ySpeed, double turn) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turn);

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.FASTEST_SPEED_METERS);

        double[] returnValue = new double[8];

        for(int i = 0; i < 4; i++) {
            moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], Rotation2d.fromDegrees(0));
            returnValue[i] = moduleStates[i].speedMetersPerSecond;
            returnValue[i + 4] = moduleStates[i].angle.getDegrees() * Constants.DEGREES_TO_ENCODER_TICKS;
        }

        return returnValue;
    }

    public static String chasisSpeedsPrintOut(double[] chasisSpeeds){
        String output = "Wheel Speeds: ";
        for(int i = 0; i < 4; i++) {
            output += chasisSpeeds[i] + "\n";
        }
        output += "Angle Values: ";
        for(int i = 4; i < 8; i++) {
            output+= chasisSpeeds[i] + "\n";
        }
        return output;
    }
}


