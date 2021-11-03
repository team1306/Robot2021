/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.UserDigital;

/**
 * The SwerveDrive class uses four SwerveWheel objects which make up the[]\
 * drivetrain. This class is responsible for the math that keeps track of how
 * the swerve drive moves
 */
public class SwerveDrive extends SubsystemBase {
    // (speed motor ID, angle motor ID)
    public SwerveWheel frontRight = new SwerveWheel(Constants.K_DRIVE_FRONT_RIGHT_ID, Constants.K_TURN_FRONT_RIGHT_ID, Constants.K_ENCODER_FRONT_RIGHT_ID,false, Constants.K_FRONT_RIGHT_OFFSET,true);
    public SwerveWheel frontLeft = new SwerveWheel(Constants.K_DRIVE_FRONT_LEFT_ID, Constants.K_TURN_FRONT_LEFT_ID, Constants.K_ENCODER_FRONT_LEFT_ID, true, Constants.K_FRONT_LEFT_OFFSET,true);
    public SwerveWheel backRight = new SwerveWheel(Constants.K_DRIVE_BACK_RIGHT_ID, Constants.K_TURN_BACK_RIGHT_ID, Constants.K_ENCODER_BACK_RIGHT_ID, false, Constants.K_BACK_RIGHT_OFFSET,true);
    public SwerveWheel backLeft = new SwerveWheel(Constants.K_DRIVE_BACK_LEFT_ID, Constants.K_TURN_BACK_LEFT_ID, Constants.K_ENCODER_BACK_LEFT_ID, true, Constants.K_BACK_LEFT_OFFSET,true);


    Translation2d frontLeftWheel = new Translation2d(-Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2, Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2);
    Translation2d frontRightWheel = new Translation2d(Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2, Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2);
    Translation2d backLeftWheel = new Translation2d(-Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2, -Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2);
    Translation2d backRightWheel = new Translation2d(Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2, -Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2);

    public SwerveDriveOdometry odometry;
    private AHRS navx; //Gyro we use, navX Sensor

    public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel);
    private ChassisSpeeds chassisSpeeds;

    SwerveModuleState[] moduleStates;

    /**
     * Nothing needs to be done in the default constructor
     */
    public SwerveDrive() {      
    }

    private boolean FROn = true;
    private boolean FLOn = true;
    private boolean BROn = true;
    private boolean BLOn = true;
    
    /**
     * Creates four new SwerveModuleStates and assigns them to their respective
     * wheels
     * 
     * @param x    x-coordinate movement in meters per second
     * @param y    y-coordinate movement in meters per second
     * @param turn  rotation of the wheels in radians per second
     */
    public void driveTrain(
        double x, double y, double turn,
        boolean toggleFR, boolean toggleFL, boolean toggleBR, boolean toggleBL
    ) {
        //Converts from the x-coord, y-coord and turns into an array of module states
        chassisSpeeds = new ChassisSpeeds(y, x, turn);
        ChassisSpeeds chassisSpeeds2 = new ChassisSpeeds(y, x, -turn);
        moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveModuleState[] modulesStates2 = kinematics.toSwerveModuleStates(chassisSpeeds2);
        // making sure module states have possible values
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.FASTEST_SPEED_METERS);
        SwerveDriveKinematics.normalizeWheelSpeeds(modulesStates2, Constants.FASTEST_SPEED_METERS);
        //Getting and assigning the module states to the wheels
        
        /*

        NOTES FROM 10/30
        - code looks for button press and then inverses the current value, so if the wheel
            is activated, then deactivate it etc. However a prolonged button press (even 
            for less than a second) will invert the wheel state more than once. This can
            be demonstrated by holding down the inversion button while the wheels are 
            moving. To solve this issue, just press the button for a split second and then
            move on.
        - all wheels are moving forwards and backwards only, result of switching to arcade
            drive?
        */

        if (toggleFR) FROn = !FROn;
        if (toggleFL) FLOn = !FLOn;
        if (toggleBR) BROn = !BROn;
        if (toggleBL) BLOn = !BLOn;
        if (FLOn){
            SwerveModuleState frontLeftState = modulesStates2[0];
            //frontLeft.drive(x,y,turn);
            frontLeft.drive(frontLeftState);
        }
        
        if (FROn){
            SwerveModuleState frontRightState = moduleStates[1];
            //frontRight.drive(x,y,turn);
            frontRight.drive(frontRightState);
        }
        
        if (BLOn){
            SwerveModuleState backLeftState = moduleStates[2];
            //backLeft.drive(x,y,turn);
            backLeft.drive(backLeftState);
        }
        
        if (BROn){
            SwerveModuleState backRightState = modulesStates2[3];
            //backRight.drive(x,y,turn);
            backRight.drive(backRightState);
        }
        shuffleboard();
        
    }

    public void resetAllWheels() {
        frontLeft.resetAbsoluteZero();
        frontRight.resetAbsoluteZero();
        backLeft.resetAbsoluteZero();
        backRight.resetAbsoluteZero();
    }

    public double getAngle() {
        return navx.getAngle();
    } 

    public void setModuleStates(SwerveModuleState[] states) {
        moduleStates = states;
    }

    private void shuffleboard(){
        SmartDashboard.putBoolean("Back Right On?", BROn);
        SmartDashboard.putBoolean("Back Left On?", BLOn);
        SmartDashboard.putBoolean("Front Left On?", FLOn);
        SmartDashboard.putBoolean("Front Right On?", FROn);
        SmartDashboard.putBooleanArray("wheel states", new boolean[]{FLOn, FROn, BLOn, BROn});

    }

}