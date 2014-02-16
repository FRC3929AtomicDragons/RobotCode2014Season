/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {

    //ACTUATORS DECLARATION
    RobotDrive drive;
    Victor intake;
    Jaguar shooterMotor1;
    Jaguar shooterMotor2;

    //SENSORS DECLARATION
    Joystick logi1;
    Encoder driveLeftEncoder;
    Encoder driveRightEncoder;
    Encoder shooterEncoder;
    Gyro gyro;
    Timer timer;

    /*SHOOTER CONSTANTS*/
    private double E1 = 325.0;
    private double E2 = 375.0;
    private double POWER_LEVEL = 1.0;

    final int AUTO_SHOOTER_UP = 1;
    final int AUTO_SHOOTER_DOWN = 2;
    final int MAN_SHOOTER = 0;
    int shooterMode = MAN_SHOOTER;

    /*AUTONOMOUS CONSTANTS*/
    final int DRIVE_STRAIGHT = 0;
    final int AUTO_SHOOT = 1;
    final int FINISHED = 2;

    int autoMode = DRIVE_STRAIGHT;

    /*DRIVE CONSTANTS*/
    // Filter constants for velocity filter
    private final double ALPHA_VEL = 0.2;
    private final double VELOCITY_INNOVATION_LIMIT = 10000.0;
    private double TARGET_DISTANCE = -100.0;
    private double TARGET_VELOCITY = -100.0;
    public static final double DRIVE_ENCODER_DPP = ((Math.PI) * (2.54) * (4)) / (256.0);
    public static final double SHOOTER_ENCODER_DPP = 1;

    private double velEstimate = 0.0;

    /*PID CONSTANTS*/
    double kPvel = 1.0E-2;
    double kIvel = 0.0;
    double kDvel = 0.0;
    double kPgyro = 1.0E-2;
    double kIgyro = 0.0;
    double kDgyro = 0.0;
    PIDTool pidVel;
    PIDTool pidGyro;

    public void robotInit() {
        //initializations
        logi1 = new Joystick(1);
        shooterMotor1 = new Jaguar(6);
        shooterMotor2 = new Jaguar(7);
        intake = new Victor(5);
        timer = new Timer();
        gyro = new Gyro(2);
        driveLeftEncoder = new Encoder(1, 2, true);
        driveRightEncoder = new Encoder(3, 4, false);
        shooterEncoder = new Encoder(5, 6, true);
        drive = new RobotDrive(1, 2, 3, 4);
        pidVel = new PIDTool(kPvel, kIvel, kDvel, TARGET_VELOCITY);
        pidGyro = new PIDTool(kPgyro, kIgyro, kDgyro, 0.0);

        //Set up encoders + gyro correctly
        SmartDashboard.putNumber("e1", E1);
        SmartDashboard.putNumber("e2", E2);
        SmartDashboard.putNumber("powerLevel", POWER_LEVEL);
        SmartDashboard.putNumber("targetDistance", TARGET_DISTANCE);
        SmartDashboard.putNumber("targetVelocity", TARGET_VELOCITY);

        // shooterEncoder.setReverseDirection(true);
        // driveLeftEncoder.setReverseDirection(true);
        shooterEncoder.setDistancePerPulse(SHOOTER_ENCODER_DPP);
        driveLeftEncoder.setDistancePerPulse(DRIVE_ENCODER_DPP);
        driveRightEncoder.setDistancePerPulse(DRIVE_ENCODER_DPP);
        gyro.setSensitivity(0.007);
        resetDriveEncoders();
        startDriveEncoders();
        shooterEncoder.reset();
        shooterEncoder.start();
    }

    public void autonomousInit() {
        teleopInit();
        autoMode = DRIVE_STRAIGHT;
        shooterMode = AUTO_SHOOTER_UP;
        gyro.reset();
        intake.set(1.0);
    }

    public void autonomousPeriodic() {
        switch (autoMode) {
            case DRIVE_STRAIGHT:
                driveStraight(TARGET_VELOCITY);

                if (driveLeftEncoder.getDistance() <= TARGET_DISTANCE) {
                    Timer.delay(5);
                    autoMode = AUTO_SHOOT;
                    shooterMode = AUTO_SHOOTER_UP;
                }
                break;
            case AUTO_SHOOT:
                autoShooterPeriodic();
                if (shooterMode == MAN_SHOOTER) {
                    autoMode = FINISHED;
                }
                break;
            case FINISHED:
                intake.set(0.0);
                break;
        }
    }

    public void teleopInit() {
        shooterEncoder.reset();
        resetDriveEncoders();
        startDriveEncoders();
        E1 = SmartDashboard.getNumber("e1");
        E2 = SmartDashboard.getNumber("e2");
        TARGET_DISTANCE = SmartDashboard.getNumber("targetDistance");
        TARGET_VELOCITY = SmartDashboard.getNumber("targetVelocity");
        POWER_LEVEL = SmartDashboard.getNumber("powerLevel");
        shooterEncoder.start();
    }

    public void teleopPeriodic() {
        if(logi1.getRawButton(1)){
            drivePeriodic(true);
        }
        else {
            drivePeriodic(false);
        }
        intakePeriodic();
        shooterPeriodic();
        SmartDashboard.putNumber("right Encoder Rate", driveRightEncoder.getRate());
        SmartDashboard.putNumber("left Encoder rate", driveLeftEncoder.getRate());
        SmartDashboard.putNumber("gyro", gyro.getAngle());
        SmartDashboard.putNumber("Shooter Encoder Rate",shooterEncoder.getRate() );
        
    }

    public void testInit() {
        //Put encoders and gyro on live window. I think we can comment out for competitions
        driveLeftEncoder.startLiveWindowMode();
        driveRightEncoder.startLiveWindowMode();
        shooterEncoder.startLiveWindowMode();
        gyro.startLiveWindowMode();
    }

    public void testPeriodic() {
        System.out.println("gyro" + gyro.getAngle());
        System.out.println("shooterEncoder" + shooterEncoder.getDistance());
        System.out.println("rightEncoder" + driveRightEncoder.getRate());
        System.out.println("leftEncoder" + driveLeftEncoder.getRate());
        gyro.updateTable();
        shooterEncoder.updateTable();
        driveRightEncoder.updateTable();
        driveLeftEncoder.updateTable();

        if (logi1.getRawButton(10)) {
            shooterEncoder.reset();
        }
    }

    public void drivePeriodic(boolean turbo) {
        if (!turbo) {
            drive.arcadeDrive(0.5 * logi1.getY(), -logi1.getX() * 0.5);
        } else {
            drive.arcadeDrive(1.0 * logi1.getY(), -logi1.getX() * 1.0);
        }
    }

    public void driveStraight(double targetVelocity) {
        double angle, velControl, gyroControl;

        updateFilteredVelocity();

        angle = gyro.getAngle();
        pidVel.setSetpoint(targetVelocity);
        velControl = pidVel.computeControl(velEstimate);
        gyroControl = pidGyro.computeControl(angle);

        setMotorCD(velControl, gyroControl);

        SmartDashboard.putNumber("right Encoder Rate", driveRightEncoder.getRate());
        SmartDashboard.putNumber("left Encoder rate", driveLeftEncoder.getRate());
        System.out.println("vel estimate " + velEstimate);
        System.out.println("vel control " + velControl);
    }

    public void intakePeriodic() {
        if (logi1.getRawButton(3)) {
            intake.set(1.0);
        } else if (logi1.getRawButton(2)) {
            intake.set(-1.0);
        } else {
            intake.set(0.0);
        }
    }

    public void shooterPeriodic() {
        System.out.println("shooterMode : " + shooterMode);
        System.out.println("shooter encoder: " + shooterEncoder.getDistance());
        switch (shooterMode) {
            case MAN_SHOOTER:
                manShooterPeriodic();
                break;
            case AUTO_SHOOTER_DOWN:
            case AUTO_SHOOTER_UP:
                autoShooterPeriodic();
                break;
        }
    }

    public void manShooterPeriodic() {
        //eventually might put limit switch conditional in
        if (logi1.getRawButton(4)) {
            shooterMotorsSet(1.0);
        } else if (logi1.getRawButton(5)) {
            shooterMotorsSet(-0.45);
        } else {
            shooterMotorsSet(0.0);
        }

        System.out.println("shooter encoder dist: " + shooterEncoder.getDistance());
        if (logi1.getRawButton(9)) {
            shooterEncoder.reset();
            shooterMode = AUTO_SHOOTER_UP;
        }
    }

    public void autoShooterPeriodic() {
        //stop catapult based on encoder feedback on distance
        double eDist = shooterEncoder.getDistance();

        //OVERRIDE AUTONOMOUS BUTTON
        if (logi1.getRawButton(10)) {
            shooterMode = MAN_SHOOTER;
        }
        if (shooterMode == AUTO_SHOOTER_UP) {
            if ((eDist < E1)) {
                shooterMotorsSet(POWER_LEVEL);
            } else if ((eDist > E1) && (eDist < E2)) {
                shooterMotorsSet(0.75 * POWER_LEVEL);
            } else if (eDist > E2) {
                shooterMotorsSet(0.0);
                shooterMode = AUTO_SHOOTER_DOWN;
            }
        } else {
            if ((eDist > 10.0)) {
                shooterMotorsSet(-0.15);
            } else if (eDist < 10.0) {
                shooterMotorsSet(0.0);
                shooterMode = MAN_SHOOTER;
            }
        }
    }

    public void shooterMotorsSet(double set) {
        //reverse motors
        shooterMotor1.set(-set);
        shooterMotor2.set(-set);
    }

    public void startDriveEncoders() {
        driveRightEncoder.start();
        driveLeftEncoder.start();
    }

    public void stopDriveEncoders() {
        driveRightEncoder.stop();
        driveLeftEncoder.stop();
    }

    public void resetDriveEncoders() {
        driveRightEncoder.reset();
        driveLeftEncoder.reset();
    }

    /*Used for auto-driving*/
    public void setMotorCD(double common, double differential) {
        drive.setLeftRightMotorOutputs(common - differential, common + differential);
    }

    public void updateFilteredVelocity() {
        // Read current velocity estimate
        double leftEncoderRate = driveLeftEncoder.getRate();
        double rightEncoderRate = driveRightEncoder.getRate();
        double avgVelocity = ((0.5) * ((leftEncoderRate) + (rightEncoderRate)));

        if (Math.abs(avgVelocity - velEstimate) <= VELOCITY_INNOVATION_LIMIT) {
            velEstimate = (ALPHA_VEL * avgVelocity) + ((1 - ALPHA_VEL) * velEstimate);
        }
    }
}
