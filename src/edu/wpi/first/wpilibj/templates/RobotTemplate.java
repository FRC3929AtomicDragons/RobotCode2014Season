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
    Joystick driveLogi;
    Joystick opLogi;
    Encoder driveLeftEncoder;
    Encoder driveRightEncoder;
    Encoder shooterEncoder;
    Gyro gyro;
    Timer timer;

    /*SHOOTER CONSTANTS*/
    private double E1[] = {250.0, 200.0, 325.0};
    private double E2[] = {285.0, 225.0, 375.0};
    private double POWER_LEVEL[] = {0.9, 0.9, 1.0};
    int shotIndex = 0;

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
    private double TARGET_DISTANCE = -400.0;
    private double TARGET_VELOCITY = -125.0;
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
        driveLogi = new Joystick(1);
        opLogi = new Joystick(2);
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
        shotIndex = 0;
        SmartDashboard.putNumber("e1auto", E1[shotIndex]);
        SmartDashboard.putNumber("e2auto", E2[shotIndex]);
        SmartDashboard.putNumber("powerLevelauto", POWER_LEVEL[shotIndex]);
        SmartDashboard.putNumber("e1", E1[1]);
        SmartDashboard.putNumber("e2", E2[1]);
        SmartDashboard.putNumber("powerLevel", POWER_LEVEL[1]);
        SmartDashboard.putNumber("targetDistanceauto", TARGET_DISTANCE);
        SmartDashboard.putNumber("targetVelocityauto", TARGET_VELOCITY);

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
        shotIndex = 0;
        intake.set(1.0);
        Timer.delay(2);
    }

    public void autonomousPeriodic() {
        switch (autoMode) {
            case DRIVE_STRAIGHT:
                driveStraight(TARGET_VELOCITY);

                if (driveLeftEncoder.getDistance() <= TARGET_DISTANCE) {
                    Timer.delay(2);
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
        
        E1[0] = SmartDashboard.getNumber("e1auto");
        E2[0] = SmartDashboard.getNumber("e2auto");
        TARGET_DISTANCE = SmartDashboard.getNumber("targetDistanceauto");
        TARGET_VELOCITY = SmartDashboard.getNumber("targetVelocityauto");
        POWER_LEVEL[0] = SmartDashboard.getNumber("powerLevelauto");
        E1[1] = SmartDashboard.getNumber("e1");
        E2[1] = SmartDashboard.getNumber("e2");
        POWER_LEVEL[1] = SmartDashboard.getNumber("powerLevel");
        // Replicate parameters in shot 1
        shotIndex = 1;
        shooterEncoder.start();
    }

    public void teleopPeriodic() {
        drivePeriodic(driveLogi.getRawButton(1));
        intakePeriodic();
        shooterPeriodic();
        SmartDashboard.putNumber("right Encoder Rate", driveRightEncoder.getRate());
        SmartDashboard.putNumber("left Encoder rate", driveLeftEncoder.getRate());
        SmartDashboard.putNumber("gyro", gyro.getAngle());
        SmartDashboard.putNumber("Shooter Encoder Rate", shooterEncoder.getRate());

    }

    public void drivePeriodic(boolean turbo) {
        double scale = turbo ? 1.0 : 0.75;
        drive.arcadeDrive(scale * driveLogi.getY(), -scale * driveLogi.getX());
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
        if (opLogi.getRawButton(4)) {
            intake.set(1.0);
        } else if (opLogi.getRawButton(5)) {
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
        if (opLogi.getRawButton(6)) {
            shooterMotorsSet(1.0);
        } else if (opLogi.getRawButton(7)) {
            shooterMotorsSet(-0.45);
        } else {
            shooterMotorsSet(0.0);
        }
        if(opLogi.getRawButton(11)){
            shotIndex = 1;
        } else if(opLogi.getRawButton(10)){
            shotIndex = 2;
        }
        
        System.out.println("shooter encoder dist: " + shooterEncoder.getDistance());
        if (opLogi.getRawButton(3)) autoShot(1);
        if (opLogi.getRawButton(2)) autoShot(2);
    }
    
    public void autoShot(int index) {
        this.shotIndex = index;
        shooterEncoder.reset();
        shooterMode = AUTO_SHOOTER_UP;
    }

    public void autoShooterPeriodic() {
        //stop catapult based on encoder feedback on distance
        double eDist = shooterEncoder.getDistance();

        //OVERRIDE AUTONOMOUS BUTTON
        if (opLogi.getRawButton(2)) {
            shooterMode = MAN_SHOOTER;
        }
        if (shooterMode == AUTO_SHOOTER_UP) {
            if ((eDist < E1[shotIndex])) {
                shooterMotorsSet(POWER_LEVEL[shotIndex]);
            } else if ((eDist > E1[shotIndex]) && (eDist < E2[shotIndex])) {
                shooterMotorsSet(0.75 * POWER_LEVEL[shotIndex]);
            } else if (eDist > E2[shotIndex]) {
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
