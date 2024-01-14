// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;


public class Drivetrain extends SubsystemBase {
  //Compbot motor controllers
  //private final CANSparkMax m_frontLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
  //private final CANSparkMax m_rearLeft = new CANSparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushless);
  //private final CANSparkMax m_frontRight = new CANSparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
  //private final CANSparkMax m_rearRight = new CANSparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushless);

  //private MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);


  //Testbed motor controllers
  private final WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(DriveConstants.kFrontLeftMotorPort);
  private final WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(DriveConstants.kRearLeftMotorPort);
  private final WPI_VictorSPX m_frontRight = new WPI_VictorSPX(DriveConstants.kFrontRightMotorPort);
  private final WPI_VictorSPX m_rearRight = new WPI_VictorSPX(DriveConstants.kRearRightMotorPort);

  private final MecanumDrive m_drive =
    new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);



  //Encooders
  
    // Compbot encoders
    //private final RelativeEncoder m_frontLeftEncoder; 
    //private final RelativeEncoder m_rearLeftEncoder; 
    //private final RelativeEncoder m_frontRightEncoder; 
    //private final RelativeEncoder m_rearRightEncoder;  

  // The front-left-side drive encoder
  private final Encoder m_frontLeftEncoder =
      new Encoder(
          DriveConstants.kFrontLeftEncoderPorts[0],
          DriveConstants.kFrontLeftEncoderPorts[1],
          DriveConstants.kFrontLeftEncoderReversed);

  // The rear-left-side drive encoder
  private final Encoder m_rearLeftEncoder =
      new Encoder(
          DriveConstants.kRearLeftEncoderPorts[0],
          DriveConstants.kRearLeftEncoderPorts[1],
          DriveConstants.kRearLeftEncoderReversed);

  // The front-right--side drive encoder
  private final Encoder m_frontRightEncoder =
      new Encoder(
          DriveConstants.kFrontRightEncoderPorts[0],
          DriveConstants.kFrontRightEncoderPorts[1],
          DriveConstants.kFrontRightEncoderReversed);

  // The rear-right-side drive encoder
  private final Encoder m_rearRightEncoder =
      new Encoder(
          DriveConstants.kRearRightEncoderPorts[0],
          DriveConstants.kRearRightEncoderPorts[1],
          DriveConstants.kRearRightEncoderReversed);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();



  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(
          PhysicalConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new MecanumDriveWheelPositions());

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    // Calibrate gyro
      new Thread(() -> {
        try {
          Thread.sleep(1000);
          zeroHeading();
        } catch (Exception e){
        }
      }).start();


    //Register values for dashboard
    SendableRegistry.addChild(m_drive, m_frontLeft);
    SendableRegistry.addChild(m_drive, m_rearLeft);
    SendableRegistry.addChild(m_drive, m_frontRight);
    SendableRegistry.addChild(m_drive, m_rearRight);
    //TODO: add other sensors? (encoders, gyro, etc.)

    //Compbot encoders
    //m_frontLeftEncoder = m_frontLeft.getEncoder();
    //m_rearLeftEncoder = m_rearLeft.getEncoder();
    //m_frontRightEncoder = m_frontRight.getEncoder();
    //m_rearRightEncoder = m_rearRight.getEncoder();

    //Compbot Factory reset motor controllers 
    //m_frontLeft.restoreFactoryDefaults();
    //m_rearLeft.restoreFactoryDefaults();
    //m_frontRight.restoreFactoryDefaults();
    //m_rearRight.restoreFactoryDefaults();

    //Testbed Factory reset motor controllers
    m_frontLeft.configFactoryDefault();
    m_rearLeft.configFactoryDefault();
    m_frontRight.configFactoryDefault();
    m_rearRight.configFactoryDefault();

    // Sets the distance per pulse for the encoders
    //Compbot
    //m_frontLeftEncoder.setPositionConversionFactor(PhysicalConstants.kEncoderDistancePerPulse);
    //m_rearLeftEncoder.setPositionConversionFactor(PhysicalConstants.kEncoderDistancePerPulse);
    //m_frontRightEncoder.setPositionConversionFactor(PhysicalConstants.kEncoderDistancePerPulse);
    //m_rearRightEncoder.setPositionConversionFactor(PhysicalConstants.kEncoderDistancePerPulse);

    //Testbed
    m_frontLeftEncoder.setDistancePerPulse(PhysicalConstants.kEncoderDistancePerPulse);
    m_rearLeftEncoder.setDistancePerPulse(PhysicalConstants.kEncoderDistancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(PhysicalConstants.kEncoderDistancePerPulse);
    m_rearRightEncoder.setDistancePerPulse(PhysicalConstants.kEncoderDistancePerPulse);
    
    //Invert motors on one side
    m_frontLeft.setInverted(DriveConstants.kFrontLeftMotorReversed);
    m_rearLeft.setInverted(DriveConstants.kRearLeftMotorReversed);
    m_frontRight.setInverted(DriveConstants.kFrontRightMotorReversed);
    m_rearRight.setInverted(DriveConstants.kRearRightMotorReversed);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getCurrentWheelDistances());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
    } else {
      m_drive.driveCartesian(xSpeed, ySpeed, rot);
    }
  }

  /* Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    //Compbot
    //m_frontLeftEncoder.setPosition(0);
    //m_rearLeftEncoder.setPosition(0);
    //m_frontRightEncoder.setPosition(0);
    //m_rearRightEncoder.setPosition(0);

    //Testbed
    m_frontLeftEncoder.reset();
    m_rearLeftEncoder.reset();
    m_frontRightEncoder.reset();
    m_rearRightEncoder.reset();
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public Encoder getFrontLeftEncoder() {
    //Compbot
    //return m_frontLeftEncoder.getEncoder();

    //Testbed
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public Encoder getRearLeftEncoder() {
    //Compbot
    //return m_rearLeftEncoder.getEncoder();

    //Testbed
    return m_rearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public Encoder getFrontRightEncoder() {
    //Compbot
    //return m_frontRightEncoder.getEncoder();

    //Testbed
    return m_frontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public Encoder getRearRightEncoder() {
    //Compbot
    //return m_rearRightEncoder.getEncoder();

    //Testbed
    return m_rearRightEncoder;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(

        //Compbot
        //m_frontLeftEncoder.getVelocity(),
        //m_rearLeftEncoder.getVelocity(),
        //m_frontRightEncoder.getVelocity(),
        //m_rearRightEncoder.getVelocity());

        //Testbed
        m_frontLeftEncoder.getRate(),
        m_rearLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_rearRightEncoder.getRate());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {

    //Compbot
    //return new MecanumDriveWheelPositions(
    //    m_frontLeftEncoder.getPosition(),
    //    m_rearLeftEncoder.getPosition(),
    //    m_frontRightEncoder.getPosition(),
    //    m_rearRightEncoder.getPosition());  

    //Testbed
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getDistance(),
        m_rearLeftEncoder.getDistance(),
        m_frontRightEncoder.getDistance(),
        m_rearRightEncoder.getDistance());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public boolean getFieldRelative() {
    return m_gyro.isConnected();
  }

}
