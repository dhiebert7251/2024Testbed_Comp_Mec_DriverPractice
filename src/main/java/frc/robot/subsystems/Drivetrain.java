// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;

import java.time.Instant;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;



public class Drivetrain extends SubsystemBase {
    // Shuffleboard
    private final ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain Data");
    private GenericEntry headingEntry;
    private GenericEntry turnRateEntry;
    private GenericEntry fieldRelativeEntry;

    private GenericEntry FLRateEntry;
    private GenericEntry RLRateEntry;
    private GenericEntry FRRateEntry;
    private GenericEntry RRRateEntry;

    private GenericEntry FLDistanceEntry;
    private GenericEntry RLDistanceEntry;
    private GenericEntry FRDistanceEntry;
    private GenericEntry RRDistanceEntry;

    private GenericEntry FLPositionEntry;
    private GenericEntry RLPositionEntry;
    private GenericEntry FRPositionEntry;
    private GenericEntry RRPositionEntry;

    private GenericEntry FLVoltageEntry;
    private GenericEntry RLVoltageEntry;
    private GenericEntry FRVoltageEntry;
    private GenericEntry RRVoltageEntry;

    private GenericEntry Joy1XEntry;
    private GenericEntry Joy1YEntry;
    private GenericEntry Joy2XEntry;


  
  public void log(String message) {
    System.out.println(Instant.now() + ": " + message);
  }

  //Testbed motor controllers
  private final WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(DriveConstants.kFrontLeftMotorPort);
  private final WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(DriveConstants.kRearLeftMotorPort);
  private final WPI_TalonSRX m_frontRight = new WPI_TalonSRX(DriveConstants.kFrontRightMotorPort);
  private final WPI_VictorSPX m_rearRight = new WPI_VictorSPX(DriveConstants.kRearRightMotorPort);

  private final MecanumDrive m_drive =
    new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);

  private Field2d field = new Field2d();


  //Encoders
   

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

  // Wheel PID controllers
  private final PIDController m_frontLeftPIDController =
      new PIDController(PhysicalConstants.kPFrontLeft, 
                        PhysicalConstants.kIFrontLeft,
                        PhysicalConstants.kDFrontLeft);

  private final PIDController m_rearLeftPIDController = 
      new PIDController(PhysicalConstants.kPRearLeft, 
                        PhysicalConstants.kIRearLeft,
                        PhysicalConstants.kDRearLeft);

  private final PIDController m_frontRightPIDController =
      new PIDController(PhysicalConstants.kPFrontRight, 
                        PhysicalConstants.kIFrontRight,
                        PhysicalConstants.kDFrontRight);

  private final PIDController m_rearRightPIDController =
      new PIDController(PhysicalConstants.kPRearRight, 
                        PhysicalConstants.kIRearRight,
                        PhysicalConstants.kDRearRight);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();



  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(
          PhysicalConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new MecanumDriveWheelPositions());

  // The feedforward for the drive TODO: how do you tune this?
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

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

    // Initialize Shuffleboard widgets
    headingEntry = driveTab.add("Heading", 0).withWidget("Gyro").withPosition(0, 0).withSize(2, 2).getEntry();
    turnRateEntry = driveTab.add("Turn Rate", 0).withWidget("Dial").withPosition(2, 0).withSize(2, 2).getEntry();
    fieldRelativeEntry = driveTab.add("Field Relative", false).withWidget("Boolean Box").withPosition(4, 0).withSize(1, 1).getEntry();
  
    // Initialize Shuffleboard widgets for encoder positions
    FLPositionEntry = driveTab.add("FL Encoder Position", 0).getEntry();
    RLPositionEntry = driveTab.add("RL Encoder Position", 0).getEntry();
    FRPositionEntry = driveTab.add("FR Encoder Position", 0).getEntry();
    RRPositionEntry = driveTab.add("RR Encoder Position", 0).getEntry();

    // Initialize Shuffleboard widgets for encoder distances
    FLDistanceEntry = driveTab.add("FL Encoder Distance", 0.0).getEntry();
    RLDistanceEntry = driveTab.add("RL Encoder Distance", 0.0).getEntry();
    FRDistanceEntry = driveTab.add("FR Encoder Distance", 0.0).getEntry();
    RRDistanceEntry = driveTab.add("RR Encoder Distance", 0.0).getEntry();

    // Initialize Shuffleboard widgets for voltages
    FLVoltageEntry = driveTab.add("FL Voltage", 0.0).getEntry();
    RLVoltageEntry = driveTab.add("RL Voltage", 0.0).getEntry();
    FRVoltageEntry = driveTab.add("FR Voltage", 0.0).getEntry();
    RRVoltageEntry = driveTab.add("RR Voltage", 0.0).getEntry();

    // Initialize Shuffleboard widgets for encoder rates
    FLRateEntry = driveTab.add("FL Encoder Rate", 0.0).getEntry();
    RLRateEntry = driveTab.add("RL Encoder Rate", 0.0).getEntry();
    FRRateEntry = driveTab.add("FR Encoder Rate", 0.0).getEntry();
    RRRateEntry = driveTab.add("RR Encoder Rate", 0.0).getEntry();

    // Initialize Shuffleboard widgets for joystick values
    Joy1XEntry = driveTab.add("Joy1 X", 0.0).getEntry();
    Joy1YEntry = driveTab.add("Joy1 Y", 0.0).getEntry();
    Joy2XEntry = driveTab.add("Joy2 X", 0.0).getEntry();



    //Testbed Factory reset motor controllers
    m_frontLeft.configFactoryDefault();
    m_rearLeft.configFactoryDefault();
    m_frontRight.configFactoryDefault();
    m_rearRight.configFactoryDefault();

    //Testbed break/coast mode
    m_frontLeft.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
    m_rearLeft.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
    m_frontRight.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
    m_rearRight.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);

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


    
    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getChassisSpeeds, 
      this::driveFieldRelative, 
      AutoConstants.kPathFollowerConfig,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }

        return false;
      },
      this
    );
    

    
  }

/* Periodic
 * 
 * 
 * 
 * 
 */

  @Override
  public void periodic() {

    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getCurrentWheelDistances());

    // Update the field
    field.setRobotPose(getPose());

    showTelemetry();
  }

/*
 * 
 * 
 * 
 */

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  //TODO: update for aprilTag detection of current pose

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }

 /*  Drive methods
  * 
  *
  *
  *
  */

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    MecanumDriveWheelSpeeds targetStates = PhysicalConstants.kDriveKinematics.toWheelSpeeds(targetSpeeds);
    setSpeeds(targetStates);

  }


/**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
    double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
      var mecanumDriveWheelSpeeds =
          PhysicalConstants.kDriveKinematics.toWheelSpeeds(
             ChassisSpeeds.discretize(
                  fieldRelative
                      ? ChassisSpeeds.fromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                      : new ChassisSpeeds(xSpeed, ySpeed, rot),
                  periodSeconds));
      mecanumDriveWheelSpeeds.desaturate(PhysicalConstants.kMaxVelocity);
      setSpeeds(mecanumDriveWheelSpeeds);

      //log("Driving with speeds X: " + xSpeed + " Y: " + ySpeed + " Rot: " + rot);
      //SmartDashboard.putNumber("Joy1 X", xSpeed);
      //SmartDashboard.putNumber("Joy1 Y", ySpeed);
      //SmartDashboard.putNumber("Joy2 X", rot);

      Joy1XEntry.setDouble(xSpeed);
      Joy1YEntry.setDouble(ySpeed);
      Joy2XEntry.setDouble(rot);


  }





  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }




  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {

    //Testbed
    m_frontLeftEncoder.reset();
    m_rearLeftEncoder.reset();
    m_frontRightEncoder.reset();
    m_rearRightEncoder.reset();
  }


  public Encoder getFrontLeftEncoder() {
    //Testbed
    return m_frontLeftEncoder;
  }

  public Encoder getRearLeftEncoder() {
    //Testbed
    return m_rearLeftEncoder;
  }

  public Encoder getFrontRightEncoder() {
    //Testbed
    return m_frontRightEncoder;
  }


  public Encoder getRearRightEncoder() {

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


    //Testbed
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getDistance(),
        m_rearLeftEncoder.getDistance(),
        m_frontRightEncoder.getDistance(),
        m_rearRightEncoder.getDistance());
  }

  /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

    final double frontLeftOutput =
        m_frontLeftPIDController.calculate(
            m_frontLeftEncoder.getRate(), speeds.frontLeftMetersPerSecond);
    final double frontRightOutput =
        m_frontRightPIDController.calculate(
            m_frontRightEncoder.getRate(), speeds.frontRightMetersPerSecond);
    final double backLeftOutput =
        m_rearLeftPIDController.calculate(
            m_rearLeftEncoder.getRate(), speeds.rearLeftMetersPerSecond);
    final double backRightOutput =
        m_rearRightPIDController.calculate(
            m_rearRightEncoder.getRate(), speeds.rearRightMetersPerSecond);

    m_frontLeft.setVoltage(frontLeftOutput + frontLeftFeedforward);
    m_frontRight.setVoltage(frontRightOutput + frontRightFeedforward);
    m_rearLeft.setVoltage(backLeftOutput + backLeftFeedforward);
    m_rearRight.setVoltage(backRightOutput + backRightFeedforward);
    log("Setting speeds FL: " + speeds.frontLeftMetersPerSecond + " FR: " + speeds.frontRightMetersPerSecond + " ...");
  }


  public ChassisSpeeds getChassisSpeeds(){
    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds = PhysicalConstants.kDriveKinematics.toChassisSpeeds(getCurrentWheelSpeeds());
    return chassisSpeeds;
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


/*
    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
    */

  public void showTelemetry(){
    // Update the Shuffleboard entries with current values
    FLPositionEntry.setDouble(m_frontLeftEncoder.getRaw());
    RLPositionEntry.setDouble(m_rearLeftEncoder.getRaw());
    FRPositionEntry.setDouble(m_frontRightEncoder.getRaw());
    RRPositionEntry.setDouble(m_rearRightEncoder.getRaw());

    FLDistanceEntry.setDouble(m_frontLeftEncoder.getDistance());
    RLDistanceEntry.setDouble(m_rearLeftEncoder.getDistance());
    FRDistanceEntry.setDouble(m_frontRightEncoder.getDistance());
    RRDistanceEntry.setDouble(m_rearRightEncoder.getDistance());

    FLVoltageEntry.setDouble(m_frontLeft.getMotorOutputVoltage());
    RLVoltageEntry.setDouble(m_rearLeft.getMotorOutputVoltage());
    FRVoltageEntry.setDouble(m_frontRight.getMotorOutputVoltage());
    RRVoltageEntry.setDouble(m_rearRight.getMotorOutputVoltage());

    FLRateEntry.setDouble(m_frontLeftEncoder.getRate());
    RLRateEntry.setDouble(m_rearLeftEncoder.getRate());
    FRRateEntry.setDouble(m_frontRightEncoder.getRate());
    RRRateEntry.setDouble(m_rearRightEncoder.getRate());

    headingEntry.setDouble(getHeading());
    turnRateEntry.setDouble(getTurnRate());
    fieldRelativeEntry.setBoolean(getFieldRelative());




    
  
  }
}
