// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeTest extends SubsystemBase {

  // Subsystem parameters
  private static final int kPivotLeft = 9;
  private static final int kPivotRight = 10;
  private static final int kRollerId = 0;

  private static final double kNativeToRad = Math.PI / 2.0 / 12.0;
  private static final double kNominalVolt = 10.0;

  private static final double kNeutralRad = Math.toRadians(-35.0);
  private static final double kHorizontalPercent = 0.06;

  private static final double kTargetTolRad = Math.toRadians(5.0);
  private static final double kP = 0.003; //have to continue tweaking
  private static final double kI = 0.0;
  private static final double kD = 0.006;

  private static final double kAngVelRad = Math.toRadians(40.0); //used to be 45
  private static final double kAngAccRed = Math.toRadians(20.0); //used to be 90

  private static final double kRetractRad = Math.toRadians(45.0);
  private static final double kExtendRad = Math.toRadians(0.0);
  private static final double kRollerPercent = 0.5;

  // Member objects
  private final CANSparkMax m_pivotLeft = new CANSparkMax(kPivotLeft, MotorType.kBrushless);
  private final CANSparkMax m_pivotRight = new CANSparkMax(kPivotRight, MotorType.kBrushless);
  //private final CANSparkMax m_roller = new CANSparkMax(kRollerId, MotorType.kBrushless);

  private final SparkMaxPIDController m_pidLeft = m_pivotLeft.getPIDController();
  private final RelativeEncoder m_encoderLeft = m_pivotLeft.getEncoder();
  private final SparkMaxPIDController m_pidRight = m_pivotRight.getPIDController();
  private final RelativeEncoder m_encoderRight = m_pivotRight.getEncoder();

  private final ShuffleboardTab m_intakeTab = Shuffleboard.getTab("intake");
  private final GenericEntry m_percentEntry = m_intakeTab.add("targetPercent", 0.0).getEntry();
  private final GenericEntry m_angleEntry = m_intakeTab.add("currentAngle", 0.0).getEntry();
  private final GenericEntry m_targetEntry =  m_intakeTab.add("targetAngle", 0.0).getEntry();
  private final GenericEntry m_pidEntry = m_intakeTab.add("feedForward", 0.0).getEntry();


  // Process variables
  private double m_targetRad = kNeutralRad;


  /** Creates a new Intake. */
  public IntakeTest() {

   // m_roller.restoreFactoryDefaults();
    //m_roller.burnFlash();

    m_pivotLeft.restoreFactoryDefaults();
    m_pivotLeft.setInverted(true);
    m_pivotLeft.enableVoltageCompensation(kNominalVolt);

    m_encoderLeft.setPositionConversionFactor(kNativeToRad);
    m_encoderLeft.setPosition(kNeutralRad);

    m_pidLeft.setP(kP);
    m_pidLeft.setD(kD);
    m_pidLeft.setI(kI);

    m_pidLeft.setSmartMotionMaxVelocity(kAngVelRad, 0);
    m_pidLeft.setSmartMotionMaxAccel(kAngAccRed, 0);

    

    m_pivotLeft.burnFlash();

    m_pivotRight.restoreFactoryDefaults();
    m_pivotRight.enableVoltageCompensation(kNominalVolt);

    m_encoderRight.setPositionConversionFactor(kNativeToRad);
    m_encoderRight.setPosition(kNeutralRad);

    m_pidRight.setP(kP);
    m_pidRight.setD(kD);
    m_pidRight.setI(kI);

    m_pidRight.setSmartMotionMaxVelocity(kAngVelRad, 0);
    m_pidRight.setSmartMotionMaxAccel(kAngAccRed, 0);

    m_pivotRight.burnFlash();

    // Stop intake by default
    // this.setDefaultCommand(this.stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Set target to current when robot is disabled to prevent sudden motion on enable
    if (DriverStation.isDisabled()) {
      m_targetRad = m_encoderLeft.getPosition();
      //m_targetRad = m_encoderRight.getPosition();
     

    }

    // Set reference in periodic to allow for arbitrary PID computation
    m_pidLeft.setReference(m_targetRad, ControlType.kSmartMotion, 0, this.computeFeedForward( m_encoderLeft.getPosition()), ArbFFUnits.kPercentOut);
    m_pidRight.setReference(m_targetRad, ControlType.kSmartMotion, 0, this.computeFeedForward( m_encoderRight.getPosition()), ArbFFUnits.kPercentOut);

  

   m_angleEntry.setDouble(Math.toDegrees(m_encoderLeft.getPosition()));
   m_percentEntry.setDouble(m_pivotLeft.getAppliedOutput());
   m_targetEntry.setDouble(Math.toDegrees(m_targetRad));
   m_pidEntry.setDouble(this.computeFeedForward(m_encoderLeft.getPosition()));

   System.out.printf("Pos L %.2f    Pos R %.2f    Pos* %.0f    FF %.2f    Cmd %.2f\n", Math.toDegrees(m_encoderLeft.getPosition()), Math.toDegrees(m_encoderRight.getPosition()), Math.toDegrees(m_targetRad), this.computeFeedForward(m_encoderLeft.getPosition()), m_pivotLeft.getAppliedOutput());
  }

  /**
   * Compute arbitrary feed-forward for current intake position
   *
   * @return feed-forward (percent)
   */
  private double computeFeedForward(double angle) {
    return kHorizontalPercent * Math.cos(angle);
    
  }

  /**
   * Check if the intake has reached its target angle
   *
   * @return intake is on target
   */
  private boolean onTarget() {
    return Math.abs(m_encoderLeft.getPosition() - m_targetRad) < kTargetTolRad
    & Math.abs(m_encoderRight.getPosition() - m_targetRad) < kTargetTolRad;
  }

  /**
   * Set the roller speed and pivot to a given angle
   *
   * @param rollerPercent roller speed (percent)
   * @param pivotRad angle above horizontal (rad)
   * @return blocking command
   */
   private Command operate(double rollerPercent, double pivotRad) {
    //return this.runOnce(() -> m_roller.set(rollerPercent))
        //.andThen(this.run(() -> m_targetRad = pivotRad).until(this::onTarget));

        return this.run(() -> m_targetRad = pivotRad).until(this::onTarget);

  }

  /**
   * Start the roller and extend the intake
   *
   * @return blocking command
   */
  public Command extend() {
    return this.operate(kRollerPercent, kExtendRad);
  }

  /**
   * Stop the roller and retract the intake
   *
   * @return blocking command
   */
  public Command retract() {
    return this.operate(0.0, kRetractRad);
  }

  /**
   * Stop the roller
   *
   * @return instant command
   */
  public Command stop() {
    //return this.runOnce(() -> m_roller.set(0.0));
    return this.runOnce(() -> System.out.println("stopped"));
  }

  public Command activate(double changeBy) {
    return this.runOnce(() -> m_targetRad = changeBy);
  }
}
