// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the 
package frc.lib;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import java.util.Objects;

/**
 * A trapezoid-shaped velocity profile.
 *
 * <p>While this class can be used for a profiled movement from start to finish, the intended usage
 * is to filter a reference's dynamics based on trapezoidal velocity constraints. To compute the
 * reference obeying this constraint, do the following.
 *
 * <p>Initialization:
 *
 * <pre><code>
 * TrapezoidProfile.Constraints constraints =
 *   new TrapezoidProfile.Constraints(kMaxV, kMaxA);
 * TrapezoidProfile.State previousProfiledReference =
 *   new TrapezoidProfile.State(initialReference, 0.0);
 * </code></pre>
 *
 * <p>Run on update:
 *
 * <pre><code>
 * TrapezoidProfile profile =
 *   new TrapezoidProfile(constraints, unprofiledReference, previousProfiledReference);
 * previousProfiledReference = profile.calculate(timeSincePreviousUpdate);
 * </code></pre>
 *
 * <p>where `unprofiledReference` is free to change between calls. Note that when the unprofiled
 * reference is within the constraints, `calculate()` returns the unprofiled reference unchanged.
 *
 * <p>Otherwise, a timer can be started to provide monotonic values for `calculate()` and to
 * determine when the profile has completed via `isFinished()`.
 */
public class SProfile {
  // The direction of the profile, either 1 for forwards or -1 for inverted
  private int m_direction;

  private Constraints m_constraints;
  private State m_initial;
  private State m_goal;

  private double tJerk;
  private double tAccel;
  private double tVelocity;

  private double totalTime;
  private double t1;
  private double t2;
  private double t3;
  private double t4;
  private double t5;
  private double t6;
  private double t7;

  public static class Constraints {
    public final double maxVelocity;
    public final double maxJerk;
    public final double maxAcceleration;

    /**
     * Construct constraints for a TrapezoidProfile.
     *
     * @param maxVelocity maximum velocity
     * @param maxAcceleration maximum acceleration
     */
    public Constraints(double maxVelocity, double maxAcceleration, double jerkPercent) {
      this.maxVelocity = maxVelocity;
      this.maxAcceleration = maxAcceleration;
      this.maxJerk = maxAcceleration / ((maxVelocity/maxAcceleration) * jerkPercent);
      MathSharedStore.reportUsage(MathUsageId.kTrajectory_TrapezoidProfile, 1);
    }
  }

  public static class State {
    public double position;
    public double acceleration;
    public double velocity;
    public double jerk;

    public State() {}

    public State(double position, double velocity, double acceleration, double jerk) {
      this.position = position;
      this.velocity = velocity;
      this.acceleration = acceleration;
      this.jerk = jerk;
    }

    @Override
    public boolean equals(Object other) {
      if (other instanceof State) {
        State rhs = (State) other;
        return this.position == rhs.position && this.velocity == rhs.velocity;
      } else {
        return false;
      }
    }

    @Override
    public int hashCode() {
      return Objects.hash(position, velocity);
    }
  }

  /**
   * Construct a TrapezoidProfile.
   *
   * @param constraints The constraints on the profile, like maximum velocity.
   * @param goal The desired state when the profile is complete.
   * @param initial The initial state (usually the current state).
   */
  public SProfile(Constraints constraints, State goal, State initial) {
    m_direction = shouldFlipAcceleration(initial, goal) ? -1 : 1;
    m_constraints = constraints;
    m_initial = direct(initial);
    m_goal = direct(goal);

    if (m_initial.velocity > m_constraints.maxVelocity) {
      m_initial.velocity = m_constraints.maxVelocity;
    }

    tJerk = m_constraints.maxAcceleration/m_constraints.maxJerk;
    tAccel = m_constraints.maxVelocity/m_constraints.maxAcceleration - tJerk;
    tVelocity = (m_goal.position - (2 * t3(t3).position)) / m_constraints.maxVelocity;

    t1 = tJerk;
    t2 = tAccel;
    t3 = tJerk + tAccel;
    t4 = tVelocity;
    t5 = tVelocity + tJerk;
    t6 = tVelocity + tAccel;
    t7 = tVelocity + tJerk + tAccel;

    this.totalTime = 0;
  }

  public void setTotalTime(double time) {
    this.totalTime = time;
  }

  /**
   * Construct a SProfile.
   *
   * @param constraints The constraints on the profile, like maximum velocity.
   * @param goal The desired state when the profile is complete.
   */
  public SProfile(Constraints constraints, State goal) {
    this(constraints, goal, new State(0, 0, 0, 0));
  }

  /**
   * Calculate the correct position and velocity for the profile at a time t where the beginning of
   * the profile was at time t = 0.
   *
   * @param t The time since the beginning of the profile.
   * @return The position and velocity of the profile at time t.
   */
  public State calculate(double t) {
    State result = new State(m_initial.position, m_initial.velocity, m_initial.acceleration, m_initial.jerk);

    if (t < t1) {
      result.position = t1(t).position + m_initial.position;
      result.velocity = t1(t).velocity;
      result.acceleration = t1(t).acceleration;
      result.jerk = t1(t).jerk;
    } else if (t < t2) {
      result.position = t2(t).position + m_initial.position;
      result.velocity = t2(t).velocity;
      result.acceleration = t2(t).acceleration;
      result.jerk = t2(t).jerk;
    } else if (t < t3) {
      result.position = t3(t).position + m_initial.position;
      result.velocity = t3(t).velocity;
      result.acceleration = t3(t).acceleration;
      result.jerk = t3(t).jerk;     
    } else if (t < t4) {
      result.position = t4(t).position + m_initial.position;
      result.velocity = t4(t).velocity;
      result.acceleration = t4(t).acceleration;
      result.jerk = t4(t).jerk;
    } else if (t < t5) {
      result.position = t5(t).position + m_initial.position;
      result.velocity = t5(t).velocity;
      result.acceleration = t5(t).acceleration;
      result.jerk = t5(t).jerk;
    } else if (t < t6) {
      result.position = t6(t).position + m_initial.position;
      result.velocity = t6(t).velocity;
      result.acceleration = t6(t).acceleration;
      result.jerk = t6(t).jerk;
    } else if (t < t7) {
      result.position = t7(t).position + m_initial.position;
      result.velocity = t7(t).velocity;
      result.acceleration = t7(t).acceleration;
      result.jerk = t7(t).jerk;
    } else if (t > t7) {
      result.position = m_goal.position;
      result.velocity = 0;
      result.acceleration = 0;
      result.jerk = 0;
    }

    return direct(result);
  }

  public State calculateRecursive(double step) {
    totalTime += step;


    return direct(calculate(totalTime));
  }


 public State t1(double t) {
    return new State(
    m_constraints.maxJerk, 
    m_constraints.maxJerk * t, 
    0.5 * m_constraints.maxJerk * Math.pow(t, 2),
    (1/6) * m_constraints.maxJerk * Math.pow(t, 3));
  }

  public State t2(double t) {
    return new State(
      0, 
      m_constraints.maxAcceleration, 
      t1(t1).velocity + m_constraints.maxAcceleration * (t - t1),
      t1(t1).position + t1(t1).velocity * (t - t1) + 0.5 * t1(t1).acceleration * Math.pow(t - t1, 2)
    );
  }

  public State t3(double t) {
    return new State(
      -m_constraints.maxJerk, 
      -m_constraints.maxJerk * (t - t3), 
      t2(t2).velocity - 0.5 * m_constraints.maxJerk * Math.pow(t - t3, 2), 
      t2(t2).position + t3VAD(t) - t3VAD(t3)
    );
  }

  // Temp functions for code simplicity

  private double t3VAD(double t) {
    return m_constraints.maxVelocity * (t - t2) - (1/6) * m_constraints.maxJerk * Math.pow(t - t3, 3);
  }

  public State t4(double t) {
    return new State(
      0,
      0,
      m_constraints.maxVelocity,
      t3(t3).position + m_constraints.maxVelocity * (t - t3)
    );
  }

  public State t5(double t) {
    return new State(
      -m_constraints.maxJerk,
      -m_constraints.maxJerk * (t - t4), 
      t4(t4).velocity - 0.5 * m_constraints.maxJerk * Math.pow(t - t4, 2),
      t4(t4).position + t3(t + t2 - t4).position - t2(t2).position
    );
  }

  public State t6(double t) {
    return new State(
      0,
      -m_constraints.maxAcceleration,
      t1(t1).velocity - m_constraints.maxAcceleration * (t - t6),
      t2(-(t - t4 - t3 )).position + t2(t2).position + t5(t).position
    );
  }

  public State t7(double t) {
    return new State(
      m_constraints.maxJerk,
      m_constraints.maxJerk * (t - t7),
      0.5 * m_constraints.maxJerk * Math.pow(t - t7, 2),
      -t1(-(t - t7)).position + t1(t1).position
    );
  }

  /**
   * Returns the time left until a target distance in the profile is reached.
   *
   * @param target The target distance.
   * @return The time left until a target distance in the profile is reached.
   */
  public double timeLeftUntil(double target) {
    return 0;
  }

  /**
   * Returns the total time the profile takes to reach the goal.
   *
   * @return The total time the profile takes to reach the goal.
   */
  public double totalTime() {
    return t7;
  }

  /**
   * Returns true if the profile has reached the goal.
   *
   * <p>The profile has reached the goal if the time since the profile started has exceeded the
   * profile's total time.
   *
   * @param t The time since the beginning of the profile.
   * @return True if the profile has reached the goal.
   */
  public boolean isFinished(double t) {
    return t >= totalTime();
  }

  /**
   * Returns true if the profile inverted.
   *
   * <p>The profile is inverted if goal position is less than the initial position.
   *
   * @param initial The initial state (usually the current state).
   * @param goal The desired state when the profile is complete.
   */
  private static boolean shouldFlipAcceleration(State initial, State goal) {
    return initial.position > goal.position;
  }

  // Flip the sign of the velocity and position if the profile is inverted
  private State direct(State in) {
    State result = new State(in.position, in.velocity, in.acceleration, in.jerk);
    result.position = result.position * m_direction;
    result.velocity = result.velocity * m_direction;
    result.acceleration = result.acceleration * m_direction;
    return result;
  }
}

