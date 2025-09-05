package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double getForward();

  public double getStrafe();

  public double getTurn();

  public Trigger resetFieldCentric();

  public Trigger intake();

  public Trigger rev();

  public Trigger index();

  public Trigger eject();

  public Trigger amp();

  public Trigger align();
}
