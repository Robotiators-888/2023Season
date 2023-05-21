package frc.robot.subsystems;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import io.github.pseudoresonance.pixy2api.pixy2ccc.Block;
public class SUB_Pixy extends SubsystemBase {
  private Pixy2 pixyCam;
  private boolean isCamera = false;
  public void robotInit() {
    pixyCam = Pixy2.createInstance(new SPILink());
  }
  public void teleopPeriodic() {
    if (!isCamera) {
      int state = pixyCam.init(1);
      isCamera = state >= 0;
      SmartDashboard.putBoolean("Camera", isCamera);
    }
    pixyCam.getCCC().getBlocks(false, 255, 255);
    ArrayList<Block> blocks  = pixyCam.getCCC().getBlocks();
    if (blocks.size() > 0) {
      double xcoord = blocks.get(0).getX();
      double ycoord = blocks.get(0).getY();
      String data = blocks.get(0).toString();
      SmartDashboard.putBoolean("present", true);
      SmartDashboard.putNumber("Xcoord", xcoord);
      SmartDashboard.putNumber("Ycoord", ycoord);
      SmartDashboard.putString("Data", data);
    } else {
      SmartDashboard.putBoolean("present", false);
    }
    SmartDashboard.putNumber("size", blocks.size());
  }
}