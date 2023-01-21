
package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class Saitek extends GenericHID {
  
  
  /** Represents a digital button on an Saitek. */
  public enum Button {
    pinkTopLeft(1),
    pinkTopMiddle(2),
    pinkTopRight(3),
    pinkBottomLeft(6),
    pinkBottomMiddle(7),
    pinkBottomRight(8),
    orangeTopLeft(4),
    orangeTopRight(5),
    orangeBottomLeft(9),
    orangeBottomRight(10),
    yellowTopLeft(11),
    yellowTopMiddle(13),
    yellowTopRight(15),
    yellowBottomLeft(12),
    yellowBottomMiddle(14),
    yellowBottomRight(16);
  
    private final int value;
    Button(int value) {
      this.value = value;
    }


  }
  public enum Axis {
    xAxis1(0),
    yAxis1(1),
    rotate1(2),
    xAxis2(3),
    yAxis2(4),
    rotate2(5);

    public final int value;

    Axis(int value) {
      this.value = value;
    }


  }

  public Saitek(final int port) {
    super(port);
  }



  public double getxAxis1() {
    return getRawAxis(Axis.xAxis1.value);
  }

  public double getyAxis1() {
    return getRawAxis(Axis.yAxis1.value);
  }
  
  public double rotate1() {
    return getRawAxis(Axis.xAxis1.value);
  }
  
  public double getxAxis2() {
    return getRawAxis(Axis.xAxis2.value);
  }

  public double getyAxis2() {
    return getRawAxis(Axis.yAxis2.value);
  }
  
  public double rotate2() {
    return getRawAxis(Axis.xAxis2.value);
  }

//Pink Buttons
  //Top Left
  public boolean getPinkTopLeftButton() {
    return getRawButton(Button.pinkTopLeft.value);
  }


  public boolean getPinkTopLeftButtonPressed() {
    return getRawButtonPressed(Button.pinkTopLeft.value);
  }


  public boolean getPinkTopLeftButtonReleased() {
    return getRawButtonReleased(Button.pinkTopLeft.value);
  }


  @SuppressWarnings("MethodName")
  public BooleanEvent pinkTopLeft(EventLoop loop) {
    return new BooleanEvent(loop, this::getPinkTopLeftButton);
  }
  //Top middle
  public boolean getPinkTopMiddleButton() {
    return getRawButton(Button.pinkTopMiddle.value);
  }


  public boolean getPinkTopMiddleButtonPressed() {
    return getRawButtonPressed(Button.pinkTopMiddle.value);
  }


  public boolean getPinkTopMiddleButtonReleased() {
    return getRawButtonReleased(Button.pinkTopMiddle.value);
  }


  @SuppressWarnings("MethodName")
  public BooleanEvent pinkTopMiddle(EventLoop loop) {
    return new BooleanEvent(loop, this::getPinkTopMiddleButton);
  }
  //Top Right
  public boolean getPinkTopRightButton() {
    return getRawButton(Button.pinkTopRight.value);
  }


  public boolean getPinkTopRightButtonPressed() {
    return getRawButtonPressed(Button.pinkTopRight.value);
  }


  public boolean getPinkTopRightButtonReleased() {
    return getRawButtonReleased(Button.pinkTopRight.value);
  }


  @SuppressWarnings("MethodName")
  public BooleanEvent pinkTopRight(EventLoop loop) {
    return new BooleanEvent(loop, this::getPinkTopRightButton);
  }
  //Bottom Left
  public boolean getPinkBottomLeftButton() {
    return getRawButton(Button.pinkBottomLeft.value);
  }


  public boolean getPinkBottomLeftButtonPressed() {
    return getRawButtonPressed(Button.pinkBottomLeft.value);
  }


  public boolean getPinkBottomLeftButtonReleased() {
    return getRawButtonReleased(Button.pinkBottomLeft.value);
  }


  @SuppressWarnings("MethodName")
  public BooleanEvent pinkBottomLeft(EventLoop loop) {
    return new BooleanEvent(loop, this::getPinkBottomLeftButton);
  }
  //Bottom Middle
  public boolean getPinkBottomMiddleButton() {
    return getRawButton(Button.pinkBottomMiddle.value);
  }


  public boolean getPinkBottomMiddleButtonPressed() {
    return getRawButtonPressed(Button.pinkBottomMiddle.value);
  }


  public boolean getPinkBottomMiddleButtonReleased() {
    return getRawButtonReleased(Button.pinkBottomMiddle.value);
  }


  @SuppressWarnings("MethodName")
  public BooleanEvent pinkBottomMiddle(EventLoop loop) {
    return new BooleanEvent(loop, this::getPinkBottomMiddleButton);
  }

  //Bottom Right
  public boolean getPinkBottomRightButton() {
    return getRawButton(Button.pinkBottomRight.value);
  }


  public boolean getPinkBottomRightButtonPressed() {
    return getRawButtonPressed(Button.pinkBottomRight.value);
  }


  public boolean getPinkBottomRightButtonReleased() {
    return getRawButtonReleased(Button.pinkBottomRight.value);
  }


  @SuppressWarnings("MethodName")
  public BooleanEvent pinkBottomRight(EventLoop loop) {
    return new BooleanEvent(loop, this::getPinkBottomRightButton);
  }
  
// Orange Buttons  
  // Top Left
  public boolean getOrangeTopLeftButton() {
    return getRawButton(Button.orangeTopLeft.value);
  }

  public boolean getOrangeTopLeftButtonPressed() {
    return getRawButtonPressed(Button.orangeTopLeft.value);
  }

  public boolean getOrangeTopLeftButtonReleased() {
    return getRawButtonReleased(Button.orangeTopLeft.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent orangeTopLeft(EventLoop loop) {
    return new BooleanEvent(loop, this::getOrangeTopLeftButton);
  }
  
  // Top Right
  public boolean getOrangeTopRightButton() {
    return getRawButton(Button.orangeTopRight.value);
  }

  public boolean getOrangeTopRightButtonPressed() {
    return getRawButtonPressed(Button.orangeTopRight.value);
  }

  public boolean getOrangeTopRightButtonReleased() {
    return getRawButtonReleased(Button.orangeTopRight.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent orangeTopRight(EventLoop loop) {
    return new BooleanEvent(loop, this::getOrangeTopRightButton);
  }
 
  // Bottom Left
  public boolean getOrangeBottomLeftButton() {
    return getRawButton(Button.orangeBottomLeft.value);
  }

  public boolean getOrangeBottomLeftButtonPressed() {
    return getRawButtonPressed(Button.orangeBottomLeft.value);
  }

  public boolean getOrangeBottomLeftButtonReleased() {
    return getRawButtonReleased(Button.orangeBottomLeft.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent orangeBottomLeft(EventLoop loop) {
    return new BooleanEvent(loop, this::getOrangeBottomLeftButton);
  }

  // Bottom Right
  public boolean getOrangeBottomRightButton() {
    return getRawButton(Button.orangeBottomRight.value);
  }

  public boolean getOrangeBottomRightButtonPressed() {
    return getRawButtonPressed(Button.orangeBottomRight.value);
  }

  public boolean getOrangeBottomRightButtonReleased() {
    return getRawButtonReleased(Button.orangeBottomRight.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent orangeBottomRight(EventLoop loop) {
    return new BooleanEvent(loop, this::getOrangeBottomRightButton);
  }

// Yellow Buttons
  // Top Left
  public boolean getYellowTopLeftButton() {
    return getRawButton(Button.yellowTopLeft.value);
  }

  public boolean getYellowTopLeftButtonPressed() {
    return getRawButtonPressed(Button.yellowTopLeft.value);
  }

  public boolean getYellowTopLeftButtonReleased() {
    return getRawButtonReleased(Button.yellowTopLeft.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent yellowTopLeft(EventLoop loop) {
    return new BooleanEvent(loop, this::getYellowTopLeftButton);
  }

  // Top Middle
  public boolean getYellowTopMiddleButton() {
    return getRawButton(Button.yellowTopMiddle.value);
  }

  public boolean getYellowTopMiddleButtonPressed() {
    return getRawButtonPressed(Button.yellowTopMiddle.value);
  }

  public boolean getYellowTopMiddleButtonReleased() {
    return getRawButtonReleased(Button.yellowTopMiddle.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent yellowTopMiddle(EventLoop loop) {
    return new BooleanEvent(loop, this::getYellowTopMiddleButton);
  }

  // Top Right
  public boolean getYellowTopRightButton() {
    return getRawButton(Button.yellowTopRight.value);
  }

  public boolean getYellowTopRightButtonPressed() {
    return getRawButtonPressed(Button.yellowTopRight.value);
  }

  public boolean getYellowTopRightButtonReleased() {
    return getRawButtonReleased(Button.yellowTopRight.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent yellowTopRight(EventLoop loop) {
    return new BooleanEvent(loop, this::getYellowTopRightButton);
  }
  
  // Bottom Left
  public boolean getYellowBottomLeftButton() {
    return getRawButton(Button.yellowBottomLeft.value);
  }

  public boolean getYellowBottomLeftButtonPressed() {
    return getRawButtonPressed(Button.yellowBottomLeft.value);
  }

  public boolean getYellowBottomLeftButtonReleased() {
    return getRawButtonReleased(Button.yellowBottomLeft.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent yellowBottomLeft(EventLoop loop) {
    return new BooleanEvent(loop, this::getYellowBottomLeftButton);
  }

  // Bottom Middle
  public boolean getYellowBottomMiddleButton() {
    return getRawButton(Button.yellowBottomLeft.value);
  }

  public boolean getYellowBottomMiddleButtonPressed() {
    return getRawButtonPressed(Button.yellowBottomMiddle.value);
  }

  public boolean getYellowBottomMiddleButtonReleased() {
    return getRawButtonReleased(Button.yellowBottomMiddle.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent yellowBottomMiddle(EventLoop loop) {
    return new BooleanEvent(loop, this::getYellowBottomMiddleButton);
  }

  // Bottom Right
  public boolean getYellowBottomRightButton() {
    return getRawButton(Button.yellowBottomRight.value);
  }

  public boolean getYellowBottomRightButtonPressed() {
    return getRawButtonPressed(Button.yellowBottomRight.value);
  }

  public boolean getYellowBottomRightButtonReleased() {
    return getRawButtonReleased(Button.yellowBottomRight.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent yellowBottomRight(EventLoop loop) {
    return new BooleanEvent(loop, this::getYellowBottomRightButton);
  }
} 