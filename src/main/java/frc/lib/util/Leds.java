// All credit to FRC 6328
// http://github.com/Mechanical-Advantage


package frc.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.Optional;

public class Leds extends SubsystemBase {

  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  // Robot state tracking
  public int loopCycleCount = 0;

  public boolean intaking = false;
  public boolean hasGamePiece = false;
  public boolean shooting = false;

  public boolean distraction = false;

  public boolean endgameAlert = false;
  public boolean sameBattery = false;
  public boolean autoFinished = false;
  public double autoFinishedTime = 0.0;
  public boolean lowBatteryAlert = false;
  public boolean demoMode = false;

  private Optional<Alliance> alliance = Optional.empty();
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;
  
  private final AddressableLED underglowLeds;
  private final AddressableLEDBuffer underglowBuffer;

  // Constants
  private static final int minLoopCycleCount = 10;
  private static final double strobeFastDuration = 0.1;
  private static final double strobeSlowDuration = 0.2;
  private static final double breathDuration = 1.0;

  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;

  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveSlowCycleLength = 25.0;
  private static final double waveSlowDuration = 3.0;

  private static final double breathAllianceCycleLength = 15.0;

  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal

  // LED Lengths
  private static final int length = 43;
  private static final int staticLength = 14;
  private static final int staticSectionLength = 3;

  private static final int underglowLength = 24;

  private Leds() {
    System.out.println("[Init] Creating LEDs");
    leds = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
    loadingNotifier = new Notifier(
        () -> {
          synchronized (this) {
            breath(
                Section.STATIC_LOW,
                Color.kWhite,
                Color.kBlack,
                0.25,
                System.currentTimeMillis() / 1000.0);
            leds.setData(buffer);
          }
        });
    loadingNotifier.startPeriodic(0.02);


    underglowLeds = new AddressableLED(1);
    underglowBuffer = new AddressableLEDBuffer(underglowLength);
    underglowLeds.setLength(underglowLength);
    underglowLeds.setData(underglowBuffer);
    underglowLeds.start();
  }

  public synchronized void periodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
    }

    // Update auto state
    if (DriverStation.isDisabled()) {
      autoFinished = false;
    } else {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getFPGATimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    // Select LED mode
    solid(Section.FULL, Color.kBlack); // Default to off

    if (estopped) {
      solid(Section.FULL, Color.kRed);
    } else if (DriverStation.isDisabled()) {
      if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
        solid(1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / autoFadeTime), Color.kGreen);

      } else if (lowBatteryAlert) {
        // Low battery
        solid(Section.FULL, Color.kOrangeRed);

      } else {
        // Default pattern
        switch (alliance.get()) {
          case Red:
            breath(
                Section.FULL,
                Color.kRed,
                Color.kBlack,
                breathAllianceCycleLength,
                System.currentTimeMillis() / 1000.0);
            break;
          case Blue:
            breath(
                Section.FULL,
                Color.kBlue,
                Color.kBlack,
                breathAllianceCycleLength,
                System.currentTimeMillis() / 1000.0);
            break;
          default:
            wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveSlowCycleLength, waveSlowDuration);
            break;
        }
      }
    } else if (DriverStation.isAutonomous()) {
      wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
      if (autoFinished) {
        double fullTime = (double) length / waveFastCycleLength * waveFastDuration;
        solid((Timer.getFPGATimestamp() - autoFinishedTime) / fullTime, Color.kGreen);
      }
    } else {
      // Demo mode background
      if (demoMode) {
        wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveSlowCycleLength, waveSlowDuration);
      }

      // Set special modes
      if (endgameAlert) {
        strobe(Section.SHOULDER, Color.kBlue, strobeSlowDuration);
      } else if (shooting) {
        rainbow(Section.SHOULDER, rainbowCycleLength, rainbowDuration);
      } else if (intaking) {
        if (hasGamePiece) {
          strobe(Section.SHOULDER, Color.kGreen, strobeSlowDuration);
        } else {
          strobe(Section.SHOULDER, Color.kPurple, strobeSlowDuration);
        }
      }
    }

    // Same battery alert
    if (sameBattery) {
      breath(Section.STATIC_LOW, Color.kRed, Color.kBlack, breathDuration);
    }

    // Update LEDs
    leds.setData(buffer);
    underglowLeds.setData(underglowBuffer);
  }

  private void solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        buffer.setLED(i, color);
      }
    }
  }

  private void solid(double percent, Color color) {
    for (int i = 0; i < MathUtil.clamp(length * percent, 0, length); i++) {
      buffer.setLED(i, color);
    }
  }

  private void strobe(Section section, Color color, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(section, on ? color : Color.kBlack);
  }

  private void breath(Section section, Color c1, Color c2, double duration) {
    breath(section, c1, c2, duration, Timer.getFPGATimestamp());
  }

  private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(section, new Color(red, green, blue));
  }

  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= section.start()) {
        buffer.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      if (i >= section.start()) {
        double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  private void stripes(Section section, List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = section.start(); i < section.end(); i++) {
      int colorIndex = (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  public static enum HPGamePiece {
    NONE,
    CUBE,
    CONE
  }

  private static enum Section {
    STATIC,
    SHOULDER,
    FULL,
    STATIC_LOW,
    STATIC_MID,
    STATIC_HIGH;

    private int start() {
      switch (this) {
        case STATIC:
          return 0;
        case SHOULDER:
          return staticLength;
        case FULL:
          return 0;
        case STATIC_LOW:
          return 0;
        case STATIC_MID:
          return staticSectionLength;
        case STATIC_HIGH:
          return staticLength - staticSectionLength;
        default:
          return 0;
      }
    }

    private int end() {
      switch (this) {
        case STATIC:
          return staticLength;
        case SHOULDER:
          return length;
        case FULL:
          return length;
        case STATIC_LOW:
          return staticSectionLength;
        case STATIC_MID:
          return staticLength - staticSectionLength;
        case STATIC_HIGH:
          return staticLength;
        default:
          return length;
      }
    }
  }
}