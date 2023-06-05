// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

/**
 * A class for keeping track of how much time it takes for different parts of code to execute. This
 * is done with epochs, that are added by calls to {@link #addEpoch(String)}, and can be printed
 * with a call to {@link #printEpochs()}.
 *
 * <p>Epochs are a way to partition the time elapsed so that when overruns occur, one can determine
 * which parts of an operation consumed the most time.
 */
public class Tracer {
  private static final long kMinPrintPeriod = 1000000; // microseconds

  private long m_lastEpochsPrintTime; // microseconds
  private long m_startTime; // microseconds
  private boolean m_publishNT;
  private NetworkTableInstance m_inst;
  private String m_ntTopic;
  private boolean m_dataLogEnabled;
  private DataLog m_dataLog;
  private String m_dataLogEntry;

  private final Map<String, Long> m_epochs = new HashMap<>(); // microseconds

  /** Tracer constructor. */
  public Tracer() {
    resetTimer();
  }

  /**
   * Starts publishing added epochs to NetworkTables.
   *
   * @param topicName The NetworkTables topic to publish to
   */
  public void publishToNetworkTables(String topicName) {
    m_publishNT = true;
    m_inst = NetworkTableInstance.getDefault();
    m_ntTopic = topicName;
  }

  /**
   * Starts logging added epochs to the data log.
   *
   * @param dataLog The data log to log epochs to
   * @param entry The name of the entry to log to
   */
  public void startDataLog(DataLog dataLog, String entry) {
    m_dataLog = dataLog;
    m_dataLogEntry = entry;
    m_dataLogEnabled = true;
  }

  /** Clears all epochs. */
  public void clearEpochs() {
    m_epochs.clear();
    resetTimer();
  }

  /** Restarts the epoch timer. */
  public final void resetTimer() {
    m_startTime = RobotController.getFPGATime();
  }

  /**
   * Adds time since last epoch to the list printed by printEpochs().
   *
   * <p>Epochs are a way to partition the time elapsed so that when overruns occur, one can
   * determine which parts of an operation consumed the most time.
   *
   * <p>This should be called immediately after execution has finished, with a call to this method
   * or {@link #resetTimer()} before execution.
   *
   * @param epochName The name to associate with the epoch.
   */
  public void addEpoch(String epochName) {
    long currentTime = RobotController.getFPGATime();
    long epoch = currentTime - m_startTime;
    m_epochs.put(epochName, epoch);
    if (m_publishNT) {
      IntegerTopic topic = m_inst.getIntegerTopic(m_ntTopic + "/" + epochName);
      IntegerPublisher pub = topic.publish();
      pub.setDefault(0);
      topic.setRetained(true);
      pub.set(epoch);
      pub.close();
    }
    if (m_dataLogEnabled) {
      m_dataLog.appendInteger(m_dataLog.start(m_dataLogEntry + "/" + epochName, "int64"), epoch, 0);
    }
    m_startTime = currentTime;
  }

  /** Prints list of epochs added so far and their times to the DriverStation. */
  public void printEpochs() {
    printEpochs(out -> DriverStation.reportWarning(out, false));
  }

  /**
   * Prints list of epochs added so far and their times to the entered String consumer.
   *
   * <p>This overload can be useful for logging to a file, etc.
   *
   * @param output the stream that the output is sent to
   */
  public void printEpochs(Consumer<String> output) {
    long now = RobotController.getFPGATime();
    if (now - m_lastEpochsPrintTime > kMinPrintPeriod) {
      StringBuilder sb = new StringBuilder();
      m_lastEpochsPrintTime = now;
      m_epochs.forEach(
          (key, value) -> sb.append(String.format("\t%s: %.6fs\n", key, value / 1.0e6)));
      if (sb.length() > 0) {
        output.accept(sb.toString());
      }
    }
  }
}
