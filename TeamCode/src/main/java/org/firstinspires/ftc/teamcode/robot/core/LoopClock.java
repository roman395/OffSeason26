package org.firstinspires.ftc.teamcode.robot.core;

/**
 * Измеряет время между итерациями основного цикла робота.
 */
public final class LoopClock {
  
  private static final double NANOS_PER_SECOND = 1_000_000_000.0;
  
  /**
   * Максимальное значение dt, передаваемое подсистемам.
   * <p>
   * Защищает регуляторы от большого скачка времени,
   * например после остановки на breakpoint.
   */
  public static final double DEFAULT_MAX_DT_SECONDS = 0.1;
  
  private final double maxDtSeconds;
  
  private boolean started;
  private long previousTimeNanos;
  
  private double rawDtSeconds;
  private double dtSeconds;
  
  public LoopClock() {
    this(DEFAULT_MAX_DT_SECONDS);
  }
  
  public LoopClock(double maxDtSeconds) {
    if (maxDtSeconds <= 0
        || Double.isNaN(maxDtSeconds)
        || Double.isInfinite(maxDtSeconds)) {
      
      throw new IllegalArgumentException(
          "maxDtSeconds must be finite and greater than zero"
      );
    }
    
    this.maxDtSeconds = maxDtSeconds;
  }
  
  /**
   * Начинает измерение времени заново.
   */
  public void reset() {
    previousTimeNanos = System.nanoTime();
    rawDtSeconds = 0;
    dtSeconds = 0;
    started = true;
  }
  
  /**
   * Измеряет время после предыдущего вызова tick().
   *
   * @return ограниченное время цикла в секундах
   */
  public double tick() {
    long currentTimeNanos = System.nanoTime();
    
    if (!started) {
      previousTimeNanos = currentTimeNanos;
      started = true;
      rawDtSeconds = 0;
      dtSeconds = 0;
      return 0;
    }
    
    long elapsedNanos = currentTimeNanos - previousTimeNanos;
    previousTimeNanos = currentTimeNanos;
    
    rawDtSeconds = elapsedNanos / NANOS_PER_SECOND;
    dtSeconds = Math.min(rawDtSeconds, maxDtSeconds);
    
    return dtSeconds;
  }
  
  /**
   * Возвращает ограниченное значение, используемое регуляторами.
   */
  public double getDtSeconds() {
    return dtSeconds;
  }
  
  /**
   * Возвращает настоящее время цикла без ограничения.
   * <p>
   * Используется для диагностики и телеметрии.
   */
  public double getRawDtSeconds() {
    return rawDtSeconds;
  }
  
  /**
   * Приблизительная частота основного цикла.
   */
  public double getLoopFrequencyHz() {
    if (rawDtSeconds <= 0) {
      return 0;
    }
    
    return 1.0 / rawDtSeconds;
  }
  
}