package org.firstinspires.ftc.teamcode.robot.config;

/**
 * Настраиваемые параметры лифта.
 *
 * <p>Позиции и PID пока намеренно не откалиброваны. Перед первым
 * позиционным запуском их нужно подобрать на собранном механизме.</p>
 */
public final class LiftConstants {
  
  private LiftConstants() {
  }
  
  public static final double KP = 0.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  
  /**
   * Безопасный начальный предел для настройки PID.
   * После проверки механики значение можно увеличить.
   */
  public static final double POSITION_MAX_POWER = 0.20;
  
  /**
   * Временный допуск. Уточняется после измерения шума энкодеров
   * и реального разброса позиции.
   */
  public static final int POSITION_TOLERANCE_TICKS = 20;
  
  /*
   * Все пресеты пока совпадают с нулём намеренно:
   * неизвестные положения не должны запускать механизм.
   */
  public static final int BOTTOM_TICKS = 0;
  public static final int MIDDLE_TICKS = 0;
  public static final int TOP_TICKS = 0;
  public static final double MANUAL_MAX_POWER = 0.20;
  public static final double MANUAL_DEADBAND = 0.05;
  public static final boolean SOFT_LIMITS_ENABLED = false;
  
  public static final int MIN_POSITION_TICKS = 0;
  public static final int MAX_POSITION_TICKS = 0;
}
