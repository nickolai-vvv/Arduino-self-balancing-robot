/**************************************************************
 *                 Arduino-self-balancing-robot               *
 **************************************************************
 * v1.01 - c измененным списком инициализации
 * v1.02 - добавлен фильтр Калмана
 * 
 */

#include "PID.h"
#include "Motor.h"
#include "Kalman.h"
#include "MPU6050.h"
#include "Wire.h"

// Перечень макросов, необходимых для
// калибровки MPU6050
#define MPU6050_ACCEL_FS_2 0x00
#define MPU6050_GYRO_FS_250 0x00
#define BUFFER_SIZE 100
#define START_BYTE 1010
void calibration();

// Фильтр Калмана
Kalman kalmanX, kalmanY;
#define RAD_TO_DEG ((double)57.29577951)
double calculateAngles();

// Глобальные переменные
MPU6050 mpu;
Motor motor;
PID pid(1.821, 1.821*140/21,1.821*0.8/21, 10);


void setup() {
  Wire.begin();
  Serial.begin(115200);

  pid.setpoint = 4.25; // 9 - поскольку робот наклонен
  pid.setLimits(-195, 195);

  motor.initialize();
  
  Serial.println("Trying to connect to MPU6050...");
  mpu.initialize();
  Serial.println("MPU6050 connected");

  // Калибровка MPU средствами библиотеки MPU6050
  // - достаточно продолжительный процесс
  // mpu.CalibrateAccel(2); 

  // Калибровка посредством функции 
  // представленой AlexGyver'om
  // calibration();
  // Serial.print("+ calibrated!");

  // Индикация, свидетельствующая об окончании настроек.
  for(size_t i = 0; i < 5; ++i){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }//for
}//setup()

void loop() {
  
  double angle = calculateAngles();

  static uint32_t tmr;
  if(millis() - tmr > pid.getDt()){
    tmr = millis();

    pid.input = angle;
    pid.compute();

    //----------------------- Отладочная информация ------------------------//
    Serial.print("angle: "); Serial.print(angle); Serial.print(" | ");
    Serial.print("pid: "); Serial.print(pid.output + 50); Serial.print(" | ");
    //----------------------- Отладочная информация ------------------------//

    // Если робот упал, то мощность выдаваемая на моторы = 0, иначе - управляющее воздействие
    abs(angle) > 60 ? motor.set_power(0) : motor.set_power(abs(pid.output) + 50);

    // В зависимости от угла наклона крутим колеса в нужную сторону
    if (pid.output > 0) motor.run(direction::BACK);
    else if (pid.output <= 0) motor.run(direction::FORWARD);
  }//if
}//loop();

// Функция калибровки MPU6050
void calibration() {
  long offsets[6];
  long offsetsOld[6];
  int16_t mpuGet[6];
  
  // используем стандартную точность
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  
  // обнуляем оффсеты
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  delay(10);
  
  for (byte n = 0; n < 10; n++){  // 10 итераций калибровки
    for (byte j = 0; j < 6; j++)  // обнуляем калибровочный массив
      offsets[j] = 0;
    
    for (byte i = 0; i < 100 + BUFFER_SIZE; i++) {
      // делаем BUFFER_SIZE измерений для усреднения
      mpu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);
      // пропускаем первые 99 измерений
      if (i >= 99) {
        for (byte j = 0; j < 6; j++)
          offsets[j] += (long)mpuGet[j];    // записываем в калибровочный массив
      }//if
    }//for < 100 + BUFFER_SIZE
    
    for (byte i = 0; i < 6; i++) {
      offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE); // учитываем предыдущую калибровку
      if (i == 2) offsets[i] += 16384;                               // если ось Z, калибруем в 16384
      offsetsOld[i] = offsets[i];
    }//for
    
    // ставим новые оффсеты
    mpu.setXAccelOffset(offsets[0] / 8);
    mpu.setYAccelOffset(offsets[1] / 8);
    mpu.setZAccelOffset(offsets[2] / 8);
    mpu.setXGyroOffset(offsets[3] / 4);
    mpu.setYGyroOffset(offsets[4] / 4);
    mpu.setZGyroOffset(offsets[5] / 4);
    delay(2);
  }//for (byte n = 0; n < 10; n++)
}//calibration

double calculateAngles() {
  static uint32_t tmr_kalmn = micros();
  static double gyroXangle = 0;
  static double gyroYangle = 0;

  Wire.beginTransmission(0x68);     // Открываем канал связи по шине I2C с ведомым устройством (MPU6050)
  Wire.write(0x3B);                 // начинаем с регистра 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);      // Отправляем данные, которые были поставлены в очередь методом write() и завершаем передачу.
  Wire.requestFrom(0x68, 14, true); // Запрашиваем 14 байт.
  
  int16_t  accX = Wire.read() << 8 | Wire.read();    // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  int16_t  accY = Wire.read() << 8 | Wire.read();    // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  int16_t  accZ = Wire.read() << 8 | Wire.read();    // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  int16_t  tempRaw = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  int16_t  gyroX = Wire.read() << 8 | Wire.read();   // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  int16_t  gyroY = Wire.read() << 8 | Wire.read();   // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  int16_t  gyroZ = Wire.read() << 8 | Wire.read();   // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)
  
  // Рассчитываем углы на основе разных осей и переводим в градусы
  double accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  double accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  
  double gyroXrate =   (double)gyroX / 131.0;
  double gyroYrate = -((double)gyroY / 131.0);
  // Вычисляем угол наклона гироскопа без какого-либо фильтра
  gyroXangle += gyroXrate * ((double)(micros() - tmr_kalmn) / 1000000);
  gyroYangle += gyroYrate * ((double)(micros() - tmr_kalmn) / 1000000);

  // Вычисляем с использованием фильтра Калмана
  //double kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - tmr_kalmn) / 1000000);
  double kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (float)(micros() - tmr_kalmn) / 1000000);
  tmr_kalmn = micros();
  return kalAngleY - 180;
}//calculateAngles()
