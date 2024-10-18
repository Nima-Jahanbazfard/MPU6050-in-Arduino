#include <Wire.h>
#include <MPU6050.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// ایجاد شی برای سنسور MPU6050
MPU6050 mpu;

// ایجاد شی برای LCD با آدرس I2C 0x27 و اندازه 16x2
LiquidCrystal_I2C lcd(0x27, 16, 2);

// متغیرهای فیلتر کالمن
float Q_angle = 0.001; // فرایند نویز کواریانس برای زاویه
float Q_bias = 0.003;  // فرایند نویز کواریانس برای بایاس
float R_measure = 0.01; // اندازه گیری نویز کواریانس

float angle_pitch = 0.0; // زاویه اندازه‌گیری شده
float angle_yaw = 0.0; // زاویه اندازه‌گیری شده
float angle_roll = 0.0; // زاویه اندازه‌گیری شده
float bias_pitch = 0.0;  // بایاس محاسبه‌شده
float bias_yaw = 0.0;  // بایاس محاسبه‌شده
float bias_roll = 0.0;  // بایاس محاسبه‌شده
float rate_pitch = 0.0;  // نرخ زاویه اندازه‌گیری شده
float rate_yaw = 0.0;  // نرخ زاویه اندازه‌گیری شده
float rate_roll = 0.0;  // نرخ زاویه اندازه‌گیری شده

float P_pitch[2][2] = { {1, 0}, {0, 1} };
float P_yaw[2][2] = { {1, 0}, {0, 1} };
float P_roll[2][2] = { {1, 0}, {0, 1} };

unsigned long lastTime = 0;
float dt = 0;

// متغیرهای کالیبراسیون
float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;

// مقدار آستانه برای نادیده گرفتن تغییرات کوچک
const float THRESHOLD = 0.05; // کاهش آستانه به 0.05

// تعداد نمونه‌ها برای فیلتر میانگین متحرک
const int numSamples = 20; // افزایش تعداد نمونه‌ها برای کاهش نویز
float accelX_samples[numSamples] = {0};
float accelY_samples[numSamples] = {0};
float accelZ_samples[numSamples] = {0};
float gyroX_samples[numSamples] = {0};
float gyroY_samples[numSamples] = {0};
float gyroZ_samples[numSamples] = {0};

int sampleIndex = 0;

void calibrateMPU() {
  int numCalibReadings = 2000; // افزایش تعداد خوانش‌های کالیبراسیون
  float sumGyroX = 0;
  float sumGyroY = 0;
  float sumGyroZ = 0;

  for (int i = 0; i < numCalibReadings; i++) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    sumGyroX += gx;
    sumGyroY += gy;
    sumGyroZ += gz;

    delay(2);
  }

  gyroX_offset = sumGyroX / numCalibReadings;
  gyroY_offset = sumGyroY / numCalibReadings;
  gyroZ_offset = sumGyroZ / numCalibReadings;
}

void setup() {
  // آغاز ارتباط سریال برای نمایش پیام‌ها
  Serial.begin(115200);
  delay(100);
  // شروع ارتباط I2C
  Wire.begin();

  // آغاز ارتباط با MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Failed to connect to MPU6050");
    while (1) {
      delay(10);
    }
  }

  // کالیبراسیون MPU6050
  calibrateMPU();

  // آغاز کار LCD
  lcd.begin(16, 2); // مقدار صحیح ستون‌ها و سطرها را وارد کنید
  lcd.backlight();  // روشن کردن پس زمینه LCD

  // زمان اولیه
  lastTime = millis();

  lcd.setCursor(0, 0);
  lcd.print("Pitch: ");
  lcd.setCursor(0, 1);
  lcd.print("Yaw: ");
  lcd.setCursor(0, 2);
  lcd.print("Roll: ");
}

float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias, float P[2][2]) {
  // پیش‌بینی مرحله
  float rate = newRate - bias;
  angle += dt * rate;

  // خطای کوواریانس تخمینی
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // محاسبه کلمان گین
  float S = P[0][0] + R_measure;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // محاسبه زاویه و بایاس
  float y = newAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;

  // محاسبه خطای کوواریانس تخمینی
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}

float movingAverage(float newValue, float samples[], int numSamples, int &index) {
  samples[index] = newValue;
  index = (index + 1) % numSamples;

  float sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += samples[i];
  }

  return sum / numSamples;
}

void loop() {
  // خواندن داده‌های سنسور
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // زمان فعلی
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0; // تبدیل به ثانیه
  lastTime = currentTime;

  // مقیاس‌دهی داده‌های شتاب‌سنج و ژیروسکوپ
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  float gyroX = (gx - gyroX_offset) / 131.0;
  float gyroY = (gy - gyroY_offset) / 131.0;
  float gyroZ = (gz - gyroZ_offset) / 131.0;

  // اعمال فیلتر میانگین متحرک بر روی داده‌ها
  accelX = movingAverage(accelX, accelX_samples, numSamples, sampleIndex);
  accelY = movingAverage(accelY, accelY_samples, numSamples, sampleIndex);
  accelZ = movingAverage(accelZ, accelZ_samples, numSamples, sampleIndex);
  gyroX = movingAverage(gyroX, gyroX_samples, numSamples, sampleIndex);
  gyroY = movingAverage(gyroY, gyroY_samples, numSamples, sampleIndex);
  gyroZ = movingAverage(gyroZ, gyroZ_samples, numSamples, sampleIndex);

  // محاسبه زاویه‌ها با استفاده از فیلتر کالمن
  float accelPitch = atan2(accelY, accelZ) * 180 / PI;
  float accelYaw = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;
   float accelRoll = atan2(accelX, accelZ) * 180 / PI;

  float pitch = kalmanFilter(accelPitch, gyroX, dt, angle_pitch, bias_pitch, P_pitch);
  float yaw = kalmanFilter(accelYaw, gyroZ, dt, angle_yaw, bias_yaw, P_yaw);
  float roll = kalmanFilter(accelRoll, gyroY, dt, angle_roll, bias_roll, P_roll);

  // اعمال آستانه برای نادیده گرفتن تغییرات کوچک
  if (fabs(pitch) < THRESHOLD) {
    pitch = 0;
  }
  if (fabs(yaw) < THRESHOLD) {
    yaw = 0;
  }
  if (fabs(roll) < THRESHOLD) {
    roll = 0;
  }

  // نمایش زاویه‌ها روی LCD
  lcd.setCursor(7, 0);
  lcd.print(pitch, 2); // نمایش با دو رقم اعشار
  lcd.print("   ");
  lcd.setCursor(7, 1);
  lcd.print(yaw, 2); // نمایش با دو رقم اعشار
  lcd.print("   ");
  lcd.setCursor(7, 2);
  lcd.print(roll, 2); // نمایش با دو رقم اعشار
  lcd.print("   ");

  // تاخیر برای جلوگیری از نویز
  delay(10);
}


