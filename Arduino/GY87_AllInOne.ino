#include <Wire.h>

/*
  ESP32-S3-DevKitC-1 + GY-87 (I2C) single-file example

  구성 분리:
  1) IMU        : MPU6050 (가속도/자이로)
  2) Magnetometer: HMC5883L (지자계)
  3) Barometer  : BMP180 (기압/온도)

  참고:
  - GY-87 보드는 보통 MPU6050(0x68), HMC5883L(0x1E), BMP180(0x77) 주소를 사용한다.
  - ESP32-S3 기본 I2C 핀은 보드 설정/배선에 따라 다를 수 있으므로 실제 연결에 맞게 변경한다.
*/

// ---------- I2C pins (ESP32-S3, 필요시 변경) ----------
static const int SDA_PIN = 8;
static const int SCL_PIN = 9;

// ---------- device addresses ----------
static const uint8_t MPU6050_ADDR = 0x68;
static const uint8_t HMC5883L_ADDR = 0x1E;
static const uint8_t BMP180_ADDR = 0x77;

// ---------- MPU6050 registers ----------
static const uint8_t MPU_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU_GYRO_XOUT_H  = 0x43;

// ---------- HMC5883L registers ----------
static const uint8_t HMC_CONFIG_A = 0x00;
static const uint8_t HMC_CONFIG_B = 0x01;
static const uint8_t HMC_MODE     = 0x02;
static const uint8_t HMC_DATA_X_MSB = 0x03;

// ---------- BMP180 registers ----------
static const uint8_t BMP180_CALIB_START = 0xAA;
static const uint8_t BMP180_CONTROL      = 0xF4;
static const uint8_t BMP180_OUT_MSB      = 0xF6;
static const uint8_t BMP180_TEMP_CMD     = 0x2E;
static const uint8_t BMP180_PRESS_CMD_OSS0 = 0x34;

// BMP180 calibration data
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;
bool bmp180CalibOk = false;

// ------------------------------------------------------------
// I2C helpers
// ------------------------------------------------------------
bool i2cWrite8(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool i2cRead(uint8_t dev, uint8_t reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t r = Wire.requestFrom((int)dev, (int)len);
  if (r != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}

int16_t be16(const uint8_t* p) {
  return (int16_t)((p[0] << 8) | p[1]);
}

uint16_t ube16(const uint8_t* p) {
  return (uint16_t)((p[0] << 8) | p[1]);
}

// ------------------------------------------------------------
// 1) IMU: MPU6050
// ------------------------------------------------------------
bool initMPU6050() {
  // Sleep 해제
  if (!i2cWrite8(MPU6050_ADDR, MPU_PWR_MGMT_1, 0x00)) return false;
  delay(20);
  return true;
}

bool readMPU6050(float& ax_g, float& ay_g, float& az_g, float& gx_dps, float& gy_dps, float& gz_dps) {
  uint8_t buf[14];
  if (!i2cRead(MPU6050_ADDR, MPU_ACCEL_XOUT_H, buf, sizeof(buf))) return false;

  int16_t ax = be16(&buf[0]);
  int16_t ay = be16(&buf[2]);
  int16_t az = be16(&buf[4]);
  int16_t gx = be16(&buf[8]);
  int16_t gy = be16(&buf[10]);
  int16_t gz = be16(&buf[12]);

  // 기본 스케일 (ACC ±2g, GYRO ±250 dps)
  ax_g = ax / 16384.0f;
  ay_g = ay / 16384.0f;
  az_g = az / 16384.0f;
  gx_dps = gx / 131.0f;
  gy_dps = gy / 131.0f;
  gz_dps = gz / 131.0f;
  return true;
}

// ------------------------------------------------------------
// 2) Magnetometer: HMC5883L
// ------------------------------------------------------------
bool initHMC5883L() {
  // 8-sample averaging, 15Hz output rate, normal measurement
  if (!i2cWrite8(HMC5883L_ADDR, HMC_CONFIG_A, 0x70)) return false;
  // gain = 1.3Ga (LSB/Gauss = 1090)
  if (!i2cWrite8(HMC5883L_ADDR, HMC_CONFIG_B, 0x20)) return false;
  // continuous measurement mode
  if (!i2cWrite8(HMC5883L_ADDR, HMC_MODE, 0x00)) return false;
  delay(10);
  return true;
}

bool readHMC5883L(float& mx_gauss, float& my_gauss, float& mz_gauss) {
  uint8_t buf[6];
  if (!i2cRead(HMC5883L_ADDR, HMC_DATA_X_MSB, buf, sizeof(buf))) return false;

  int16_t mx = be16(&buf[0]);
  int16_t mz = be16(&buf[2]);
  int16_t my = be16(&buf[4]);

  // gain 1.3Ga 기준
  mx_gauss = mx / 1090.0f;
  my_gauss = my / 1090.0f;
  mz_gauss = mz / 1090.0f;
  return true;
}

// ------------------------------------------------------------
// 3) Barometer: BMP180
// ------------------------------------------------------------
bool readBMP180Calib() {
  uint8_t c[22];
  if (!i2cRead(BMP180_ADDR, BMP180_CALIB_START, c, sizeof(c))) return false;

  ac1 = be16(&c[0]);
  ac2 = be16(&c[2]);
  ac3 = be16(&c[4]);
  ac4 = ube16(&c[6]);
  ac5 = ube16(&c[8]);
  ac6 = ube16(&c[10]);
  b1 = be16(&c[12]);
  b2 = be16(&c[14]);
  mb = be16(&c[16]);
  mc = be16(&c[18]);
  md = be16(&c[20]);
  return true;
}

bool bmp180ReadUT(int32_t& ut) {
  if (!i2cWrite8(BMP180_ADDR, BMP180_CONTROL, BMP180_TEMP_CMD)) return false;
  delay(5);
  uint8_t d[2];
  if (!i2cRead(BMP180_ADDR, BMP180_OUT_MSB, d, 2)) return false;
  ut = ube16(d);
  return true;
}

bool bmp180ReadUP(int32_t& up) {
  const int oss = 0; // oversampling 0
  if (!i2cWrite8(BMP180_ADDR, BMP180_CONTROL, BMP180_PRESS_CMD_OSS0 + (oss << 6))) return false;
  delay(5);
  uint8_t d[3];
  if (!i2cRead(BMP180_ADDR, BMP180_OUT_MSB, d, 3)) return false;
  up = (((int32_t)d[0] << 16) | ((int32_t)d[1] << 8) | d[2]) >> (8 - oss);
  return true;
}

bool readBMP180(float& temp_c, float& pressure_pa) {
  if (!bmp180CalibOk) return false;

  int32_t ut, up;
  if (!bmp180ReadUT(ut)) return false;
  if (!bmp180ReadUP(up)) return false;

  // Bosch datasheet compensation (integer)
  int32_t x1 = ((ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
  int32_t denom = x1 + md;
  if (denom == 0) return false;
  int32_t x2 = ((int32_t)mc << 11) / denom;
  int32_t b5 = x1 + x2;
  int32_t t = (b5 + 8) >> 4;
  temp_c = t / 10.0f;

  int32_t b6 = b5 - 4000;
  x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (ac2 * b6) >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = ((((int32_t)ac1 * 4 + x3) + 2)) >> 2;

  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;
  if (b4 == 0) return false;
  uint32_t b7 = ((uint32_t)up - b3) * 50000;

  int32_t p;
  if (b7 < 0x80000000) p = (b7 * 2) / b4;
  else p = (b7 / b4) * 2;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p = p + ((x1 + x2 + 3791) >> 4);

  pressure_pa = (float)p;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  bool okIMU = initMPU6050();
  bool okMAG = initHMC5883L();
  bmp180CalibOk = readBMP180Calib();

  Serial.println("=== GY-87 init ===");
  Serial.printf("MPU6050: %s\n", okIMU ? "OK" : "FAIL");
  Serial.printf("HMC5883L: %s\n", okMAG ? "OK" : "FAIL");
  Serial.printf("BMP180: %s\n", bmp180CalibOk ? "OK" : "FAIL");
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  float mx, my, mz;
  float t, p;

  bool imu = readMPU6050(ax, ay, az, gx, gy, gz);
  bool mag = readHMC5883L(mx, my, mz);
  bool bar = false;
  if (!bmp180CalibOk) {
    bmp180CalibOk = readBMP180Calib();
  }
  if (bmp180CalibOk) {
    bar = readBMP180(t, p);
  }

  if (imu) {
    Serial.printf("IMU  ACC[g] %.3f %.3f %.3f | GYRO[dps] %.3f %.3f %.3f\n", ax, ay, az, gx, gy, gz);
  } else {
    Serial.println("IMU read fail");
  }

  if (mag) {
    Serial.printf("MAG  [gauss] %.3f %.3f %.3f\n", mx, my, mz);
  } else {
    Serial.println("MAG read fail");
  }

  if (bar) {
    Serial.printf("BAR  Temp[°C] %.2f | Pressure[Pa] %.1f\n", t, p);
  } else if (!bmp180CalibOk) {
    Serial.println("BAR calibration missing (retrying)");
  } else {
    Serial.println("BAR read fail");
  }

  Serial.println("------------------------------------");
  delay(500);
}
