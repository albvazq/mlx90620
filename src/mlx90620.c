#include <mlx90620.h>
#ifdef __cplusplus
extern "C" {
#endif
static byte refreshRate = Hz_LSB_32; // see thermomin.h

void initSensor(MLX90620 *s) {
  readEEPROM(s);
  writeTrimmingValue(s);
  usleep(5000);
  setConfiguration();
  usleep(5000);
  readRes(s);
}

void readEEPROM(MLX90620 *s)
{
  int j;
  i2c_start(*getBus());
  i2c_send_byte(*getBus(), 0x50 << 1 | I2C_WRITE);
  i2c_send_byte(*getBus(), 0x00);
  i2c_start(*getBus());
  i2c_send_byte(*getBus(), 0x50 << 1 | I2C_READ);
  for (j = 0; j < 256; j ++)
  {
    s->eeprom[j] = i2c_read_byte(*getBus());
    i2c_send_bit(*getBus(), I2C_ACK);
  }
  i2c_stop(*getBus());
}

void writeTrimmingValue(MLX90620 *s)
{
  i2c_start(*getBus());
  i2c_send_byte(*getBus(), 0x60 << 1 | I2C_WRITE);
  i2c_send_byte(*getBus(), 0x04);
  i2c_send_byte(*getBus(), (byte)s->eeprom[OSC_TRIM_VALUE] - 0xAA);
  i2c_send_byte(*getBus(), s->eeprom[OSC_TRIM_VALUE]);
  i2c_send_byte(*getBus(), 0x56);
  i2c_send_byte(*getBus(), 0x00);
  i2c_stop(*getBus());
}

void setConfiguration()
{
  byte defaultConfig_H = 0b01000100;            //0x54=0b00000100;
  i2c_start(*getBus());
  i2c_send_byte(*getBus(), 0x60 << 1 | I2C_WRITE);
  i2c_send_byte(*getBus(), 0x03);
  i2c_send_byte(*getBus(), (byte)refreshRate - 0x55);
  i2c_send_byte(*getBus(), refreshRate);
  i2c_send_byte(*getBus(), defaultConfig_H - 0x55);
  i2c_send_byte(*getBus(), defaultConfig_H);
  i2c_stop(*getBus());
}

int16_t readConfig()
{
  int16_t configuration = 0;
  byte configLow = 0, configHigh = 0;
  i2c_start(*getBus());
  i2c_send_byte(*getBus(), 0x60 << 1 | I2C_WRITE);
  i2c_send_byte(*getBus(), 0x02);
  i2c_send_byte(*getBus(), 0x92);
  i2c_send_byte(*getBus(), 0x00);
  i2c_send_byte(*getBus(), 0x01);
  i2c_start(*getBus());
  i2c_send_byte(*getBus(), 0x60 << 1 | I2C_READ);
  configLow=i2c_read_byte(*getBus());
  i2c_send_bit(*getBus(), I2C_ACK);
  configHigh=i2c_read_byte(*getBus());
  i2c_send_bit(*getBus(), I2C_ACK);
  i2c_stop(*getBus());
  // printf("Configuration: 0x%02X 0x%02X\n", configLow, configHigh);
  configuration = ((uint16_t) (configHigh << 8) | configLow);
  return configuration;
}

bool checkConfig()           //Poll the MLX90621 for its current status. Returns true if the POR/Brown out bit is set
{
  bool check = !((readConfig() & 0x0400) >> 10);
  return check;
}

void readRes(MLX90620 *s) {
  s->resolution = (readConfig() & 0x30) >> 4;      //Read the resolution from the config register
}

void readPTAT(MLX90620 *s)                     //Absolute ambient temperature data of the device can be read by using the following function.
{
  byte ptatLow = 0, ptatHigh = 0;
  i2c_start(*getBus());
  i2c_send_byte(*getBus(), 0x60 << 1 | I2C_WRITE);
  i2c_send_byte(*getBus(), 0x02);
  i2c_send_byte(*getBus(), 0x40);
  i2c_send_byte(*getBus(), 0x00);
  i2c_send_byte(*getBus(), 0x01);
  i2c_start(*getBus());
  i2c_send_byte(*getBus(), 0x60 << 1 | I2C_READ);
  ptatLow=i2c_read_byte(*getBus());
  i2c_send_bit(*getBus(), I2C_ACK);
  ptatHigh=i2c_read_byte(*getBus());
  i2c_send_bit(*getBus(), I2C_ACK);
  i2c_stop(*getBus());
  s->ptat = ((uint16_t) (ptatHigh << 8) | ptatLow);
}

void calculateTA(MLX90620 *s)                                          //See Datasheet MLX90621
{
  float v_th = 0, k_t1 = 0, k_t2 = 0;

  int16_t k_t1_scale = (int16_t) (s->eeprom[KT_SCALE] & 0xF0) >> 4;    //KT_SCALE=0xD2[7:4]
  int16_t k_t2_scale = (int16_t) (s->eeprom[KT_SCALE] & 0x0F) + 10;    //KT_SCALE=0xD2[3:0]+10
  v_th = (float) 256 * s->eeprom[VTH_H] + s->eeprom[VTH_L];
  if (v_th >= 32768.0)   v_th -= 65536.0;
  v_th = v_th / pow(2, (3 - s->resolution));
  k_t1 = (float) 256 * s->eeprom[KT1_H] + s->eeprom[KT1_L];
  if (k_t1 >= 32768.0)   k_t1 -= 65536.0;
  k_t1 /= (pow(2, k_t1_scale) * pow(2, (3 - s->resolution)));
  k_t2 = (float) 256 * s->eeprom[KT2_H] + s->eeprom[KT2_L];
  if (k_t2 >= 32768.0)   k_t2 -= 65536.0;
  k_t2 /= (pow(2, k_t2_scale) * pow(2, (3 - s->resolution)));      //0.000768
  s->tambient = ((-k_t1 + sqrt((k_t1*k_t1) - (4 * k_t2 * (v_th - (float) s->ptat)))) / (2 * k_t2)) + 25.0;
}

void readCPIX(MLX90620 *s)                     //Compensation pixel data of the device can be read by using the following function.
{
  byte cpixLow = 0, cpixHigh = 0;
  i2c_start(*getBus());
  i2c_send_byte(*getBus(), 0x60 << 1 | I2C_WRITE);
  i2c_send_byte(*getBus(), 0x02);
  i2c_send_byte(*getBus(), 0x41);
  i2c_send_byte(*getBus(), 0x00);
  i2c_send_byte(*getBus(), 0x01);
  i2c_start(*getBus());
  i2c_send_byte(*getBus(), 0x60 << 1 | I2C_READ);
  cpixLow=i2c_read_byte(*getBus());
  i2c_send_bit(*getBus(), I2C_ACK);
  cpixHigh=i2c_read_byte(*getBus());
  i2c_send_bit(*getBus(), I2C_ACK);
  i2c_stop(*getBus());
  s->cpix = ((int16_t) (cpixHigh << 8) | cpixLow);
  if (s->cpix >= 32768)    s->cpix -= 65536;
}

void readIR(MLX90620 *s)     //IR data of the device that it is read as a whole.
{
  int j;

  i2c_start(*getBus());
  i2c_send_byte(*getBus(), 0x60 << 1 | I2C_WRITE);
  i2c_send_byte(*getBus(), 0x02);
  i2c_send_byte(*getBus(), 0x00);
  i2c_send_byte(*getBus(), 0x01);
  i2c_send_byte(*getBus(), 0x40);
  i2c_start(*getBus());
  i2c_send_byte(*getBus(), 0x60 << 1 | I2C_READ);

  for (j = 0; j < 64; j++)
  {
    byte pixelDataLow = i2c_read_byte(*getBus());
    i2c_send_bit(*getBus(), I2C_ACK);
    byte pixelDataHigh = i2c_read_byte(*getBus());
    i2c_send_bit(*getBus(), I2C_ACK);
    s->irData[j] = (int16_t) ((pixelDataHigh << 8) | pixelDataLow);  //Each pixel value takes up two bytes thus NUM_PIXELS * 2
  }
  i2c_stop(*getBus());
}

void calculateTO(MLX90620 *s)                                              //See Datasheet MLX90621
{
  float a_ij[64], b_ij[64], alpha_ij[64];
  float emissivity, tgc, alpha_cp, a_cp, b_cp;
  int16_t a_common, a_i_scale, b_i_scale;
  int i;

  emissivity = (256 * s->eeprom[CAL_EMIS_H] + s->eeprom[CAL_EMIS_L]) / 32768.0;
  a_common = (int16_t) 256 * s->eeprom[CAL_ACOMMON_H] + s->eeprom[CAL_ACOMMON_L];
  if (a_common >= 32768)    a_common -= 65536;
  alpha_cp = (256 * s->eeprom[CAL_alphaCP_H] + s->eeprom[CAL_alphaCP_L]) / (pow(2, CAL_A0_SCALE) * pow(2, (3 - s->resolution)));
  a_i_scale = (int16_t) (s->eeprom[CAL_AI_SCALE] & 0xF0) >> 4;
  b_i_scale = (int16_t) s->eeprom[CAL_BI_SCALE] & 0x0F;
  a_cp = (float) 256 * s->eeprom[CAL_ACP_H] + s->eeprom[CAL_ACP_L];
  if (a_cp >= 32768.0)    a_cp -= 65536.0;
  a_cp /= pow(2, (3 - s->resolution));
  b_cp = (float) s->eeprom[CAL_BCP];
  if (b_cp > 127.0)     b_cp -= 256.0;
  b_cp /= (pow(2, b_i_scale) * pow(2, (3 - s->resolution)));
  tgc = (float) s->eeprom[CAL_TGC];
  if (tgc > 127.0)  tgc -= 256.0;
  tgc /= 32.0;
  float v_cp_off_comp = (float) s->cpix - (a_cp + b_cp * (s->tambient - 25.0));
  float v_ir_off_comp, v_ir_tgc_comp, v_ir_norm, v_ir_comp;
  for (i = 0; i < 64; i++)
  {
    a_ij[i] = ((float) a_common + s->eeprom[i] * pow(2, a_i_scale)) / pow(2, (3 - s->resolution));
    b_ij[i] = s->eeprom[0x40 + i];
    if (b_ij[i] > 127)        b_ij[i] -= 256;
    b_ij[i] /= (pow(2, b_i_scale) * pow(2, (3 - s->resolution)));
    v_ir_off_comp = s->irData[i] - (a_ij[i] + b_ij[i] * (s->tambient - 25.0));
    v_ir_tgc_comp = v_ir_off_comp - tgc * v_cp_off_comp;
    alpha_ij[i] = ((256 * s->eeprom[CAL_A0_H] + s->eeprom[CAL_A0_L]) / pow(2, s->eeprom[CAL_A0_SCALE]));
    alpha_ij[i] += (s->eeprom[0x80 + i] / pow(2, s->eeprom[CAL_DELTA_A_SCALE]));
    alpha_ij[i] /= pow(2, 3 - s->resolution);
    v_ir_norm = v_ir_tgc_comp / (alpha_ij[i] - tgc * alpha_cp);
    v_ir_comp = v_ir_norm / emissivity;
    s->temperatures[i] = sqrt(sqrt((v_ir_comp + pow((s->tambient + 273.15), 4)))) - 273.15;
  }
}
#ifdef __cplusplus
}
#endif