#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#define ADDR 80
#include "SparkFun_SCD30_Arduino_Library.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <DFRobot_MLX90614.h>
#include "innerWdt.h"
// byte deviceAddress = 0x35;


SCD30 airSensor;

MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255

//definir sensor MLX
DFRobot_MLX90614_I2C sensor;   // instantiate an object to drive our sensor

#include <DFRobot_MAX30102.h>

DFRobot_MAX30102 particleSensor1;
/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x00, 0x8E };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x66, 0x01 };

/* ABP para*/
uint8_t nwkSKey[] = { 0xB1, 0x34, 0x29, 0x6E, 0xA3, 0x1B, 0x55, 0x2D, 0x04, 0xFE, 0x67, 0x7D, 0xF3, 0xA4, 0x17, 0x91 };
uint8_t appSKey[] = { 0x1B, 0x75, 0x6A, 0xCE, 0xFD, 0xBB, 0xB5, 0xFD, 0x0F, 0x5E, 0xBF, 0x8E, 0x62, 0xF0, 0x3A, 0x20 };
uint32_t devAddr =  ( uint32_t )0x260CA363;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 14000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

//VARIABLES SCD30
float co2Concentracion;
float temperaturascd;
float humedadscd;

// VARIABLES MAX3012
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
float tempmax;

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
int32_t spo2_; //SPO2 value
int8_t validSPO2_; //indicator to show if the SPO2 calculation is valid

//VARIABLES MLX90614
float ambientTemp;
float objectTemp;


//VARIABLES NiCLA
byte variables[64];
float d, c, t, h, p, g;
int16_t accx, accy, accz;
int16_t gyrox, gyroy, gyroz;
// int16_t magx, magy, magz;
float act;
uint16_t iaq_reg;
uint16_t iaq_stat;
float bvoc;
uint32_t co2eq;
float orix, oriy, oriz;

// VARIABLES KY-038
uint8_t vector[1000];
int suma;

// CONTADOR
int j =0;

//BATERIA
uint16_t batteryVoltage;
uint16_t batteryVoltage1;

void scd30(){

  
  if (airSensor.dataAvailable())
  {
    co2Concentracion = airSensor.getCO2();
    temperaturascd = airSensor.getTemperature();
    humedadscd = airSensor.getHumidity();          

    Serial.println();
    Serial.println("------------VALORES SCD30..:--------------");
    
    Serial.print("co2(ppm):");
    Serial.print(co2Concentracion);

    Serial.print(" temp(C):");
    Serial.print(temperaturascd, 2);

    Serial.print(" humidity(%):");
    Serial.println(humedadscd, 2);
            
  }
  else
    Serial.println("Waiting for new data");

}
//funcion SPO2 CORREGIDO
int sumarVector(uint32_t vectorx[], int tamano) 
{
  int suma = 0;
  for (int i = 0; i < tamano; i++) {
    suma += vectorx[i];
  }
  return suma;
}
void maxspo2()
{
  int tamanoVector = sizeof(irBuffer) / sizeof(irBuffer[0]);

  uint32_t sumatir = sumarVector(irBuffer, tamanoVector);
  float promir = sumatir/tamanoVector;
  uint32_t sumatred = sumarVector(redBuffer, tamanoVector);
  float promred = sumatred/tamanoVector;

  // if (particleSensor.getRed()< 10000 && particleSensor.getIR() < 10000) 
  // { 
  //   spo2_ = -999;
  //   validSPO2_ = 0;
    
  //   Serial.print(F("SPO2="));
  //   Serial.print(spo2_);
  //   Serial.print(F(", SPO2Valid="));
  //   Serial.println(validSPO2_);

  // }else {

  float prom = promred/promir;
  
  // spo2 = 100 - (10*prom);
  // spo2_ = 104 - (17*prom); AJUSTE AOARTIR DE LA CURVA DE CALIBRACIÓN
  spo2_ = 104 - (13*prom);
  validSPO2_ = 1;

  Serial.print(F(", SPO2="));
  Serial.print(spo2_);
  Serial.print(F(", SPO2Valid="));
  Serial.println(validSPO2_);
  Serial.print(", constante: ");
  Serial.println(17*prom);

  // }
}
void toma100datos(){
  
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    
    Serial.print(F("Dato="));
    Serial.print(i);
    Serial.print(F(", red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }  
}
void toma25datos(){
   //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }
  //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
             particleSensor.check(); //Check the sensor for new data

      // Serial.println("Check true");
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }

    Serial.println();
    Serial.println("------------VALORES MAX30102------------:"); 
 
 if (particleSensor.getRed()< 10000 && particleSensor.getIR() < 10000) 
  { 
    spo2_ = -999;
    validSPO2_ = 0;
    
    Serial.print(F("SPO2="));
    Serial.print(spo2_);
    Serial.print(F(", SPO2Valid="));
    Serial.println(validSPO2_);

    heartRate = -999;
    validHeartRate = 0;
    
    Serial.print(F("HR="));
    Serial.print(heartRate, DEC);
    Serial.print(F(", HRvalid="));
    Serial.println(validHeartRate, DEC);

  } else{
     maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);   
    // particleSensor1.heartrateAndOxygenSaturation(/**SPO2=*/&spo2, /**SPO2Valid=*/&validSPO2, /**heartRate=*/&heartRate, /**heartRateValid=*/&validHeartRate);
    // tempmax=particleSensor.readTemperature();
    maxspo2();
   
      Serial.print(F("HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.println(validHeartRate, DEC);

      Serial.print(F("Temperaturamax="));
      Serial.println(tempmax);
  }
  tempmax=particleSensor.readTemperature();
}

void datosmlx(){
  ambientTemp = sensor.getAmbientTempCelsius();
  objectTemp = sensor.getObjectTempCelsius();
  //GRAFICAR
  Serial.println();
  Serial.println("VALORES MLX:");
  // print measured data in Celsius, unit is Celsius (°C)
  Serial.print("Ambient celsius : "); Serial.print(ambientTemp); Serial.println(" °C");
  Serial.print("Object celsius : ");  Serial.print(objectTemp);  Serial.println(" °C");
  delay(500);
}
void medicionesnicla(){
  // delay(1000);
  Wire1.requestFrom(ADDR, 64);
  while (Wire1.available()) {
  Wire1.readBytes(variables, 64); 
  }
  memcpy(&d, &variables[0], 4);
  memcpy(&c, &variables[4], 4);
  memcpy(&t, &variables[8], 4);
  memcpy(&h, &variables[12], 4);
  memcpy(&p, &variables[16], 4);
  memcpy(&g, &variables[20], 4);

  memcpy(&gyrox, &variables[24], 2);
  memcpy(&gyroy, &variables[26], 2);
  memcpy(&gyroz, &variables[28], 2);

  memcpy(&accx, &variables[30], 2);
  memcpy(&accy, &variables[32], 2);
  memcpy(&accz, &variables[34], 2);
 
  memcpy(&act, &variables[36], 4);

  memcpy(&iaq_reg, &variables[40], 2);
  memcpy(&iaq_stat, &variables[42], 2);
  memcpy(&bvoc,  &variables[44], 4);
  memcpy(&co2eq, &variables[48], 4);

  memcpy(&orix, &variables[52], 4);
  memcpy(&oriy, &variables[56], 4);
  memcpy(&oriz, &variables[60], 4);

  
  Serial.println();
  Serial.println("------------VALORES NICLA-----------------------");
  Serial.print("step_detector:");
  Serial.println(d);
  Serial.print("step_counter: ");
  Serial.println(c);
  Serial.print("temperatura:");
  Serial.println(t);
  Serial.print("humedad: ");
  Serial.println(h);
  Serial.print("Presion: ");
  Serial.println(p);
  Serial.print("gases: ");
  Serial.println(g);
  Serial.print("Gyroscopio: ");
  Serial.print(gyrox);
  Serial.print(",");
  Serial.print(gyroy);
  Serial.print(",");
  Serial.println(gyroz);
  Serial.print("Acelerometro: ");
  Serial.print(accx);
  Serial.print(",");
  Serial.print(accy);
  Serial.print(",");
  Serial.println(accz);
  Serial.print("Actividad: ");
  Serial.println(act);

  Serial.print("IAQ: ");
  Serial.println(iaq_reg); // IAQ value for regular use case (0 to 500)
  Serial.print("Stationary_IAQ: ");
  Serial.println(iaq_stat); // IAQ value for stationary use cases (0 to 500)
  Serial.print("BVOC: ");
  Serial.println(bvoc); // BVOC equivalent (ppm)
  Serial.print("CO2:  ");
  Serial.println(co2eq); // CO2 equivalent (ppm) [400,]

  Serial.print("Orientacion: ");
  Serial.print(orix);
  Serial.print(",");
  Serial.print(oriy);
  Serial.print(",");
  Serial.println(oriz);
  // Serial.println("Orientacion: " + String(orix) + "," + oriy + "," + oriz); 
 
}
//SENSOR DE SONIDO KY-037
void vaciarVector(uint8_t vector[], int longitud) {
  for (int i = 0; i < longitud; i++) {
    vector[i] = 0;
  }
}
int sumarVectorky(uint8_t vectorx[], int tamano) {
  int suma = 0;
  for (int i = 4; i < tamano; i++) {
    suma += vectorx[i];
  }
  return suma;
}
void sensorky037(){
  vaciarVector(vector, sizeof(vector) / sizeof(vector[0]));
  for (int i = 0; i < 1000; i++) {
    vector[i] = analogRead(ADC2);
    // Serial.print("Dato: ");
     if (vector[i]<160){
      vector[i]=0;
    }
  }
  int tamanoVector = sizeof(vector) / sizeof(vector[0]);
  suma = sumarVectorky(vector, tamanoVector);
}


/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/

    scd30();
    long  co2 = co2Concentracion*1000;
    long  temp = temperaturascd*1000;
    long  hum = humedadscd*1000;

    appDataSize = 100;
    appData[0]= co2 >> 24;
    appData[1]= co2 >> 16;
    appData[2]= co2 >> 8;
    appData[3]= co2;

    appData[4]= temp >> 24;
    appData[5]= temp >> 16;
    appData[6]= temp >> 8;
    appData[7]= temp;

    appData[8]= hum >> 24;
    appData[9]= hum >> 16;
    appData[10]= hum >> 8;
    appData[11]= hum;
    
    toma25datos(); 
      
    long heart_ttn = heartRate*1000;
    long valid_heart_ttn = validHeartRate*10;
    long SPO2_ttn = spo2_*1000;
    long SPO2Valid_ttn = validSPO2_*10;
    long tempmax_ttn=tempmax*1000;

    appData[12]= heart_ttn >> 24; //Frecuencia Cardiaca
    appData[13]= heart_ttn >> 16;
    appData[14]= heart_ttn >> 8;
    appData[15]= heart_ttn;

    appData[16]= valid_heart_ttn >> 8;
    appData[17]= valid_heart_ttn;

    appData[18]= SPO2_ttn >> 24; //Oxigenacion en la sangre
    appData[19]= SPO2_ttn >> 16;
    appData[20]= SPO2_ttn >> 8;
    appData[21]= SPO2_ttn;

    appData[22]= SPO2Valid_ttn >> 8;
    appData[23]= SPO2Valid_ttn;

    appData[24]= tempmax_ttn >> 24; //Temperatura max30102
    appData[25]= tempmax_ttn >> 16;
    appData[26]= tempmax_ttn >> 8;
    appData[27]= tempmax_ttn;
      // // Wire.endTransmission(0x57);

      // Wire1.beginTransmission(80);
    datosmlx();
      
    long ambientTemp_ttn = ambientTemp*1000;
    long objectTemp_ttn = objectTemp*1000;

    appData[28]= ambientTemp_ttn >> 24; //Temperatura
    appData[29]= ambientTemp_ttn >> 16;
    appData[30]= ambientTemp_ttn >> 8;
    appData[31]= ambientTemp_ttn;

    appData[32]= objectTemp_ttn >> 24;  //Humedad
    appData[33]= objectTemp_ttn >> 16;
    appData[34]= objectTemp_ttn >> 8;
    appData[35]= objectTemp_ttn;

    medicionesnicla();

    long detec_ttn = d;
    long cont_ttn = c;
    long temp1_ttn = t*1000;
    long hum1_ttn = h*1000;
    long presion_ttn = p*1000;
    long gasesg_ttn = g*1000;

    long gyrox_ttn = gyrox*1000;
    long gyroy_ttn = gyroy*1000;
    long gyroz_ttn = gyroz*1000;

    long accx_ttn = accx*1000;
    long accy_ttn = accy*1000;
    long accz_ttn = accz*1000;

    long act_ttn = act;

    long iaq_reg_ttn = iaq_reg*10;
    long iaq_stat_ttn = iaq_stat*10;
    long bvoc_ttn = bvoc*100;
    long co2eq_ttn = co2eq*1000;

    long orix_ttn = orix*1000;
    long oriy_ttn = oriy*1000;
    long oriz_ttn = oriz*1000;

    appData[36]= detec_ttn >> 24;   //Detector de pasos
    appData[37]= detec_ttn >> 16; 
    appData[38]= detec_ttn >> 8; 
    appData[39]= detec_ttn; 

    appData[40]= cont_ttn >> 24;    //Contador de pasos
    appData[41]= cont_ttn >> 16; 
    appData[42]= cont_ttn >> 8; 
    appData[43]= cont_ttn; 

    appData[44]= temp1_ttn >> 24; //Temperatura
    appData[45]= temp1_ttn >> 16;
    appData[46]= temp1_ttn >> 8;
    appData[47]= temp1_ttn;

    appData[48]= hum1_ttn >> 24;  //Humedad
    appData[49]= hum1_ttn >> 16;
    appData[50]= hum1_ttn >> 8;
    appData[51]= hum1_ttn;

    appData[52]= presion_ttn >> 24;  //Presion
    appData[53]= presion_ttn >> 16;
    appData[54]= presion_ttn >> 8;
    appData[55]= presion_ttn;

    appData[56]= gasesg_ttn >> 24;  //gases. g
    appData[57]= gasesg_ttn >> 16;
    appData[58]= gasesg_ttn >> 8;
    appData[59]= gasesg_ttn;

    appData[60]= gyrox_ttn >> 8; //Giroscopio Eje x
    appData[61]= gyrox_ttn; 
    appData[62]= gyroy_ttn >> 8; //Giroscopio Eje y
    appData[63]= gyroy_ttn; 
    appData[64]= gyroz_ttn >> 8; //Giroscopio Eje z
    appData[65]= gyroz_ttn; 

    appData[66]= accx_ttn >> 8; //Magnometro Eje x
    appData[67]= accx_ttn; 
    appData[68]= accy_ttn >> 8; //Magnometro Eje y
    appData[69]= accy_ttn; 
    appData[70]= accz_ttn >> 8; //Magnometro Eje z
    appData[71]= accz_ttn; 

    appData[72]= act_ttn >> 8;  //Actividad
    appData[73]= act_ttn;

    appData[74]= iaq_reg_ttn >> 8; //Indice calidad de aire regulada
    appData[75]= iaq_reg_ttn;

    // appData[76]= iaq_stat_ttn >> 8; //Indice calidad de aire estacionaria
    // appData[77]= iaq_stat_ttn;

    appData[76]= bvoc_ttn >> 8; //Compuestos organicos volatiles
    appData[77]= bvoc_ttn;

    appData[78]= co2eq_ttn >> 24; //Co2 Equivalente
    appData[79]= co2eq_ttn >> 16;
    appData[80]= co2eq_ttn >> 8;
    appData[81]= co2eq_ttn;

    appData[82]= orix_ttn >> 8; //Acelerometro Eje x
    appData[83]= orix_ttn; 
    appData[84]= oriy_ttn >> 8; //Acelerometro Eje y
    appData[85]= oriy_ttn; 
    appData[86]= oriz_ttn >> 8; //Acelerometro Eje z
    appData[87]= oriz_ttn; 
    // Wire.endTransmission(80);

    Serial.println("------------SENSOR KY-038--------------------");

    // pinMode(Vext, OUTPUT);
    // digitalWrite(Vext, LOW);
    sensorky037();
    long sumattn = suma;
    appData[88]= sumattn >> 8;
    appData[89]= sumattn;
    Serial.print("Segundo 1: ");
    Serial.println(suma);

    sensorky037();
    long sumattn2 = suma;
    appData[90]= sumattn2 >> 8;
    appData[91]= sumattn2;
    Serial.print("Segundo 2: ");
    Serial.println(suma);

    sensorky037();
    long sumattn3 = suma;
    appData[92]= sumattn3 >> 8;
    appData[93]= sumattn3;
    Serial.print("Segundo 3: ");
    Serial.println(suma);

    sensorky037();
    long sumattn4 = suma;
    appData[94]= sumattn4 >> 8;
    appData[95]= sumattn4;
    Serial.print("Segundo 4: ");
    Serial.println(suma);
    
    uint16_t batteryVoltage = getBatteryVoltage();
    Serial.println();
    Serial.print(F("Bateria= "));
    Serial.print(batteryVoltage);

    appData[96] = (uint8_t)(batteryVoltage>>8);
    appData[97] = (uint8_t)batteryVoltage;

    float count = j++;
    long count_ttn = count;

    Serial.print(F(", Contador= "));
    Serial.println(count);

    appData[98]= count_ttn >> 8;
    appData[99]= count_ttn;

}

void setup() {
	Serial.begin(115200);
  Wire1.begin();
  Wire.begin();
  // pinMode(Vext, OUTPUT);
	// digitalWrite(Vext, LOW);

  // Inicializar scd30  
  if (airSensor.begin(Wire1, 400000) == false)
  {
    Serial.println("Air sensor not detected. Please check wiring. Freezing...");
    // while (1)
      ;
  }
// // Inicializar Max30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 no detectado. Please check wiring/power."));
    innerWdtEnable(false);
    while (1);
  }

    while( NO_ERR != sensor.begin() ){
    Serial.println("Communication with device failed, please check connection");
    // delay(3000);
  }
  Serial.println("Begin ok!");
  sensor.enterSleepMode();
  delay(50);
  sensor.enterSleepMode(false);
  delay(200);
  Serial.println("sensor MAX ok!");
  //caracteristicas para el funcionamiento sensor MAX
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  toma100datos();
  // maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

#if(AT_SUPPORT)
	enableAt();
#endif
	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
}

void loop()
{
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
#if(LORAWAN_DEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
			getDevParam();
#endif
			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
			deviceState = DEVICE_STATE_JOIN;
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
			prepareTxFrame( appPort );
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(txDutyCycleTime);
			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
			LoRaWAN.sleep();
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}
}
