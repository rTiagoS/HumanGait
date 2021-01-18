//Inclusão de bibliotecas necessárias para o funcionamento do script

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "MPU6050_6Axis_MotionApps_V6_12.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//Inclusão das bibliotecas de rede WiFi e MQTT
#include <WiFi.h>
#include <PubSubClient.h>

// Tópicos do Broker onde os dados coletados são publicados
const char mqtt_angle_csv[] = "/mpu6050/angle/csv";
const char mqtt_accel_csv[] = "/mpu6050/accel/csv";

//  Configuração de Rede Wi-Fi
char* ssid = "FAMILIARAMOS";
const char* password = "ferreira123";

// Configuração de Rede do Broker MQTT
const char* mqttServer = "192.168.0.4";
const int mqttPort = 1883;
//const char* mqttUser = "percebsz";
//const char* mqttPassword = "0612";

//Declaração das variáveis para controle de tempo de simulação
unsigned long TempoInicial;
unsigned long TempoFinal;
unsigned long PeriodoDeSimulacao;
unsigned long TimeSample; 
bool SIMULAR;

//Instaciar espClient
WiFiClient espClient;
PubSubClient client(espClient);


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT


// Definição dos pinos do ESP32 a serem usados. O pino 21 é para tráfego de dados e o pino
// 22 é para controle de clock. O pino 15 é para controle de interrupção.
#define INTERRUPT_PIN 15  // use pin 15 on Arduino Uno & most boards
#define SDA 21
#define SCL 22

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// Se a mensagem payload for igual a "iniciar", SIMULAR muda de False para True e o IF desencadeia
// diversas ações durante um periodo de simulação de 1 minuto dentro do while.
// o delay de 10 microsegundos representa mais ou menos a frequência de amostragem dos dados. A cada 10 microsegundos
// é publicado dados do sensor MPU6050.
// O retorno void significa ausência de retorno. Já um tipo void * significa um ponteiro genérico, um ponteiro de um tipo 
// desconhecido ou não especificado, um ponteiro para qualquer coisa, um endereço de memória qualquer.

void datasend_interval()
{
  if (SIMULAR)
  {
    TempoInicial = micros();
    TempoFinal = micros();
    PeriodoDeSimulacao = 5000000;
    int GuardaUltPos=0;
    int *PtrUltPos;
    PtrUltPos = &GuardaUltPos;

    while (((TempoFinal - TempoInicial) <= PeriodoDeSimulacao or n <= NumeroDeAmostras) and n <= NumeroDeAmostras)
    {
      TimeSample = (TempoFinal - TempoInicial);
      mpu_loop();
      delayMicroseconds(50);
      mpu.resetFIFO();
      TempoFinal = micros();  
    }
    client.publish(mqtt_flag_simulacao, 'SimulacaoFinalizada);
    
  }
}n <= NumeroDeAmostras

// Definição função callback para, posteriormente, desencadear o início da simulação verificando se
// a mensagem payload é igual a "iniciar". Se a mensagem payload for "iniciar", a variável booleana
// SIMULAR recebe muda o estado de False para True, desencadeando o início da simulação.
void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived: ");
  Serial.print(topic);
  String s = "";
  for (int i = 0; i < length; i++)
  {
    Serial.println((char)payload[i]);
    s += ((char)payload[i]);
  }
  Serial.println(s);
  if (s.equals("iniciar"))
  {
    SIMULAR = true; 
  }
  else
  {
    SIMULAR = false;
  }
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

//O objetivo desta função é implementar uma estrutura de dados do tipo FIFO para armazenar um número de n amostras de dados coletados
//a partir do sensor MPU6050. O armazenamento vai ficar temporariamente no ESP32 e depois será enviado ao servidor broker. Após isso,
//ele deverá liberar o espaço na memória que foi previamente alocada. 
/*memset(buf, 0, sizeof(buf));
    sprintf(buf,"%d,%d,%d,%.2f,%.2f,%.2f,%d",aaReal.x,aaReal.y,aaReal.z,(ypr[0] * 180 / M_PI),(ypr[1] * 180 / M_PI),(ypr[2] * 180 / M_PI), TimeSample);
    client.publish(mqtt_accel_csv, buf);
*/

void BigBuffer (char *GaitDataStream[], int NumeroDeAmostras)
{
/*  if (SimulacaoFinalizou)
  {
    *PtrUltPos = 0; 
  }
  elif (GuardaUltPos < (NumeroDeAmostras*QuantidadeDeVariaveis - 1))
*/
// {
    GaitDataStream[GuardaUltPos+0]=aaReal.x;
    GaitDataStream[GuardaUltPos+1]=aaReal.y;
    GaitDataStream[GuardaUltPos+2]=aaReal.z;
    GaitDataStream[GuardaUltPos+3]=(ypr[0] * 180 / M_PI);
    GaitDataStream[GuardaUltPos+4]=(ypr[1] * 180 / M_PI);
    GaitDataStream[GuardaUltPos+5]=(ypr[2] * 180 / M_PI);
    *PtrUltPos= (*PtrUltPos) +6;  
// }
/*
  else
  {
    SimulacaoFinalizou = true;
  }
*/  
}

void PublishBigBuffer ()
{
  
}
void mpu_setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(SDA, SCL, 400000);
  //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  /*// wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  */

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51); 
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150); //1150
  mpu.setYAccelOffset(-50);  //-50
  mpu.setZAccelOffset(1060); //1060
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6); 
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  } 
}

void setup_wifi()
{
 
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
 
  Serial.println("Connected to the WiFi network");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() 
{ 
  while (!client.connected())
  {
    Serial.print("Connecting to MQTT...");

    if (client.connect("ESP32mellitus"))
    {
      Serial.println("connected");
      client.subscribe("/mpu6050/listener");
    }
    else
    {
      Serial.print("failed with state");
      Serial.print(client.state());
      delay(2000); 
    }
  }
}

void setup(void)
{
  Serial.begin(115200);

  setup_wifi();
  
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  mpu_setup();
}

void mpu_loop()
{
//Variável buffer para enviar dados dos sensores para o broker mqtt e ser limpada a cada laço
char buf[50];
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    /*
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI); */
    
    /*
      mpu.dmpGetAccel(&aa, fifoBuffer);
      Serial.print("\tRaw Accl XYZ\t");
      Serial.print(aa.x);
      Serial.print("\t");
      Serial.print(aa.y);
      Serial.print("\t");
      Serial.print(aa.z);
      mpu.dmpGetGyro(&gy, fifoBuffer);
      Serial.print("\tRaw Gyro XYZ\t");
      Serial.print(gy.x);
      Serial.print("\t");
      Serial.print(gy.y);
      Serial.print("\t");
      Serial.print(gy.z);
    */
    //Serial.println();

#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    /*
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
    */

    memset(buf, 0, sizeof(buf));
    sprintf(buf,"%d,%d,%d,%.2f,%.2f,%.2f,%d",aaReal.x,aaReal.y,aaReal.z,(ypr[0] * 180 / M_PI),(ypr[1] * 180 / M_PI),(ypr[2] * 180 / M_PI), TimeSample);
    client.publish(mqtt_accel_csv, buf);

    
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
  }
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() 
{
  if (!client.connected())
  {
    reconnect();
  }
  

  client.loop(); //Loop da biblioteca PubSubClient que fica chamando a função callback.

  datasend_interval(); //função que captura e envia os dados dos sensores para o broker junto com o tempo durante um período de simulação fixo
}
