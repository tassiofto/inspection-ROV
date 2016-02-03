// Biblioteca usada para para comunicação I2C
#include <Wire.h>

// Inclui funções matemáticas, como sin, cos, sqrt...
#include <Math.h>

// Contém dados gerais do MPU-6050 e outras constantes
#include "dados.h"


// ================================================================
// ===            MPU           ===
// ================================================================

#define MOSTRAR_YAWPITCHROLL    // Descomentar para mostrar os ângulos Yaw, Pitch e Roll
#define MPU6050_ADD       0x68  // Endereço de I2C mais comum do MPU-6050
#define LED_PIN         13    // Indicativo de funcionamento

bool blinkState = false;        // Estado inicial do LED
bool dmpReady = false;          // Habilita o uso da interrupção em loop()
uint8_t mpuIntStatus;         // Guarda o status atual da interrupção do MPU
uint16_t packetSize;          // Tamanho do pacote de dados do DMP (padrão é 42 bytes)
uint16_t fifoCount;           // Armazena a quantidade de bites no FIFO
uint8_t fifoBuffer[64];         // Buffer para armazenar FIFO
uint8_t buffer[2];



// ================================================================
// ===            PSX            ===
// ================================================================
                   #include <Psx.h>                                          // Includes the Psx Library 
             // Any pins can be used since it is done in software
                   #define dataPin 30
                   #define cmndPin 31
                   #define attPin 32
                   #define clockPin 33
                   Psx Psx;  // Initializes the library

                   unsigned int data = 0; // data stores the controller response



// ================================================================
// ===            Servos           ===
// ================================================================
#include <Servo.h>
#include <I2Cdev.h>
Servo myservox,myservoy;  // create servo object to control a servo 
int posx = 65;    // variable to store the servo position 
int posy = 80;



// ================================================================
// ===            Controle Motores         ===
// ================================================================

int Ddd = 0, Vdd = 0, Dde = 0, Vde = 0, Dtd = 0, Vtd = 0, Dte = 0, Vte = 0, Did = 0, Vid = 0, Dit = 0, Vit = 0, De = 0, Ve = 0;

//pinagem dos motores
//-motores id e it:
const int PWMid = 2;  
const int Did1 = 50; 
const int Did2 = 51;
const int PWMit = 3; 
const int Dit1 = 52; 
const int Dit2 = 53;
//-motores de e td:
const int PWMde = 4; 
const int Dde1 = 46; 
const int Dde2 = 47; 
const int PWMtd = 5; 
const int Dtd1 = 48; 
const int Dtd2 = 49; 
//-motores dd e te:
const int PWMdd = 6; 
const int Ddd1 = 40;
const int Ddd2 = 41; 
const int PWMte = 7; 
const int Dte1 = 42; 
const int Dte2 = 43; 
//-motores empuxo:
const int PWMe = 8; 
const int De1 = 36; 
const int De2 = 37; 



// ================================================================
// ===        outros            ===
// ================================================================
int flag1 = 0;
int flag2 = 0;

const int LEDPin = 13; //   Telltale LED
const int Duration = 2000;
const int LEDg = 22;
const int LEDy = 23;
const int LEDr = 24;

// ================================================================
// ===            ROTINA DE DETECÇÃO DE INTERRUPÇÃO             ===
// ================================================================

volatile bool mpuInterrupt = false;   // Indica se o pino de interripção do MPU foi a HIGH
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===            PID           ===
// ================================================================
#include <PID_v1.h>

int Dx1,Dx2,Dy1,Dy2,Dz1,Dz2;

const float xoffset = 0;
const float yoffset = 0;

double Setpointx, Inputx, Outputx; // these are just variables for storing values
float Kpx=2; //Initial Proportional Gain 
float Kix=5; //Initial Integral Gain 
float Kdx=1; //Initial Differential Gain 
PID myPIDx(&Inputx, &Outputx, &Setpointx,Kpx,Kix,Kdx, DIRECT);

double Setpointy, Inputy, Outputy; // these are just variables for storing values
float Kpy=2; //Initial Proportional Gain 
float Kiy=5; //Initial Integral Gain 
float Kdy=1; //Initial Differential Gain 
PID myPIDy(&Inputy, &Outputy, &Setpointy,Kpy,Kiy,Kdy, DIRECT);

double Setpointz, Inputz, Outputz; // these are just variables for storing values
float Kpz=2; //Initial Proportional Gain 
float Kiz=5; //Initial Integral Gain 
float Kdz=1; //Initial Differential Gain 
PID myPIDz(&Inputz, &Outputz, &Setpointz,Kpz,Kiz,Kdz, DIRECT);

typedef struct Quaternion {        // Define um Quaternion (vetor de quatro dimensões)
  float w;
  float x;
  float y;
  float z;
};

typedef struct VectorFloat {      // Define um vetor tridimensional
  float x;
  float y;
  float z;
};

const int sampleRate = 1; // Variable that determines how fast our PID loop runs

Quaternion q;        // Armazena os quaternions do DMP (NÃO sofre Gimbal Lock)
 VectorFloat gravity;    // Armazena o vetor gravidade (nos eixos X, Y e Z)
  float ypr[3];       // Armazena os ângulos Yaw, Pitch e Roll (sofre Gimbal Lock)
  float euler[3];       // Armazena os ângulos de Euler (sofre Gimbal Lock)



// ================================================================
// ===           Thread       ===
// ================================================================
#include <Thread.h>
#include <ThreadController.h>
// ThreadController that will controll all threads
ThreadController controll = ThreadController();

const int PIDx_TIME = 20;
const int PIDy_TIME = 20;
const int PIDz_TIME = 20;
const int MPU_TIME = 40;
const int psxcam_TIME = 20;
const int psx_TIME = 10;
const int motores_TIME = 15;

Thread Thread_1 = Thread();
Thread Thread_2 = Thread();
Thread Thread_3 = Thread();
Thread Thread_4 = Thread();
Thread Thread_5 = Thread();
Thread Thread_6 = Thread();
Thread Thread_7 = Thread();

int contador=0;// callback for myThread



// ================================================================
// ================================================================
// ================================================================
// ================================================================
// ===            SETUP         ===
// ================================================================
// ================================================================
// ================================================================
// ================================================================
void setup(){
  Serial.begin(9600);

// ================================================================
// ===            servo config            ===
// ================================================================
myservox.attach(11);  // attaches the servo on pin 9 to the servo object 
myservoy.attach(10);  // attaches the servo on pin 9 to the servo object  



// ================================================================
// ===            psx config            ===
// ================================================================
Psx.setupPins(dataPin, cmndPin, attPin, clockPin, 10);  // Defines what each pin is used
                                                          // (Data Pin #, Cmnd Pin #, Att Pin #, Clk Pin #, Delay)
                                                          // Delay measures how long the clock remains at each state,
                                                          // measured in microseconds.
                                                          // too small delay may not work (under 5)
          

// ================================================================
// ===            controla motores config            ===
// ================================================================
pinMode(PWMdd, OUTPUT);
pinMode(PWMde, OUTPUT);
pinMode(PWMtd, OUTPUT);
pinMode(PWMte, OUTPUT);
pinMode(PWMid, OUTPUT);
pinMode(PWMit, OUTPUT);
pinMode(PWMe, OUTPUT);

 pinMode(Ddd1, OUTPUT);
 pinMode(Ddd2, OUTPUT);
 pinMode(Dde1, OUTPUT);
 pinMode(Dde2, OUTPUT);
 pinMode(Dtd1, OUTPUT);
 pinMode(Dtd2, OUTPUT);
 pinMode(Dte1, OUTPUT);
 pinMode(Dte2, OUTPUT);
 pinMode(Did1, OUTPUT);
 pinMode(Did2, OUTPUT);
 pinMode(Dit1, OUTPUT);
 pinMode(Dit2, OUTPUT);
 pinMode(De1, OUTPUT);
 pinMode(De2, OUTPUT);



// ================================================================
// ===            pid config            ===
// ================================================================
  Wire.begin();    // Inicia I2C
  initialize();
  uint8_t devStatus;            // Retorna 0 se a inicialização do DMP ocorreu bem
  devStatus = dmpInitialize();
  // Define os Offsets. **CADA** dispositivo possui valores **ÚNICOS** (esse funciona apenas para o meu MPU-5060)
  // Valores obtidos com o uso do arquivo MPU6050_calibration.ino
  setXGyroOffset(162);
  setYGyroOffset(-59);
  setZGyroOffset(-55);
  setXAccelOffset(-1546);
  setYAccelOffset(-117);
  setZAccelOffset(5481);

     // Caso a inicialização ocorreu bem (retorno 0)
  if(devStatus == 0){
    // Habilita a detecção de interrupções do Arduino (External Interrup 0)
   //    Serial.println("Habilitando interrupcoes do Arduino...");
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = getIntStatus();
    // Altera dmpReady para que a interrupção possa ser usada em loop()
   //    Serial.println("DMP Pronto! Esperando pela primeira interrupcao...");
    dmpReady = true;
  }

  Setpointx = 0; // Read the SetPoint value from the potentiometer
  myPIDx.SetMode(AUTOMATIC); // Turn on the PID loop as automatic control
  myPIDx.SetSampleTime(sampleRate); // Sets the sample rate

  Setpointy = 0; // Read the SetPoint value from the potentiometer
  myPIDy.SetMode(AUTOMATIC); // Turn on the PID loop as automatic control
  myPIDy.SetSampleTime(sampleRate); // Sets the sample rate

  Setpointz = Inputz; // Read the SetPoint value from the potentiometer
  myPIDz.SetMode(AUTOMATIC); // Turn on the PID loop as automatic control
  myPIDz.SetSampleTime(sampleRate); // Sets the sample rate
  

// ================================================================
// ===            thread config         ===
// ================================================================
  Thread_1.onRun(PIDx);
  Thread_1.setInterval(PIDx_TIME);

  Thread_2.onRun(PIDy);
  Thread_2.setInterval(PIDy_TIME);

  Thread_3.onRun(PIDz);
  Thread_3.setInterval(PIDz_TIME);

  Thread_4.onRun(MPU);
  Thread_4.setInterval(MPU_TIME);

  Thread_5.onRun(psxcam);
  Thread_5.setInterval(psxcam_TIME);

  Thread_6.onRun(psx);
  Thread_6.setInterval(psx_TIME);

  Thread_7.onRun(Motores);
  Thread_7.setInterval(motores_TIME);
  // Adds both threads to the controller
  controll.add(&Thread_1);
  controll.add(&Thread_2);
  controll.add(&Thread_3);
  controll.add(&Thread_4);
  controll.add(&Thread_5);
  controll.add(&Thread_6);
  controll.add(&Thread_7);  



// ================================================================
// ===            outras config            ===
// ================================================================

  pinMode(LEDPin, OUTPUT);  
  pinMode(LEDg, OUTPUT);
  pinMode(LEDy, OUTPUT);
  pinMode(LEDr, OUTPUT);

    Thread_1.enabled = false ;
    Thread_2.enabled = false ;
    Thread_3.enabled = false ;
    Thread_4.enabled = true ;
    Thread_5.enabled = true ;
    Thread_6.enabled = true ;
    Thread_7.enabled = true ;

}/////Fecha Config//////



// ================================================================
// ================================================================
// ================================================================
// ================================================================
// ===            LOOP         ===
// ================================================================
// ================================================================
// ================================================================
// ================================================================
void loop(){
  
controll.run();
Serial.print(Inputx );
Serial.print("::");
Serial.print(Inputy);
Serial.print("::");
Serial.println(Inputz);

if (data & psxUp || data & psxDown)//frente tras
  {
    Thread_1.enabled = true ;
    Thread_2.enabled = true ;
    Thread_3.enabled = true ;
    flag1 += 1;
    flag2 += 1;
    psxfrentetras();
  }
else
{
  De = 0;
}
if (data & psxRight || data & psxLeft)//sobe ou desce
  {
    Thread_1.enabled = false ;
    Thread_2.enabled = false ;
    Thread_3.enabled = true ;
    flag1 += 1;
    psxsobedesce();
  }
else 
{
Ddd = 0, Dde = 0, Dtd = 0, Dte = 0;
}

if (data & psxL1 || data & psxR1 || data & psxL2 || data & psxR2)//gira ou anda de lado
  {
    Thread_1.enabled = true ;
    Thread_2.enabled = true ;
    Thread_3.enabled = false ;
    flag1 += 1;
    psxinferiores();
  }
else
  {
    Did = 0, Dit = 0;
  }

if (flag1 == 0 || data == 0)//liga todos threads pid se nenhum motor for solicitado
  {                        // encontrar operacao de exclusao para trocar a de data == 0
    Thread_1.enabled = true ;
    Thread_2.enabled = true ;
    Thread_3.enabled = true ;
    flag1 = 0;
  }

if (flag2 == 0)//desliga frente e tras se n estiver sendo usado
  {
    De = 0;
  }

//habilita PIDxyz se nenhum dos comandos de navegacao for executado
/*if (data != psxUp || data != psxDown || data != psxRight || data != psxLeft  || data != psxL1 || data != psxR1 || data != psxL2 || data != psxR2)
  {
    Thread_1.enabled = true ;
    Thread_2.enabled = true ;
    Thread_3.enabled = true ;
  }
 */

}




// ================================================================
// ===           Funcoes Auxiliares          ===
// ================================================================

void psx(){
data = Psx.read();
}

void PIDx(){
 if(Inputx < 0)
  {
  Ddd = 1;
  Dde = 2;    
  myPIDx.SetControllerDirection(DIRECT);
  }
 else
  {
  Ddd = 2;
  Dde = 1;    
  myPIDx.SetControllerDirection(REVERSE);
  }

  //Setpointx = 0; // Read our setpoint light Level from the potentiometer 
  myPIDx.Compute(); // Run the PID loop
  //digitalWrite(outputPin, Outputx); // Write out the output from the PID loop to our LED pin
  Vdd = Outputx;
  Vde = Outputx;
  Serial.println("pid x");
}

void PIDy(){

if(Inputy < 0)
  {
  Dtd = 1;
  Dte = 1;     
  myPIDy.SetControllerDirection(DIRECT);
  }
 else
  {
  Dtd = 2;
  Dte = 2;     
  myPIDy.SetControllerDirection(REVERSE);
  }

  //Setpointy = 0; // Read our setpoint light Level from the potentiometer 
  myPIDy.Compute(); // Run the PID loop
  //digitalWrite(outputPin, Outputy); // Write out the output from the PID loop to our LED pin
Vtd = Outputy;
Vte = Outputy;
Serial.println("       pid y");
}

void PIDz(){

 if(Inputz < 0)
  {
  Did = 1;
  Dit = 1;  
  myPIDz.SetControllerDirection(DIRECT);
  }
 else
  {
  Did = 2;
  Dit = 2; 
  myPIDz.SetControllerDirection(REVERSE);
  }

  //Setpointz = Inputz; // ver como salvar posicao atual do rov para o pid atuar..
  myPIDz.Compute(); // Run the PID loop
  //digitalWrite(outputPin, Outputz); // Write out the output from the PID loop to our LED pin
  Vid = Outputz;
  Vit = Outputz;
  Serial.println("              pid z");
}

// callback for hisThread
void MPU(){

/* Quaternion q;        // Armazena os quaternions do DMP (NÃO sofre Gimbal Lock)
 VectorFloat gravity;    // Armazena o vetor gravidade (nos eixos X, Y e Z)
  float ypr[3];       // Armazena os ângulos Yaw, Pitch e Roll (sofre Gimbal Lock)
  float euler[3];       // Armazena os ângulos de Euler (sofre Gimbal Lock)
*/
  // Se houve algum erro, o programa não continua além daqui
  if(!dmpReady)
    return;

  while (!mpuInterrupt && fifoCount < packetSize);

  // Reseta a interrupção o obtén o valor de INT_STATUS
  mpuInterrupt = false;
  mpuIntStatus = getIntStatus();

  // Obtén o tamanho atual do FIFO
  fifoCount = getFIFOCount();

  // Verifica se há overflow...
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // Reinicia e limpa o FIFO
    resetFIFO();
    Serial.println("FIFO overflow!");

  // ...senão checa por dados do DMP
  } else if (mpuIntStatus & 0x02){
    // Espera pela quantidade correta de dados no FIFO
    while (fifoCount < packetSize)
      fifoCount = getFIFOCount();

    // Lê um pacote de dados do FIFO
    getFIFOBytes(fifoBuffer, packetSize);

    // Checa se já há dados no FIFO
    fifoCount -= packetSize;

    // Obter ângulos de Yaw, Pitch e Roll (ypr[2], ypr[1] e ypr[0])
    #ifdef MOSTRAR_YAWPITCHROLL
      dmpGetQuaternion(&q, fifoBuffer);
      dmpGetGravity(&gravity, &q);
      dmpGetYawPitchRoll(ypr, &q, &gravity);
   //   Serial.print("DMP:");
   //   Serial.print(ypr[2] * 180/M_PI, 2);
   //   Serial.print("::");
   //   Serial.print(ypr[1] * 180/M_PI, 2);
   //   Serial.print("::");
   //   Serial.println(ypr[0] * 180/M_PI, 2);
    #endif 

    // Pisca o LED para indicar que o programa está rodando (acontece MUITO rápido!)
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
  }

 Inputx = ypr[2] * 180/M_PI - xoffset ;
 Inputy = ypr[1] * 180/M_PI - yoffset ;
 Inputz = ypr[0] * 180/M_PI ;
}


void statusLED()
{ 
digitalWrite(LEDg, HIGH);
digitalWrite(LEDy, HIGH);
digitalWrite(LEDr, HIGH);
}


void psxcam()
{ 
 if (data == 264)                                                                                                                                                        
  {
    if (posx < 130)
    posx += 1;                                
  }  
 if (data == 258)                                                                                                                                                        
  {
    if (posx > 0)
    posx -= 1;                                             
  }
 if (data == 260)                                       
  {
    if (posy < 160)
    posy += 1;
  }
 if (data == 257)                                       
  {
   if (posy > 10)
    posy -= 1;
  }

 myservox.write(posx);
 myservoy.write(posy);
}

void psxinferiores()
{
    if (data & psxR2)   //gira para direita              
     {                                                    
      Did = 1, Vid = 250, Dit = 1, Vit = 250; 
      Setpointz = Inputz ;                      
     }
    if (data & psxL2)//gira para esquerda
     {
      Did = 2, Vid = 250, Dit = 2, Vit = 250; 
      Setpointz = Inputz ; 
     }
    if (data & psxR1)//anda para direita
     {
      Did = 1, Vid = 250, Dit = 2, Vit = 250; 
     }
    if (data & psxL1)//anda para esquerda
      {
       Did = 2, Vid = 250, Dit = 1, Vit = 250; 
      }
}


void psxfrentetras()
{
 if (data & psxUp)//anda para frente
   {
    De = 1, Ve = 250;
   }
 if (data & psxDown)//anda para tras
   {
    De = 2, Ve = 250; 
   }

}


void psxsobedesce()
{
 if (data & psxLeft)//desce
  {
    Ddd = 2, Vdd = 250, Dde = 2, Vde = 250, Dtd = 2, Vtd = 250, Dte = 2, Vte = 250; 
  }
 if (data & psxRight)//sobe
  {
    Ddd = 1, Vdd = 250, Dde = 1, Vde = 250, Dtd = 1, Vtd = 250, Dte = 1, Vte = 250; 
  }
}


void Motores()
{

    if (Ddd != 0)
    {
              if (Ddd == 1)
              {
              digitalWrite(Ddd1, HIGH);                                                     
              digitalWrite(Ddd2, LOW);                                                       
              }
              if (Ddd == 2)
              {
              digitalWrite(Ddd1, LOW);                                                       
              digitalWrite(Ddd2, HIGH);
              }     
              analogWrite(PWMdd, Vdd);//verificar se colocar dentro ou fora####### 
                                                                                            
    }
    else
    {
        digitalWrite(Ddd1, LOW);                                                               
        digitalWrite(Ddd2, LOW);
        analogWrite(PWMdd, 0);//verfificar se interfere na performance########
    }

    if (Dde != 0)
    {
              if (Dde == 1)
              {
              digitalWrite(Dde1, HIGH);
              digitalWrite(Dde2, LOW);
              }
              if (Dde == 2)
              {
              digitalWrite(Dde1, LOW);
              digitalWrite(Dde2, HIGH);
              }     
              analogWrite(PWMde, Vde);//verificar se colocar dentro ou fora#######
    }
    else
    {
        digitalWrite(Dde1, LOW);
        digitalWrite(Dde2, LOW);
        analogWrite(PWMde, 0);//verfificar se interfere na performance########
    }

    if (Dtd != 0)
    {
              if (Dtd == 1)
              {
              digitalWrite(Dtd1, HIGH);
              digitalWrite(Dtd2, LOW);
              }
              if (Dtd == 2)
              {
              digitalWrite(Dtd1, LOW);
              digitalWrite(Dtd2, HIGH);
              }     
              analogWrite(PWMtd, Vtd);//verificar se colocar dentro ou fora#######
    }
    else
    {
        digitalWrite(Dtd1, LOW);
        digitalWrite(Dtd2, LOW);
        analogWrite(PWMtd, 0);//verfificar se interfere na performance########
    }

    if (Dte != 0)
    {
              if (Dte == 1)
              {
              digitalWrite(Dte1, HIGH);
              digitalWrite(Dte2, LOW);
              }
              if (Dte == 2)
              {
              digitalWrite(Dte1, LOW);
              digitalWrite(Dte2, HIGH);
              }     
              analogWrite(PWMte, Vte);//verificar se colocar dentro ou fora#######
    }
    else
    {
        digitalWrite(Dte1, LOW);
        digitalWrite(Dte2, LOW);
        analogWrite(PWMte, 0);//verfificar se interfere na performance########
    }


    if (Did != 0)
    {
              if (Did == 1)
              {
              digitalWrite(Did1, HIGH);
              digitalWrite(Did2, LOW);
              }
              if (Did == 2)
              {
              digitalWrite(Did1, LOW);
              digitalWrite(Did2, HIGH);
              }     
              analogWrite(PWMid, Vid);//verificar se colocar dentro ou fora#######
    }
    else
    {
        digitalWrite(Did1, LOW);
        digitalWrite(Did2, LOW);
        analogWrite(PWMid, 0);//verfificar se interfere na performance########
    }

    if (Dit != 0)
    {
              if (Dit == 1)
              {
              digitalWrite(Dit1, HIGH);
              digitalWrite(Dit2, LOW);
              }
              if (Dit == 2)
              {
              digitalWrite(Dit1, LOW);
              digitalWrite(Dit2, HIGH);
              }     
              analogWrite(PWMit, Vit);//verificar se colocar dentro ou fora#######
    }
    else
    {
        digitalWrite(Dit1, LOW);
        digitalWrite(Dit2, LOW);
        analogWrite(PWMit, 0);//verfificar se interfere na performance########
    }

    if (De != 0)
    {
              if (De == 1)
              {
              digitalWrite(De1, HIGH);
              digitalWrite(De2, LOW);
              }
              if (De == 2)
              {
              digitalWrite(De1, LOW);
              digitalWrite(De2, HIGH);
              }     
              analogWrite(PWMe, Ve);//verificar se colocar dentro ou fora#######
    }
    else
    {
        digitalWrite(De1, LOW);
        digitalWrite(De2, LOW);
        analogWrite(PWMe, 0);//verfificar se interfere na performance########
    }

}






// ================================================================
// ===                   FUNÇÕES DO MPU-5060                    ===
// ================================================================

void initialize(){
  setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  writeBits(MPU6050_ADD, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2); //setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  writeBit(MPU6050_ADD, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, false);  //setSleepEnabled(false);
}

uint8_t dmpInitialize(){
  writeBit(MPU6050_ADD, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);  //reset(); Reseta totalmente o dispositivo
  delay(30);    // Espera a reinicialização

  // Desabilita modo sleep
  writeBit(MPU6050_ADD, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, false);  //setSleepEnabled(false);

  // Obtém a versão do MPU (?)
  setMemoryBank(0x10, true, true);
  setMemoryStartAddress(0x06);
  setMemoryBank(0, false, false);

  // Define algumas coisas desconhecidas (I2C auxiliar?)
  setSlaveAddress(0, 0x7F);
  writeBit(MPU6050_ADD, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, false);  //setI2CMasterModeEnabled(false);
  setSlaveAddress(0, 0x68);
  writeBit(MPU6050_ADD, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, true);  //resetI2CMaster();
  delay(20);

  // Carregar o código do DMP para a memória do MPU
  if(writeMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0, true, true)){
    if (writeDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE, true)){
      setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
      writeByte(MPU6050_ADD, MPU6050_RA_INT_ENABLE, 0x12);  //setIntEnabled(0x12);
      writeByte(MPU6050_ADD, MPU6050_RA_SMPLRT_DIV, 4);   //setRate(4); // 1khz / (1 + 4) = 200 Hz
      writeBits(MPU6050_ADD, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, MPU6050_EXT_SYNC_TEMP_OUT_L);  //setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);
      writeBits(MPU6050_ADD, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42); //setDLPFMode(MPU6050_DLPF_BW_42);
      setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
      writeByte(MPU6050_ADD, MPU6050_RA_DMP_CFG_1, 0x03); //setDMPConfig1(0x03);
      writeByte(MPU6050_ADD, MPU6050_RA_DMP_CFG_2, 0x00); //setDMPConfig2(0x00);
      writeBit(MPU6050_ADD, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, false);  //setOTPBankValid(false);
      
      uint8_t dmpUpdate[16], j;
      uint16_t pos = 0;

      // Escreve atualização final de memória 1/7 (função desconhecida)
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      
      writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true, false);

      // Escreve atualização final de memória 2/7 (função desconhecida)
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      
      writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true, false);

      resetFIFO();
      getFIFOBytes(fifoBuffer, fifoCount);
      writeByte(MPU6050_ADD, MPU6050_RA_MOT_THR, 2);    //setMotionDetectionThreshold(2);
      writeByte(MPU6050_ADD, MPU6050_RA_ZRMOT_THR, 156);  //setZeroMotionDetectionThreshold(156);
      writeByte(MPU6050_ADD, MPU6050_RA_MOT_DUR, 80);   //setMotionDetectionDuration(80);
      writeByte(MPU6050_ADD, MPU6050_RA_ZRMOT_DUR, 0);  //setZeroMotionDetectionDuration(0);
      resetFIFO();
      writeBit(MPU6050_ADD, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, true);  //setFIFOEnabled(true);
      writeBit(MPU6050_ADD, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, true);   //setDMPEnabled(true);
      writeBit(MPU6050_ADD, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);  //resetDMP();

      // Escreve atualização final de memória 3/7 (função desconhecida)
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true, false);

      // Escreve atualização final de memória 4/7 (função desconhecida)
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true, false);

      // Escreve atualização final de memória 5/7 (função desconhecida)
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true, false);

      while ((fifoCount = getFIFOCount()) < 3);
      getFIFOBytes(fifoBuffer, fifoCount);

      // Escreve atualização final de memória 6/7 (função desconhecida)
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

      while ((fifoCount = getFIFOCount()) < 3);
      mpuIntStatus = getIntStatus();

      // Escreve atualização final de memória 7/7 (função desconhecida)
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);

      writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true, false);
      packetSize = 42;

      resetFIFO();
    } else {
      return 2; // Código de falha na atualização das configurações do MPU
    }
  } else {
    return 1; // Código de falha na inicialização da memória do MPU
  }
  return 0; // Código para inicialização bem-sucedida 
}

void dmpGetQuaternion(struct Quaternion *q, const uint8_t* packet){
  int16_t qI[4];

  qI[0] = ((packet[0] << 8) + packet[1]);
  qI[1] = ((packet[4] << 8) + packet[5]);
  qI[2] = ((packet[8] << 8) + packet[9]);
  qI[3] = ((packet[12] << 8) + packet[13]);

  q -> w = (float)qI[0] / 16384.0f;
  q -> x = (float)qI[1] / 16384.0f;
  q -> y = (float)qI[2] / 16384.0f;
  q -> z = (float)qI[3] / 16384.0f;
}

void dmpGetGravity(struct VectorFloat *v, struct Quaternion *q){
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
}

void dmpGetYawPitchRoll(float *data, struct Quaternion *q, struct VectorFloat *gravity){
  data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);  // Yaw (Guinada): em torno do eixo Z
  data[1] = -atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));// Pitch (Arfagem): em torno do eixo Y
  data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z)); // Roll (Rolagem): em torno do eixo X
}

uint8_t dmpGetEuler(float *data, struct Quaternion *q){
  data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);  // psi
  data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                     // theta
  data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);  // phi
  return 0;
}


// ================================================================
// ===                    FUNÇÕES DE AOPIO                      ===
// ================================================================

void setClockSource(uint8_t source){
  writeBits(MPU6050_ADD, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void setFullScaleGyroRange(uint8_t range){
  writeBits(MPU6050_ADD, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void resetFIFO(){
  writeBit(MPU6050_ADD, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}

uint16_t getFIFOCount() {
  readBytes(MPU6050_ADD, MPU6050_RA_FIFO_COUNTH, 2, buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
  return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

void getFIFOBytes(uint8_t *data, uint8_t length){
  readBytes(MPU6050_ADD, MPU6050_RA_FIFO_R_W, length, data, I2CDEV_DEFAULT_READ_TIMEOUT);
}

uint8_t getIntStatus() {
  readByte(MPU6050_ADD, MPU6050_RA_INT_STATUS, buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
  return buffer[0];
}

void setSlaveAddress(uint8_t num, uint8_t address){
  if(num > 3)
    return;

  writeByte(MPU6050_ADD, MPU6050_RA_I2C_SLV0_ADDR + num*3, address);
}

void setXGyroOffset(uint16_t offset){
  writeWords(MPU6050_ADD, MPU6050_RA_XG_OFFS_USRH, 1, &offset);
}

void setYGyroOffset(uint16_t offset){
  writeWords(MPU6050_ADD, MPU6050_RA_YG_OFFS_USRH, 1, &offset);
}

void setZGyroOffset(uint16_t offset){
  writeWords(MPU6050_ADD, MPU6050_RA_ZG_OFFS_USRH, 1, &offset);
}

void setXAccelOffset(uint16_t offset) {
    writeWords(MPU6050_ADD, MPU6050_RA_XA_OFFS_H, 1, &offset);
}

void setYAccelOffset(uint16_t offset) {
    writeWords(MPU6050_ADD, MPU6050_RA_YA_OFFS_H, 1, &offset);
}

void setZAccelOffset(uint16_t offset){
  writeWords(MPU6050_ADD, MPU6050_RA_ZA_OFFS_H, 1, &offset);
}

void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
  setMemoryBank(bank, false, false);
  setMemoryStartAddress(address);
  uint8_t chunkSize;

  for (uint16_t i = 0; i < dataSize;){
    // determine correct chunk size according to bank position and data size
    chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

    // make sure we don't go past the data size
    if (i + chunkSize > dataSize)
      chunkSize = dataSize - i;

    // make sure this chunk doesn't go past the bank boundary (256 bytes)
    if (chunkSize > 256 - address)
      chunkSize = 256 - address;

    // read the chunk of data as specified
    readBytes(MPU6050_ADD, MPU6050_RA_MEM_R_W, chunkSize, data + i, I2CDEV_DEFAULT_READ_TIMEOUT);
        
    i += chunkSize;     // increase byte index by [chunkSize]
    address += chunkSize; // uint8_t automatically wraps to 0 at 256

    // if we aren't done, update bank (if necessary) and address
    if (i < dataSize){
      if (address == 0)
        bank++;

      setMemoryBank(bank, false, false);
      setMemoryStartAddress(address);
    }
  }
}

bool writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem){
  setMemoryBank(bank, false, false);
  setMemoryStartAddress(address);
  uint8_t chunkSize;
  uint8_t *verifyBuffer;
  uint8_t *progBuffer;
  uint16_t i;
  uint8_t j;

  if (verify)
    verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);

  if (useProgMem)
    progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);

  for (i = 0; i < dataSize;) {
    // determine correct chunk size according to bank position and data size
    chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

    // make sure we don't go past the data size
    if (i + chunkSize > dataSize)
      chunkSize = dataSize - i;

    // make sure this chunk doesn't go past the bank boundary (256 bytes)
    if (chunkSize > 256 - address)
      chunkSize = 256 - address;

    if (useProgMem) {
      for (j = 0; j < chunkSize; j++) // write the chunk of data as specified
        progBuffer[j] = pgm_read_byte(data + i + j);
    } else {
      progBuffer = (uint8_t *)data + i; // write the chunk of data as specified
    }

    writeBytes(MPU6050_ADD, MPU6050_RA_MEM_R_W, chunkSize, progBuffer); //writeBytes(MPU6050_ADD, MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

    // verify data if needed
    if (verify && verifyBuffer) {
      setMemoryBank(bank, false, false);
      setMemoryStartAddress(address);
      readBytes(MPU6050_ADD, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer, I2CDEV_DEFAULT_READ_TIMEOUT);
      if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
        free(verifyBuffer);
        if (useProgMem)
          free(progBuffer);
        return false; // uh oh.
      }
    }
    i += chunkSize;     // increase byte index by [chunkSize]

    address += chunkSize; // uint8_t automatically wraps to 0 at 256

    // if we aren't done, update bank (if necessary) and address
    if (i < dataSize) {
      if (address == 0)
        bank++;

      setMemoryBank(bank, false, false);
      setMemoryStartAddress(address);
    }
  }
  if (verify)
    free(verifyBuffer);

  if (useProgMem)
      free(progBuffer);

  return true;
}

void setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank){
  bank &= 0x1F;
  if (userBank)
    bank |= 0x20;

  if (prefetchEnabled)
    bank |= 0x40;

  writeByte(MPU6050_ADD, MPU6050_RA_BANK_SEL, bank);
}

void setMemoryStartAddress(uint8_t address){
  writeByte(MPU6050_ADD, MPU6050_RA_MEM_START_ADDR, address);
}

bool writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem){
  uint8_t *progBuffer, success, special;
  uint16_t i, j;
  if (useProgMem) {
    progBuffer = (uint8_t *)malloc(8); // assume 8-byte blocks, realloc later if necessary
  }

  // config set data is a long string of blocks with the following structure:
  // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
  uint8_t bank, offset, length;
  for (i = 0; i < dataSize;){
    if (useProgMem){
      bank = pgm_read_byte(data + i++);
      offset = pgm_read_byte(data + i++);
      length = pgm_read_byte(data + i++);
    } else {
      bank = data[i++];
      offset = data[i++];
      length = data[i++];
    }

    // write data or perform special action
    if (length > 0) {
      if (useProgMem){
        if (sizeof(progBuffer) < length)
          progBuffer = (uint8_t *)realloc(progBuffer, length);

        for (j = 0; j < length; j++)
          progBuffer[j] = pgm_read_byte(data + i + j);
      } else {
        progBuffer = (uint8_t *)data + i;
      }
      success = writeMemoryBlock(progBuffer, length, bank, offset, true, false);
      i += length;
    } else {
      // special instruction
      // NOTE: this kind of behavior (what and when to do certain things)
      // is totally undocumented. This code is in here based on observed
      // behavior only, and exactly why (or even whether) it has to be here
      // is anybody's guess for now.
      if (useProgMem) {
        special = pgm_read_byte(data + i++);
      } else {
        special = data[i++];
      }

      if (special == 0x01) {
        // enable DMP-related interrupts
        //setIntZeroMotionEnabled(true);
        //setIntFIFOBufferOverflowEnabled(true);
        //setIntDMPEnabled(true);
        writeByte(MPU6050_ADD, MPU6050_RA_INT_ENABLE, 0x32);  // single operation

        success = true;
      } else {
        success = false;
      }
    }

    if (!success) {
      if (useProgMem) free(progBuffer);
      return false; // uh oh
    }
  }
  if (useProgMem)
    free(progBuffer);

  return true;
}

bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data){
  uint8_t status = 0;
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr); // send address

  for (uint8_t i = 0; i < length * 2; i++) {
    Wire.write((uint8_t)(data[i] >> 8));    // send MSB
    Wire.write((uint8_t)data[i++]);         // send LSB
  }
  
  status = Wire.endTransmission();
  return status == 0;
}

int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout){
  return readBytes(devAddr, regAddr, 1, data, timeout);
}

bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data){
  return writeBytes(devAddr, regAddr, 1, &data);
}

int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout){
  int8_t count = 0;
  uint32_t t1 = millis();

  for (uint8_t k = 0; k < length; k += min(length, BUFFER_LENGTH)){
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    Wire.endTransmission();
    Wire.beginTransmission(devAddr);
    Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));

    for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++)
      data[count] = Wire.read();
  }
  if (timeout > 0 && millis() - t1 >= timeout && count < length)
  count = -1; // timeout

  return count;
}

bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data){
  uint8_t status = 0;

  Wire.beginTransmission(devAddr);
  Wire.write((uint8_t) regAddr); // send address
  
  for (uint8_t i = 0; i < length; i++)
    Wire.write((uint8_t) data[i]);

  status = Wire.endTransmission();
  return status == 0;
}

int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout){
  uint8_t count, b;
  if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    *data = b;
  }
  return count;
}

bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data){
  uint8_t b;
  readByte(devAddr, regAddr, &b, I2CDEV_DEFAULT_READ_TIMEOUT);
  b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
  return writeByte(devAddr, regAddr, b);
}

bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data){
  uint8_t b;
  if (readByte(devAddr, regAddr, &b, I2CDEV_DEFAULT_READ_TIMEOUT) != 0){
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    return writeByte(devAddr, regAddr, b);
    }
  else {
    return false;
  }
}
