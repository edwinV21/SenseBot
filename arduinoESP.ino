
#include <ESP8266_XYZ.h>
#include <SoftwareSerial.h>
#include <dht.h>
//Configuracion sensor temperatura
dht DHT;
#define DHT11_PIN 5

//Configuracion sensor proximidad
long distancia; //pin 9 y 8
long tiempo;

//Configuracion Esp
#define SSID F("Starcraft")  //Nombre de la red a la que se desea conectar
#define PASS F("Qwerty102050")   //Contraseña de la red
#define server F("192.168.1.57")  //Servidor
#define PostPathG F("/api/gas")     //Ruta del Post
#define PostPathT F("/api/temp")     //Ruta del Post
#define PostPathI F("/api/intruder")     //Ruta del Post

#define ESP_TX   3           //Pin de TX del ESP
#define ESP_RX   2          //Pin de RX del ESP


//Configuracion sensor de gas
#define         MQ1                       (0)                   //define la entrada analogica para el sensor
#define         RL_VALOR             (5)                        //define el valor de la resistencia mde carga en kilo ohms
#define         RAL       (9.83)                                //resistencia del sensor en el aire limpio / RO, que se deriva de la                                             tabla de la hoja de datos
#define         GAS_LP                      (0)
String inputstring = "";                                        //Cadena recibida desde el PC
float           LPCurve[3]  =  {2.3,0.21,-0.47};
float           Ro           =  10;
int pin = A0;




const int motor1PinA = 9;
const int motor1PinB = 13;
const int motor2PinA = 11;
const int motor2PinB = 12;

const int enable1 = 8;
const int enable2 = 9;
const int delayTime = 1500;




SoftwareSerial swSerial(ESP_TX, ESP_RX); //Pines del ESP: TXD a D3, RXD a D2; (3,2)

ESP8266_XYZ esp(&swSerial, 4);    //Arg #1: puntero al stream, Arg #2: pin para reset
int counterUpdate = 0;

void setup() {
 Serial.begin(57600);             //Inicializacion del Monitor Serial a 9600
 swSerial.begin(57600);  
 //swSerial.println("AT+UART_DEF=9600,8,1,0,0");
   //swSerial.println("AT+CIOBAUD=57600");//Inicializacion de SoftwareSerial a 9600
  //delay(1000);
  pinMode(9, OUTPUT);               /*activación del pin 9 como salida: para el pulso ultrasónico*/
  pinMode(8, INPUT);  

  pinMode(6, OUTPUT);               /*activación del pin 9 como salida: para el pulso ultrasónico*/
  pinMode(7, INPUT);   
  //Calibracion Sensor gas
  Serial.println("Calibrando Sensor Gas");     //Si se muestra existe conexión
  Ro = Calibracion(MQ1);                                         //Calibrando el sensor.

   pinMode(motor1PinA, OUTPUT);
    pinMode(motor1PinB, OUTPUT);
    pinMode(motor2PinA, OUTPUT);
    pinMode(motor2PinB, OUTPUT);

    digitalWrite(enable1, HIGH);
    digitalWrite(enable2, HIGH);
    digitalWrite(motor1PinA, LOW);
    digitalWrite(motor1PinB, LOW);
    digitalWrite(motor2PinA, LOW);
    digitalWrite(motor2PinB, LOW);

  

  //Configuracion Wifi  
  Serial.println("Init");         //Mensaje de inicialización
  //No se continúa hasta asegurar el buen funcionamiento del dispositivo
  while (!esp.espTest());
  Serial.println("AT OK");     //Si se muestra existe conexión
  while (!esp.softReset());
  Serial.println("Continue");     //Si se muestra existe conexión
  while (!esp.connectAP(SSID, PASS));
  Serial.println("Setup OK");     //Si se muestra existe conexión
  esp.initServer();
  esp.setFlagSens(0);

  //Codigo motor
}

void loop() {
  

   // while(swSerial.available()){
     // esp.reviewGetRequest();
    //}
  esp.reviewGetRequest();
  checkMessage();
  //delay(3000);
  counterUpdate=checkPost(counterUpdate);
 // forward();
  
  counterUpdate++;


  Serial.println("Counter Update:"+String(counterUpdate));
 


  //delay(250);         //Suspende operaciones por 1 segundo
}

int checkPost(int counterUpdate){
  int flagSens=esp.getFlagData();
  swSerial.println("Flag Sens:"+String(flagSens));  
  String str_resp;
  if(   (counterUpdate>5) && (flagSens==1)   ){
    int tmp=readTemp();
    boolean dangerTemp = verifyTemp(tmp);
    boolean instr=readIntruder();
    int gas=porcentaje_gas(lecturaMQ(pin)/Ro,GAS_LP);
    Serial.print("LP:");
    boolean dangerGas=verifyGas(gas);
    Serial.println(gas );

  
    swSerial.println("AT+CIPMUX=0");
    delay(1000);
    /*while (swSerial.available() > 0) {
                char a = swSerial.read();
                Serial.write(a);
    }*/

    esp.addToJson("gasLevel", gas );
    esp.addToJson("danger", dangerGas );
    int resp = esp.httpPost(server, PostPathG, 8000,&str_resp);
   
    esp.addToJson("temperature", tmp );
    esp.addToJson("danger", dangerTemp );
    int resp2 = esp.httpPost(server, PostPathT, 8000,&str_resp);

    esp.addToJson("danger", instr );
    int resp3 = esp.httpPost(server, PostPathI, 8000,&str_resp);

    

    
    counterUpdate=0;
    
     while (!esp.softReset());
    Serial.println("Continue");     //Si se muestra existe conexión
     while (!esp.connectAP(SSID, PASS));
     esp.initServer();
  }
  return counterUpdate;
  


  
}

boolean verifyTemp(int pTemp){
  if(pTemp>40){
    return true;
  }
  else{
    return false;
  }
  
}

boolean verifyGas(int pGas){
  if(pGas>1500){
    return true;
  }
  else{
    return false;
  }
  
}


void checkMessage(){
  int dirA=esp.getDirA();
  int dirB=esp.getDirB();
  int dirI=esp.getDirI();
  int dirD=esp.getDirD();
  if(dirA==1){
    forward();
          Serial.println("Doing forward");
  }
  else if(dirB==1){
          Serial.println("Doing backward");
    backward();
  }
  else if(dirI==1){
   Serial.println("Doing left");

    left();
  }
  else if(dirD==1){
       Serial.println("Doing right");

    right();
  }
  else{
    halt();
    }
    esp.resetDir();
  
}


//Sensor de temperatura
int readTemp(){
   Serial.print("DHT11, \t");
  int chk = DHT.read11(DHT11_PIN);
  Serial.println(DHT.temperature, 1);
  return DHT.temperature;
  
}

//Sensor proximidad

boolean readIntruder(){
   digitalWrite(6,LOW);              /* Por cuestión de estabilización del sensor*/
    delayMicroseconds(5);
    digitalWrite(6, HIGH);            /* envío del pulso ultrasónico*/
    delayMicroseconds(10);
    tiempo=pulseIn(7, HIGH);          /* Función para medir la longitud del pulso entrante. Mide el tiempo que transcurrido entre el envío
                                       del pulso ultrasónico y cuando el sensor recibe el rebote, es decir: desde que el pin 12 empieza a 
                                       recibir el rebote, HIGH, hasta que deja de hacerlo, LOW, la longitud del pulso entrante*/
    
    distancia= int(0.017*tiempo);     /*fórmula para calcular la distancia obteniendo un valor entero*/
    
    /*Monitorización en centímetros por el monitor serial*/
    Serial.println("Distancia ");
    Serial.println(distancia);
    Serial.println(" cm");
    if(distancia>=300){
      return true;
    }
    return false;
}


//Configuracion Sensor de gas


float calc_res(int raw_adc)
{
  return ( ((float)RL_VALOR*(1023-raw_adc)/raw_adc));
}

float Calibracion(float mq_pin){
  int i;
  float val=0;
    for (i=0;i<50;i++) {                                         //tomar múltiples muestras
      val += calc_res(analogRead(mq_pin));
      delay(500);
    }
  val = val/50;                                                  //calcular el valor medio
  val = val/RAL;
  return val;
}

float lecturaMQ(int mq_pin){
  int i;
  float rs=0;
  for (i=0;i<5;i++) {
    rs += calc_res(analogRead(mq_pin));
    delay(50);
  }
rs = rs/5;
return rs;
}

int porcentaje_gas(float rs_ro_ratio, int gas_id){
   if ( gas_id == GAS_LP ) {
     return porcentaje_gas(rs_ro_ratio,LPCurve);
   }
  return 0;
}

int porcentaje_gas(float rs_ro_ratio, float *pcurve){
  return (pow(10, (((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}


void forward() {
    digitalWrite(motor1PinA, LOW);
    digitalWrite(motor1PinB, HIGH);
    digitalWrite(motor2PinA, LOW);
    digitalWrite(motor2PinB, HIGH);
    delay(delayTime);
    halt();
}

void backward() {
    digitalWrite(motor1PinA, HIGH);
    digitalWrite(motor1PinB, LOW);
    digitalWrite(motor2PinA, HIGH);
    digitalWrite(motor2PinB, LOW);
    delay(delayTime);
    halt();
}

void left() {
    digitalWrite(motor1PinA, LOW);
    digitalWrite(motor1PinB, HIGH);
    digitalWrite(motor2PinA, LOW);
    digitalWrite(motor2PinB, LOW);
    delay(delayTime);
    halt();
}

void right() {
    digitalWrite(motor1PinA, LOW);
    digitalWrite(motor1PinB, LOW);
    digitalWrite(motor2PinA, LOW);
    digitalWrite(motor2PinB, HIGH);
    delay(delayTime);
    halt();
}

void halt() {
    digitalWrite(motor1PinA, LOW);
    digitalWrite(motor1PinB, LOW);
    digitalWrite(motor2PinA, LOW);
    digitalWrite(motor2PinB, LOW);
}
