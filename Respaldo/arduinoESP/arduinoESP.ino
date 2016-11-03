
#include <ESP8266_XYZ.h>
#include <SoftwareSerial.h>

//Pin out


#define SSID F("Marielos Angeles L.A")  //Nombre de la red a la que se desea conectar
#define PASS F("Edwin1986")   //Contraseña de la red
#define server F("192.168.100.2")  //Servidor
#define PostPathG F("/api/gas")     //Ruta del Post
#define PostPathT F("/api/temp")     //Ruta del Post
//#define PostPathI F("/api/intruder")     //Ruta del Post
#define PostPathI F("/prueba2")     //Ruta del Post



#define ESP_TX   11           //Pin de TX del ESP
#define ESP_RX   10          //Pin de RX del ESP


/*

void setup()
{
    Serial.begin(9600);
    Serial1.begin(9600);
}
void loop()
{
    while (Serial1.available()) {
        Serial.write(Serial1.read());
    }
    while (Serial.available()) {
        Serial1.write(Serial.read());
    }
}*/

SoftwareSerial swSerial(ESP_TX, ESP_RX); //Pines del ESP: TXD a D3, RXD a D2; (3,2)

ESP8266_XYZ esp(&swSerial, 4);    //Arg #1: puntero al stream, Arg #2: pin para reset
int counterUpdate = 0;


void setup() {
  Serial.begin(115200);             //Inicializacion del Monitor Serial a 9600
  swSerial.begin(57600);  
  swSerial.println("AT+CIOBAUD=57600");//Inicializacion de SoftwareSerial a 9600
  Serial.println("Init");         //Mensaje de inicialización
  //No se continúa hasta asegurar el buen funcionamiento del dispositivo
  while (!esp.espTest());
    Serial.println("AT OK");     //Si se muestra existe conexión
 // while (!esp.softReset());
  Serial.println("Continue");     //Si se muestra existe conexión
  while (!esp.connectAP(SSID, PASS));
  Serial.println("Setup OK");     //Si se muestra existe conexión
      esp.initServer();
      esp.setFlagSens(1);
      
}

void loop() {
  
  int flagSens=esp.getFlagData();
  esp.reviewGetRequest();
  String str_resp;
 
  //Se envía la solicitud POST al servidor
  if(   (counterUpdate>2000) && (flagSens==1)   ){
    swSerial.println("AT+CIPMUX=0");
    delay(1000);
    while (swSerial.available() > 0) {
                char a = swSerial.read();
                Serial.write(a);
            }
    /*esp.addToJson("gasLevel", 123 );
    esp.addToJson("danger", true );
    int resp = esp.httpPost(server, PostPathG, 8000,&str_resp);
   
    esp.addToJson("temperature", 123 );
    esp.addToJson("danger", true );
    int resp2 = esp.httpPost(server, PostPathT, 8000,&str_resp);

    esp.addToJson("instruder", "" );
    int resp3 = esp.httpPost(server, PostPathI, 8000,&str_resp);*/


     esp.addToJson("hashtag", "30X" );
    int resp3 = esp.httpPost(server, PostPathI, 8081,&str_resp);

    
    counterUpdate=0;
    
    esp.initServer();
    //swSerial.println("AT+CIPMUX=1");//Modo 3 de operación: SoftAP + Station

     //esp.setFlagSens(1);
  }
  
  
  counterUpdate++;

  
  //Se imprime el código de respuesta del servidor
  Serial.println("Counter Update:"+String(counterUpdate));
 // Serial.println(str_resp);




  //Se crea un objeto String cuyo puntero será pasado como argumento

  //delay(500);         //Suspende operaciones por 1 segundo
}


