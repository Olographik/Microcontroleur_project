#include <WiFi.h>
#include <WiFiUdp.h>

//pin map
const int reseau = 25, triac = 26; //pin reseau
const int ultrason_triger_front = 23, ultrason_triger_back = 22; //pin sensor ultrason
const int ultrason_ear_front= 19,ultrason_ear_back = 18; //pin sensor ultrason
const int end_stop = 21;//pin end stop
const int Launch_coffe_btn = 5;// btn pour lancer la machine a cafe

// WiFi network name and password:
const char * networkName = "Olographik-Rasp";
const char * networkPswd = "Olographikpasword";

//char for code
const char* SENDINGCODE = "Qui est tu ?";
const char* RECEIVECODE = "Je suis Olographik";
const char* START_COFFE = "Coffe";
const char* VALUE_LAMP = "lamp";
const char* CHANGE_TASSE_VALUE="Coffee new value";
const char* COFFE_DONE = "Cafe_done";
const char* RESERVOIR_EMPTY = "WATER_EMPTY";
const char* COFFE_VALUE = "COFFEEVALUE:";
const char* HASNOCAPSULE = "NOCAPSULE";
const char* RSSI_LOW = "RSSILOW";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "255.255.255.255";
const int udpPort = 2224;

//Are we currently connected?
bool connected = false, Identified = false;
int bcle = 0, waittime = 30000;

//The udp library class
WiFiUDP udp;

// Common data
int prct_value_lamp = 0;
int value_of_coffe = 100;
int volume_of_coffe = 0;
bool value_lamp_changed=false, Coffe_start = false, volume_of_cofe = false, coffe_done = false, capsule = true;

//Multi
TaskHandle_t Cofe, Lamp, flag_survey;
SemaphoreHandle_t lamp_flag, coffe_flag, volume_cofe_flag, capsule_flag;

void setup(){
  // Initilize hardware serial:
  Serial.begin(115200);
  
  volume_cofe_flag = xSemaphoreCreateMutex();
  lamp_flag = xSemaphoreCreateMutex();
  coffe_flag= xSemaphoreCreateMutex();
  capsule_flag = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    GestionCafe,
    "CoffeTask",
    1000,
    NULL,
    1,
    &Cofe,
    0);

  delay(500); 

  xTaskCreatePinnedToCore(
    GestionLampe,
    "lampTask",
    1000,
    NULL,
    1,
    &Lamp,
    0);

  delay(500); 

  xTaskCreatePinnedToCore(
    SurveyflagtosendInfo,
    "lampTask",
    1000,
    NULL,
    1,
    &flag_survey,
    1);

  delay(500); 
  
  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
}

void loop(){
  if(connected){
    if (Identified){
      //Serial.println("we can now send command");
      Decode_command();
    }
    else 
      IdentificationAPP();
  }
  delay(1);
}

#pragma region task
void GestionCafe(void * parameter)
{
  pinMode(ultrason_triger_front, OUTPUT);
  pinMode(ultrason_triger_back, OUTPUT);
  pinMode(ultrason_ear_front, INPUT);
  pinMode(ultrason_ear_back, INPUT);
  pinMode(end_stop, INPUT);
  pinMode(Launch_coffe_btn, OUTPUT);

  bool has_capsule = false, is_water = false;

  for(;;)
  {
    if(TrigerUltrason(ultrason_triger_back, ultrason_ear_back) < 1000)// see if there is enough water
      is_water = true;
    if (digitalRead(end_stop) && !has_capsule)
    {
      has_capsule = true;
    }
    xSemaphoreTake(coffe_flag, portMAX_DELAY);
    bool flag = Coffe_start;
    long max_volume = value_of_coffe; // in %
    if (max_volume == 0)
      max_volume = 1000;// volume par defaut
    //transformation % in duration.
    if (flag)
    {
      if(has_capsule){
        Coffe_start = false;
        long volume = TrigerUltrason(ultrason_triger_front, ultrason_ear_front); 
        if(volume < 1000)//see if there is a tasse
        {
          if(is_water)
          {
            digitalWrite(Launch_coffe_btn, HIGH); // triger the button 
            delay(1);
            digitalWrite(Launch_coffe_btn, LOW);
            delay(5000); 
            long new_volume = TrigerUltrason(ultrason_triger_front, ultrason_ear_front);
            if(new_volume == volume) //si le cafe n'a pas commence a couler c'est que la machine à besoin d'une selection de tasse
            {
              digitalWrite(Launch_coffe_btn, HIGH); // triger the button 
              delay(1);
              digitalWrite(Launch_coffe_btn, LOW);
            }
            int count = 0;
            while (new_volume < max_volume)
            {
              new_volume = TrigerUltrason(ultrason_triger_front, ultrason_ear_front);
              if (volume == new_volume)// si ca ne coule plus
                count++;
              else
                count = 0;
              if (count >= 8) // on attend deux secondes pour etre sur et on relance la machine 
              {
                digitalWrite(Launch_coffe_btn, HIGH); // triger the button 
                delay(1);
                digitalWrite(Launch_coffe_btn, LOW);
              }
              volume = new_volume;
              xSemaphoreTake(volume_cofe_flag, portMAX_DELAY);
              volume_of_cofe = true;
              volume_of_coffe = new_volume*100/max_volume;
              xSemaphoreGive(volume_cofe_flag);
              delay(250);
            }
            new_volume = TrigerUltrason(ultrason_triger_front, ultrason_ear_front);
            delay(100);
            volume = TrigerUltrason(ultrason_triger_front, ultrason_ear_front);
            if (volume > new_volume)// on verifie que ca c'est arreter de couler
            {
              digitalWrite(Launch_coffe_btn, HIGH); // triger the button 
              delay(1);
              digitalWrite(Launch_coffe_btn, LOW);
            }
            has_capsule = false;
            coffe_done = true;
          }
        }
      }
      else
      {
        xSemaphoreTake(capsule_flag, portMAX_DELAY);
        capsule = false;
        xSemaphoreGive(capsule_flag);
      }
    }
    xSemaphoreGive(coffe_flag);// tant que le cafe coule on accepte pas de changement dans les valeurs
    delay(200);
  }
}

void GestionLampe(void * parameter)
{
  long time_low=0;
  pinMode(reseau, INPUT);
  pinMode(triac, OUTPUT);
  //Surveiller le Reseau pour obtenir le passage par 0.
  for(int i= 0; i<5; i++){
    time_low += pulseIn(reseau, LOW);
  }
  time_low /= 5; // get an average of the time when reseau is low
  int time_to_wait_to_zero = time_low/2; // get the average real 0 in µseconds ;
  time_low = NULL;

  int wait_time = 20; //in ms

  for(;;) 
  {    
    xSemaphoreTake(lamp_flag, portMAX_DELAY);
    bool flag = value_lamp_changed;
    if(flag)
    {
      value_lamp_changed = false;
      wait_time = prct_value_lamp;
      Serial.print("Core : ");
      Serial.print(xPortGetCoreID());
      Serial.print("Gestion lampe : pourcentage for lamp : ");
      Serial.println(wait_time);
      wait_time = ((100-wait_time)*20)/100; //transforamtion % en millis
    }
    xSemaphoreGive(lamp_flag);

    while(digitalRead(reseau)); // pour se synchroniser avec le reseau;
    delayMicroseconds(time_to_wait_to_zero);
    delay(wait_time);
    //start train d'impulsion (a changer lorsque datasheet etudiée)
    digitalWrite(triac, HIGH);
    delayMicroseconds(500);
    digitalWrite(triac, LOW);
  }
}

void SurveyflagtosendInfo(void * parameter)
{
  WiFiUDP udp_sender = udp
  ;
  for(;;)
  {
    xSemaphoreTake(coffe_flag, portMAX_DELAY);
    bool flag = coffe_done;
    if(flag)
    {
      coffe_done = false;
      udp_sender.beginPacket(udpAddress, 2225);
      udp_sender.printf(COFFE_DONE);
      udp_sender.endPacket();
    }
    xSemaphoreGive(coffe_flag);
    
    xSemaphoreTake(volume_cofe_flag, portMAX_DELAY);
    flag = volume_of_cofe;
    if(flag)
    {
      volume_of_cofe = false;
      uint8_t value = volume_of_coffe;
      udp_sender.beginPacket(udpAddress, 2225);
      udp_sender.printf(COFFE_VALUE);
      if (value != 57)
        udp.write(value);
      udp_sender.endPacket();
    }
    xSemaphoreGive(volume_cofe_flag);

    xSemaphoreTake(capsule_flag, portMAX_DELAY);
    flag = !capsule;
    if(flag)
    {
      capsule = true;
      udp_sender.beginPacket(udpAddress,2225);
      udp_sender.printf(HASNOCAPSULE);
      udp.endPacket();
    }
    xSemaphoreGive(capsule_flag);

    delay(300);
  }
}
#pragma endregion

void Decode_command(){

  char buff[32];
  int val;
  while(udp.parsePacket()==0);

  val = udp.read(buff,32);
  udp.flush();

  if(memcmp(buff, START_COFFE, sizeof(START_COFFE)) == 0 && val == 5){
    Serial.println("Demarrer la machine à café");
    xSemaphoreTake(coffe_flag,portMAX_DELAY);
    Coffe_start = true;
    xSemaphoreGive(coffe_flag);
  }
  else if (memcmp(buff, VALUE_LAMP, sizeof(VALUE_LAMP))==0)
  {
    int newvalue = 0;
    for(int i =sizeof(VALUE_LAMP) ; i<val; i++)
    {
      newvalue+= pow(10, val-i-1) *(buff[i] - '0');
    }
    Serial.print("La valeur de la lampe va changer et devenir : ");
    Serial.println( newvalue);
    xSemaphoreTake(lamp_flag, portMAX_DELAY);
    prct_value_lamp = 100*newvalue/255;
    value_lamp_changed = true;
    xSemaphoreGive(lamp_flag);

  }
  else if(memcmp(buff, CHANGE_TASSE_VALUE, 16) == 0)
  {    
    int newvalue = 0;
    for(int i =16 ; i<val; i++)
    {
      newvalue+= pow(10, val-i-1) *(buff[i] - '0');
    }
    Serial.print("La valeur de la tasse va changer et devenir : ");
    Serial.println(newvalue);
    xSemaphoreTake(coffe_flag, portMAX_DELAY);
    value_of_coffe = newvalue;
    xSemaphoreGive(coffe_flag);
  }
  else if(memcmp(buff, RSSI_LOW,7)==0)
  {
    //Va t'il partir ou rester ? telle est la question...
    Identified = false;
    waittime = 1000;
    IdentificationAPP();
  }
  
}

long TrigerUltrason(int trigPin, int echoPin){
  //triger the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  return pulseIn(echoPin, HIGH);
}

int IdentificationAPP(){
  //Send a packet
  Serial.println("Sending packet");
  udp.beginPacket(udpAddress,2225);
  udp.printf(SENDINGCODE);
  udp.endPacket();
  int count = 0 ;
  //to wait for an answer;
  while(count <= 100){
    int pack =udp.parsePacket() ;
    if(pack>0)
    {
      char buff[32];
      int value = udp.read(buff, 32);
    
      if(memcmp(buff, RECEIVECODE, value)==0){
        Serial.println("Identified");
        Identified = true;
        return 1;
      }
      udp.flush();
      break;
    }
    count ++;
    delay(1);
  }
  delay(waittime-100);
  bcle++;
  if (bcle  == 60)
    waittime = 30000;// si ca fait une minute qu'on a recu le message RSSILOW 
  return 0 ;
}

#pragma region ConnectionWIFI
void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
    }
}
#pragma endregion
