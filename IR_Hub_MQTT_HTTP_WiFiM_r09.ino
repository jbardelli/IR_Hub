#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <IRtimer.h>
#include <ESP8266WiFi.h>;
#include <PersWiFiManager.h>                          // WifiManager by Ryan Downing
#include <SPIFFSReadServer.h>                         // Extension of ESP8266WebServer with SPIFFS handlers built in, by Ryan Downing
#include <PubSubClient.h>                             // MQTT Library by Nick O'Leary
#include <DNSServer.h>
#include <FS.h>
#include <ArduinoJson.h>                              // Json library by Benoît Blanchon
//----------------- PROGRAM DEFINES ----------------------
#define HOSTNAME  "IR_Hub"              
#define AP_PASS "irhubadmin"                          // Password to connect to the device network  when it is in Access Point mode 
#define MAX_ARGUMENTS 10                              // Max number of arguments accepted in the command received from the mqtt server
#define NUM_HEX_CODES 22                              // Max number of HEX codes that can be configured for each device
#define NUM_ACCEPTED_REPEATS 8                        // Max number of accepted repeats for the volume command
#define MODE_LED D3                                   // Mode led pin, indicates whether wifi is connected, disconnected or system is in AP mode
#define IR_LED D8                                     // IR led pin
#define IR_SENSOR D2                                  // IR sensor pin
#define DELAY_LED_AP 1000                             // Delay in milliseconds for flashing led in AP mode (mode used for configuration or wifi setup)
#define IR_DELAY 100                                  // Delay in milliseconds between irsend (found by trial and error)
#define REMOTE_BTN_TIMEOUT 10000                      // Maximum time to wait for a button press when recording codes from a remote control
#define LED_STRIP_R D7                                // RED pin for the 5050 LED STRIP
#define LED_STRIP_G D6                                // GREEN pin for the 5050 LED STRIP
#define LED_STRIP_B D5                                // BLUE pin for the 5050 LED STRIP

SPIFFSReadServer server(80);                                                                  // SPIFFS Web server declaration on http port 80
DNSServer dnsServer;                                                                          // DNSServer declaration
PersWiFiManager persWM(server, dnsServer);                                                    // PersWiFiManager class declaration 
WiFiClient wifi_client;                                                                       // Declare WiFi client for the MQTT client
PubSubClient mqtt_client(wifi_client);                                                        // Declare MQTT client for PubSubClient library
IRsend irsend(IR_LED);                                                                        // Digital pin for the IR Leds
IRrecv irrecv(IR_SENSOR);                                                                     // Declare irrecv variable with sensor pin

//--------------------- PROGRAM VARIABLES ----------------------
String wifiMode, arguments[MAX_ARGUMENTS], led_mode;
unsigned long lastmillis_led;
int repeat, code, channel_number, led_R, led_G, led_B, led_intensity, sleep_status;

typedef struct {                                                                // Struct that includes all the mqtt connection data
  String mqtt_server;
  int mqtt_port;
  String mqtt_user;
  String mqtt_pass;
  String mqtt_feed;
}mqtt_server;

mqtt_server mqtt_config;                                                        // Declare the variable to store mqtt server configuration

typedef struct {                                                                // Struct that includes all the device configuration data
  String device_name;
  String ir_protocol;
  String commands[NUM_HEX_CODES];
  int hex_codes1[NUM_HEX_CODES];
  int hex_codes2[NUM_HEX_CODES];
  int hex_numbers1[10];
  int hex_numbers2[10];
}device;

device device1 { "", "", {}, {}, {}, {},{} };                                   // Declare device configuration structure for each device
device device2 { "", "", {}, {}, {}, {},{} };                                   // data will be loaded from the json config file stored in SPIFFS
device device3 { "", "", {}, {}, {}, {},{} };                                   // during setup()
device device4 { "", "", {}, {}, {}, {},{} };
device device5 { "", "", {}, {}, {}, {},{} };
device device6 { "", "", {}, {}, {}, {},{} };
String  accepted_numbers [NUM_ACCEPTED_REPEATS]={"five","ten","twenty","thirty","5","10","20","30"};  // List of accepted numbers for the repeated functions (volume)
int     accepted_repeats [NUM_ACCEPTED_REPEATS]={     5,   10,      20,      30,  5,  10,  20,  30};  

typedef struct{                                                                 // Struct that includes all the procedure configuration data
  String  keyword;
  String  device[MAX_ARGUMENTS];
  String  command[MAX_ARGUMENTS];
  int     repeats[MAX_ARGUMENTS];
  int     delay[MAX_ARGUMENTS];
}procedure;

procedure procedure1 {"", {}, {}, {},{}};                                       // Declare 3 supported procedures, which is a sequence of commands to send IR commands to various
procedure procedure2 {"", {}, {}, {},{}};                                       // devices one after the other, this could serve as a startup procedure for an entertainment 
procedure procedure3 {"", {}, {}, {},{}};                                       // system in which you need to turn tv, amplifier, and dvd, and set input, volumens, etc.
//------------------ SETUP ------------------------------------
void setup() {  
 pinMode(MODE_LED, OUTPUT);                                                     // Set led pin to OUTPUT
 Serial.begin(115200);
 Serial.println("\nSetup initiated");
 //system_update_cpu_freq(SYS_CPU_160MHZ);                                      // Change ESP clock frequency to 160 Mhz
 persWM.onConnect([]() {                                                        // Code handlers to run everytime wifi is connected
    Serial.println("MODE STA - WiFi connected - IP: " + WiFi.localIP().toString());
    wifiMode = "STA";
    digitalWrite(MODE_LED, HIGH);                                                 // Set led fixed ON once it is connected to WiFi network   
    mqtt_connect();                     
  });
 persWM.onAp([](){                                                                // Code handler to run everytime AP mode is started
    Serial.println("MODE AP - AP started - SSID: " + persWM.getApSsid());
    wifiMode = "AP";
  });
 
 SPIFFS.begin();                                                                  // Allows serving of files from SPIFFS
 persWM.setApCredentials(HOSTNAME, AP_PASS);                                      // Sets network name for AP mode and password
 persWM.setConnectNonBlock(true);                                                 // Make connecting/disconnecting to wifi non-blocking
 WiFi.hostname(HOSTNAME);                                                         // Set hostname so the DHCP knows the name of the device
 persWM.begin();
 Serial.println("MAC Address: " + WiFi.macAddress());                             // Print MAC on Serial Port                                        
 irsend.begin();                                                                  // Start IRSEND 
 irrecv.enableIRIn();                                                             // Start the receiver
 server.begin();                                                                  // Start SPIFFS Webserver
 server.on("/mqtt", process_mqtt_form);                                           // Handler used when mqtt setup page submits the config form
 server.on("/dev",  process_dev_form);                                            // Handler used when device setup page submits the config form
 server.on("/proc", process_proc_form);                                           // Handler used when procedure setup page submits the config form
 server.on("/led",  process_led_form);                                            // Handler used when led setup page submits the led config form
 server.on("/req_mqtt", req_mqtt);                                                // Handler used when mqtt setup page loads and gets configuration parameters from server
 server.on("/req_dev",  req_dev);                                                 // Handler used when device setup page loads and gets configuration parameters from server
 server.on("/req_proc", req_proc);                                                // Handler used when procedure setup page loads and gets configuration parameters from server
 server.on("/req_led", req_led);                                                  // Handler used when led setup page loads and gets configuration parameters from server
 server.on("/comm", http_msg_recv);                                               // Handler used to support commands via http instead of mqtt
 server.on("/rec_ir", receive_ir);                                                // Handler used to record (learn) ir codes from a remote control
 server.begin();                                                                  // Initialize the SPIFFS webserver
 load_mqtt();                                                                     // Load mqtt setup from the json file stored in flash memory
 load_led();                                                                      // Load led setup from the json file stored in flash memory
 load_dev(&device1, "1");                                                         // Load device1 configuration parameters and IR codes
 load_dev(&device2, "2");                                                         // Load device2 configuration parameters and IR codes
 load_dev(&device3, "3");                                                         // Load device3 configuration parameters and IR codes
 load_dev(&device4, "4");                                                         // Load device4 configuration parameters and IR codes
 load_dev(&device5, "5");                                                         // Load device5 configuration parameters and IR codes
 load_dev(&device6, "6");                                                         // Load device6 configuration parameters and IR codes
 load_proc(&procedure1, "1");                                                     // Load procedure1 configuration parameters
 load_proc(&procedure2, "2");                                                     // Load procedure2 configuration parameters
 load_proc(&procedure3, "3");                                                     // Load procedure3 configuration parameters
 device1.commands[0]= "volume+";                                                  // Initialize command names volume+ and volume- as fixed in the device struct
 device1.commands[1]= "volume-";
 device2.commands[0]= "volume+";
 device2.commands[1]= "volume-";
 device3.commands[0]= "volume+";
 device3.commands[1]= "volume-";
 device4.commands[0]= "volume+";
 device4.commands[1]= "volume-";
 device5.commands[0]= "volume+";
 device5.commands[1]= "volume-";
 device6.commands[0]= "volume+";
 device6.commands[1]= "volume-";
 Serial.println("Setup complete");
}
//--------------- LOOP ------------------------------
void loop() {
  persWM.handleWiFi();                                                            // In non-blocking mode, handleWiFi must be called in the main loop
  dnsServer.processNextRequest();                                                 // Handle DNS server requests
  server.handleClient();                                                          // Handle webserver client

  if ( wifiMode == "STA" ) mqtt_client.loop();                                    // If in STA mode handle the mqtt broker
  if ( wifiMode == "STA" && !mqtt_client.connected() && mqtt_config.mqtt_server !="" ) mqtt_connect(); // If in STA mode and disconnected and mqqt configured,
                                                                                  //then check mqtt broker connection and reconnect and subscribe if neccesary                                                                              
  if( (millis()-lastmillis_led) > DELAY_LED_AP ){                                 // Flash led slow if in AP mode      
   lastmillis_led = millis();
   if (wifiMode == "AP")digitalWrite(MODE_LED, !digitalRead(MODE_LED)); 
  }
}// loop() END 
//--------------- parse_string() ------------------------------------
void parse_string(String command)                                                 // Parses the whole command string with SPACE delimiters
{int char_ini, char_fin, command_len, i;                                          // each string found is saved in the arguments[] array, if there are no more commands 
 command_len=command.length();                                                    // complete the rest with "NONE"
 char_ini=0;
 char_fin=0;

 for ( i=0 ; i<MAX_ARGUMENTS; i++){                       
  if (command.indexOf(' ',char_ini)!= -1){                                        // Look for SPACES (delimiter)  
  char_fin=command.indexOf(' ',char_ini);                                         // get the ending character of the command or argument 
  arguments[i]=command.substring(char_ini,char_fin);                              // Save the argument in the array
  char_ini=char_fin+1;                                                            // Set the initial character to parse the next command or argument
  }
 else{                                                                            // If there are no more SPACES, the its the last command or argument 
      if(char_ini < command_len){                                                 // Check if there are more arguments or it is the last
        arguments[i]=command.substring(char_ini,command_len);
        char_fin=command_len;                                                     // If it was the last argument set char_fin to string length so next time it nows ther is no more
        char_ini=char_fin;
      }
      else arguments[i]="NONE";
      }
 }
 for ( i=0 ; i<MAX_ARGUMENTS; i++) Serial.println("Argument" + String(i+1) + ": " + arguments[i]);  // Print all the parsed arguments to the serial port
 return;
}
//-------------- search_ir_command() ------------------------------------------ 
 int search_ir_command ()                                                         // Send every command in the command array to send_command which will validate and send if applicable
 {int i;                                        
  code = -1;
  device *dev = NULL;
  if ( arguments[0] == procedure1.keyword ) exec_procedure(&procedure1);          // Check a procedure was invoked and execute the corresponding procedure
  if ( arguments[0] == procedure2.keyword ) exec_procedure(&procedure2); 
  if ( arguments[0] == procedure3.keyword ) exec_procedure(&procedure3);
  if ( arguments[0] == procedure1.keyword || arguments[0] == procedure2.keyword || arguments[0] == procedure3.keyword ) return code = 1; // If a procedure was executed ignore the rest of the commands and return                    
  if ( arguments[0] == device1.device_name ) dev = &device1;                      // Select the global variable for the correct device
  if ( arguments[0] == device2.device_name ) dev = &device2;
  if ( arguments[0] == device3.device_name ) dev = &device3;
  if ( arguments[0] == device4.device_name ) dev = &device1;
  if ( arguments[0] == device5.device_name ) dev = &device2;
  if ( arguments[0] == device6.device_name ) dev = &device3;
  if ( dev == NULL ) return -1;                                                   // If the device on the command was not found then return -1 as the command is invalid
  for ( i = 1; i < MAX_ARGUMENTS; i++){                                           // Go through all arguments and send them to send_command which will validate and send
    repeat = 1;                                                                   // Set repetitions to 1 by default
    i = send_command(i, dev);                                                     // send command and update for index in case there are commands to ignore                                     
  }
  return code;                   
 }
//--------------------------- process_volume ---------------------------------------
int process_volume (int k)                                                        // Volume commands need to be treated different because it could be up or down and have repetitions
{ int j;
  if (arguments[k+1]=="up")   arguments[k]="volume+";                             //  and look for up or down, if there is up or down...
  if (arguments[k+1]=="down") arguments[k]="volume-";                             //  modify the command to match the available commands array
  if (arguments[k+1]=="up" || arguments[k+1]=="down"){                            // If it's up or down
    for( j = 0; j < NUM_ACCEPTED_REPEATS; j++){                                   // go through the accepted repeats
      if(arguments[k+2] == accepted_numbers[j]){                                  // check if the next argument after up or down is one of the accepted repeats
        repeat = accepted_repeats[j];                                             // if it finds a valid repeat increment the index so the UP/DOWN , and REPEAT arguments are ignored
        k = k + 2;
        return k;                                                                 
      }  
    }                                                                             // if it is assign the int number corresponden to the accepted repeatup
  k = k + 1;
  }                                                                               // if there is no up or down the volume command stays the same and won't be recognized 
  return k;                                                                       // Return the updated index to avoid already processed UP/DOWN or repeat commands
}
//--------------------------- process_channel ---------------------------------------
int process_channel (int k, device *dev)                                          // Channel commands need to be treated different because it could be up or down and have repetitions
{                                                                                 // It supports channel up, channel down and channel XX formats
 int i,g;
 channel_number = 0;
 if (arguments[k]=="channel"){                                                    // Check if it's channel 
    for( i = 0; i < sizeof(arguments[k+1]); i++){  
      if (isDigit(arguments[k+1].charAt(i))) {                                    // If the next argument after "channel" is a number, assume it's the channel number
        g = arguments[k+1].charAt(i) - '0';                                       // Convert the char to integer
        send_ir (dev->ir_protocol, dev->hex_numbers1[g]);                         // Send digit
        send_ir (dev->ir_protocol, dev->hex_numbers2[g]);
        code = 1;                                                                 // Flag as valid command
        channel_number = arguments[k+1].toInt();                                  // Flag to indicate to send_command() that there is a channel number
      }
      else{
        if (arguments[k+1]=="up")   arguments[k] = "channel+";                    // If there is no channel number look for UP or DOWN and modify the command name
        if (arguments[k+1]=="down") arguments[k] = "channel-";
      }
    }
  }
return k + 1; 
}
//--------------------------- send_command() ---------------------------------
int send_command (int k, device *dev) {                                           // Looks for the device in the arguments array arguments [0] 
  int i, j, nextk = k;
  if ( arguments[k] == "volume" ) nextk = process_volume(k);                      // If command is volume process it and update argument array index to avoid procesing commands related to volume
  if ( arguments[k] == "channel") {                                               // Channel also needs a special treatment
    nextk = process_channel(k, dev);                                              // Process_channel sends the channel IR digits so we can ignore the rest of the function and return
    if ( channel_number != 0 ) return nextk;                                      // If there is a channel number in the command (not UP or DOWN) then the numbers were already sent bu process_channel()...   
  }                                                                               // so return next, if thers is an UP or DOWN in the command then continue and send the corresponding HEX code in the next for loop
  for ( i = 0; i < NUM_HEX_CODES ; i++ ){                                         // Go through all the possible commands for the device, to see if the argument command is valid        
    if ( arguments[k] ==  dev->commands[i]){                                      // If it finds the command in the device, send the HEX code
      for( j = 0; j < repeat ; j++){                                              // from HEX codes array
        if( dev->hex_codes1[i] != 0 )send_ir (dev->ir_protocol, dev->hex_codes1[i]); // If the hex code is not zero send the ir code passing the protocol and the integer for the code
        if( dev->hex_codes2[i] != 0 )send_ir (dev->ir_protocol, dev->hex_codes2[i]); 
        delay(IR_DELAY);   
      }
      code = 1;                                                                   // Flag that at least one command is correct
    }        
  }
return nextk;
}
//-------------------------- process_mqtt_form() -----------------------
 void process_mqtt_form (){                                                       // Receives the mqtt configuration from the webpage, stores it into the global variables
    Serial.println("server.on /mqtt");                                            // and saves the configuration to the json file.
    if (server.hasArg("mqtt_server")) {
      mqtt_config.mqtt_server = server.arg("mqtt_server");
      mqtt_config.mqtt_port = server.arg("mqtt_port").toInt();
      mqtt_config.mqtt_user = server.arg("mqtt_user");
      mqtt_config.mqtt_pass = server.arg("mqtt_pass");
      mqtt_config.mqtt_feed = server.arg("mqtt_feed");
      Serial.println("MQTT Server: " + mqtt_config.mqtt_server + " / MQTT Port: " + String(mqtt_config.mqtt_port) + " / MQTT User: " + mqtt_config.mqtt_user + " / MQTT Password: " + mqtt_config.mqtt_pass + " / MQTT Feed: " + mqtt_config.mqtt_feed );
      save_mqtt();
    }
    server.send(200, "text/html", "OK\r\n");
  } //process_mqtt_form
//-------------------------- req_mqtt() --------------------------------
 void req_mqtt (){                                                              // Get mqtt configuration from global variables and send data back to the client webpage
  Serial.println("server.on /req_mqtt");                                        // Used when the mqtt html page loads and populates the current configuration in the form
  StaticJsonBuffer<200> jsonBuffer;                                             // Define json buffer
  JsonObject& json = jsonBuffer.createObject();                                 // Build json object of program data
  json["mqtt_server"] = mqtt_config.mqtt_server;
  json["mqtt_port"] = mqtt_config.mqtt_port;
  json["mqtt_user"] = mqtt_config.mqtt_user;
  json["mqtt_pass"] = mqtt_config.mqtt_pass;
  json["mqtt_feed"] = mqtt_config.mqtt_feed;
  char jsonchar[500];
  json.printTo(jsonchar); //print to char array, takes more memory but sends in one piece
  Serial.println(jsonchar);
  server.send(200, "application/json", jsonchar);                               // Send OK response to http client
  jsonBuffer.clear();                                                           // Clear buffer to free used memory
 }
//-------------------------- save_mqtt() --------------------------------
 bool save_mqtt (){                                                             // Saves mqtt configuration to the json file
  StaticJsonBuffer<200> jsonBuffer;                                             // Define json buffer
  JsonObject &json = jsonBuffer.createObject();                                 // Build json object of program data
  json["mqtt_server"] = mqtt_config.mqtt_server;
  json["mqtt_port"]   = mqtt_config.mqtt_port;
  json["mqtt_user"]   = mqtt_config.mqtt_user;
  json["mqtt_pass"]   = mqtt_config.mqtt_pass;
  json["mqtt_feed"]   = mqtt_config.mqtt_feed;
  File mqtt_setup = SPIFFS.open("/mqtt_setup.json", "w");                       // Open json config file for writting
  if (!mqtt_setup) {
    Serial.println("Failed to open mqtt_setup.json file for writing");
    return false;
  }
  json.printTo(mqtt_setup);                                                     // Print json json data to the mqtt_setup file
  jsonBuffer.clear();                                                           // Clear the buffer memory
  Serial.println("MQTT Server Setup Saved in file " + String(mqtt_setup.name()));
  mqtt_setup.close();                                                           // Close json config file
  mqtt_client.disconnect();                                                     // Disconnect from the current mqtt server as new server data has been entered
  return true;
 }
//--------------------- load_mqtt() -------------------------------------
 bool load_mqtt (){                                                            // Loads the mqtt configuration from the json file during startup and stores into 
  Serial.println("Loading MQTT Server Setup from json file...");               // the global variable. 
  File mqtt_setup = SPIFFS.open("/mqtt_setup.json", "r");
  if (!mqtt_setup) {
    Serial.println("Failed to open mqtt_setup.json file for reading");
    return false;
  }
  size_t size = mqtt_setup.size();
  std::unique_ptr<char[]> buf(new char[size]);                                  // Allocate a buffer to store contents of the file.
  mqtt_setup.readBytes(buf.get(), size);                                        // Read bytes and copy them into the buffer (buf)
  StaticJsonBuffer<200> jsonBuffer;                                             // Allocate bytes to json buffer
  JsonObject& json = jsonBuffer.parseObject(buf.get());                         // Parse the config parameters
  if (!json.success()) {
    Serial.println("Failed to parse config file");
    return false;
  }
  mqtt_config.mqtt_server = json["mqtt_server"].as<String>();
  mqtt_config.mqtt_port = json["mqtt_port"].as<String>().toInt();
  mqtt_config.mqtt_user = json["mqtt_user"].as<String>();
  mqtt_config.mqtt_pass = json["mqtt_pass"].as<String>();
  mqtt_config.mqtt_feed = json["mqtt_feed"].as<String>();
  jsonBuffer.clear();                                                           // Clear the buffer memory
  mqtt_setup.close();                                                           // Close json config file
  Serial.println("MQTT Server: " + mqtt_config.mqtt_server + " / MQTT Port: " + String(mqtt_config.mqtt_port) + " / MQTT User: " + mqtt_config.mqtt_user + " / MQTT Password: " + mqtt_config.mqtt_pass + " / MQTT Feed: " + mqtt_config.mqtt_feed );
  return true;
}
//------------------------- process_dev_form() -------------------------
void process_dev_form(){
    Serial.println("server.on /dev for device " + server.arg("device"));        // Process the html form sent by the deviceX.htm page to update device config
    int i;                                                                      // this function will save the parameters in the form to the corresponding device struct
    device *dev;                                                                // with the device config data and then save all this data to the .json file in the  
    if ( server.arg("device") == "1" ) dev = &device1;                          // SPIFFS filesystem. This function uses a pointer to the device struct to avoid copying 
    if ( server.arg("device") == "2" ) dev = &device2;                          // variable data and hence being more processing and memory efficient.
    if ( server.arg("device") == "3" ) dev = &device3;
    if ( server.arg("device") == "4" ) dev = &device4;
    if ( server.arg("device") == "5" ) dev = &device5;
    if ( server.arg("device") == "6" ) dev = &device6;
    dev->device_name = server.arg("device_name");
    dev->device_name.toLowerCase();                                             // Device name defaults to lower case, commands received will also be converted to lower case for compatibility
    dev->ir_protocol = server.arg("ir_protocol");
    dev->hex_codes1[0] = hexToDec(server.arg("volup_HEX1"));
    dev->hex_codes2[0] = hexToDec(server.arg("volup_HEX2"));
    dev->hex_codes1[1] = hexToDec(server.arg("voldwn_HEX1"));
    dev->hex_codes2[1] = hexToDec(server.arg("voldwn_HEX2"));
    for ( i=0 ; i < 10 ; i++ ){
      dev->hex_numbers1[i] = hexToDec(server.arg("digit" + String(i) + "_HEX1"));
      dev->hex_numbers2[i] = hexToDec(server.arg("digit" + String(i) + "_HEX2"));
    }
    for ( i=1 ; i < (NUM_HEX_CODES-1) ; i++ ){
      dev->commands[i+1] = server.arg("comm" + String(i));
      dev->commands[i+1].toLowerCase();                                         // Commands need also to default to lower case
      dev->hex_codes1[i+1] = hexToDec(server.arg("comm" + String(i) + "_HEX1"));
      dev->hex_codes2[i+1] = hexToDec(server.arg("comm" + String(i) + "_HEX2"));
    }
    save_dev(dev);                                                              // Save the form data from the struct variable to the .json file in the SPIFFS filesystem        
    server.send(200, "text/html", "OK\r\n");                                    // Send OK response to the webserver client
}
//------------------------- req_dev() ----------------------------------
void req_dev (){                                                                // Sends all the device configuration from the device (ESP) to the client html page
  Serial.println("server.on /req_dev for device " + server.arg(0));             // the html page populates the form fields with data gathered from the ESP.
  int i;
  device *dev;
  if ( server.arg(0) == "1") dev = &device1;                                     // HTML form has a hidden input with the Device number, that is received by the webserver
  if ( server.arg(0) == "2") dev = &device2;                                     // as argument(0), this is used to decide to which variable we are copying the config data
  if ( server.arg(0) == "3") dev = &device3;
  if ( server.arg(0) == "4") dev = &device4;
  if ( server.arg(0) == "5") dev = &device5;
  if ( server.arg(0) == "6") dev = &device6;  
  DynamicJsonBuffer jsonBuffer(2000);                                            // Build json object of program data
  JsonObject& json = jsonBuffer.createObject();                                  // Define json buffer
  json["device_name"] = dev->device_name;
  json["ir_protocol"] = dev->ir_protocol;
  json["volup_HEX1"]  = decToHex(dev->hex_codes1[0], 8);
  json["volup_HEX2"]  = decToHex(dev->hex_codes2[0], 8);
  json["voldwn_HEX1"] = decToHex(dev->hex_codes1[1], 8);
  json["voldwn_HEX2"] = decToHex(dev->hex_codes2[1], 8);
  for ( i=2 ; i < NUM_HEX_CODES ; i++ ){                                     
    json["comm"+String(i-1)] = dev->commands[i];
    json["comm"+String(i-1)+"_HEX1"] = decToHex(dev->hex_codes1[i], 8);
    json["comm"+String(i-1)+"_HEX2"] = decToHex(dev->hex_codes2[i], 8);
  }
  for ( i=0 ; i < 10 ; i++ ){
    json["digit"+String(i)+"_HEX1"] = decToHex(dev->hex_numbers1[i], 8);
    json["digit"+String(i)+"_HEX2"] = decToHex(dev->hex_numbers2[i], 8);   
  }
  char jsonchar[2000];
  json.printTo(jsonchar);                                                       // Print to char array, takes more memory but sends in one piece
  //Serial.println(jsonchar);                                                   // Use for debugging purposes, prints all the configuration in json format to Serial
  server.send(200, "application/json", jsonchar);                               // Send response to the webserver client with config data
  jsonBuffer.clear();                                                           // Clear the buffer and free up memory   
 }
//-------------------------- save_dev() --------------------------------
 bool save_dev (device *dev){
  int i;                                                                        // This function saves the device configuration to the json file
  DynamicJsonBuffer jsonBuffer(2000);                                           // Build json object of program data
  JsonObject &json = jsonBuffer.createObject();                                 // Define json buffer
  File dev_setup = SPIFFS.open("/device" + server.arg("device") + ".json", "w");
  if (!dev_setup) {
    Serial.println("Failed to open device" + server.arg("device") + " json file for writing");
    return false;
  }
  json["device_name"] = dev->device_name;                                       // Convert variables to json format to send back to the http client.
  json["ir_protocol"] = dev->ir_protocol;
  json["volup_HEX1"]  = decToHex(dev->hex_codes1[0], 8);
  json["volup_HEX2"]  = decToHex(dev->hex_codes2[0], 8);
  json["voldwn_HEX1"] = decToHex(dev->hex_codes1[1], 8);
  json["voldwn_HEX2"] = decToHex(dev->hex_codes2[1], 8);
  for ( i=2 ; i < NUM_HEX_CODES ; i++ ){                                     
    json["comm"+String(i-1)] = dev->commands[i];                                // Convert commands to json format  
    json["comm"+String(i-1)+"_HEX1"] = decToHex(dev->hex_codes1[i], 8);
    json["comm"+String(i-1)+"_HEX2"] = decToHex(dev->hex_codes2[i], 8);
  }
  for ( i=0 ; i < 10 ; i++ ){
    json["digit"+String(i)+"_HEX1"] = decToHex(dev->hex_numbers1[i], 8);        // Convert digits to json format
    json["digit"+String(i)+"_HEX2"] = decToHex(dev->hex_numbers2[i], 8);   
  }
  json.printTo(dev_setup);                                                      // Send json variables to the file
  jsonBuffer.clear();                                                           // Clear the json buffer to frre up memory
  Serial.println("Device " + server.arg("device") + " configuration saved in file " + String(dev_setup.name()) );
  dev_setup.close();                                                            // Close the device config file 
  return true;
 }
//--------------------- load_dev() ----------------------------------------
bool load_dev (device *dev, String num){
  int i;                                                                        // This function loads the device configuration from the json file
  Serial.println("Loading Device " + num + " Setup from json file...");
  File dev_setup = SPIFFS.open("/device" + num + ".json", "r");
  if (!dev_setup) {
    Serial.println("Failed to open device" + num + ".json file for reading");
    return false;
  }
  size_t size = dev_setup.size();
  std::unique_ptr<char[]> buf(new char[size]);                                  // Allocate a buffer to store contents of the file.
  dev_setup.readBytes(buf.get(), size);                                         // Read bytes and copy them into the buffer (buf)
  StaticJsonBuffer<2000> jsonBuffer;                                            // Allocate bytes to json buffer
  JsonObject& json = jsonBuffer.parseObject(buf.get());                         // Parse the config parameters
  if (!json.success()) {
    Serial.println("Failed to parse config file");
    return false;
  }
    dev->device_name = json["device_name"].as<String>();                        // Store the json configuration data into the corresponding global variables
    dev->ir_protocol = json["ir_protocol"].as<String>();
    dev->hex_codes1[0] = hexToDec(json["volup_HEX1"].as<String>());
    dev->hex_codes2[0] = hexToDec(json["volup_HEX2"].as<String>());
    dev->hex_codes1[1] = hexToDec(json["voldwn_HEX1"].as<String>());
    dev->hex_codes2[1] = hexToDec(json["voldwn_HEX2"].as<String>());
    for ( i=0 ; i < 10 ; i++ ){
      dev->hex_numbers1[i] = hexToDec(json["digit" + String(i) + "_HEX1"].as<String>());
      dev->hex_numbers2[i] = hexToDec(json["digit" + String(i) + "_HEX2"].as<String>());
    }
    for ( i=1 ; i < (NUM_HEX_CODES-1) ; i++ ){
      dev->commands[i+1] = json["comm" + String(i)].as<String>();
      dev->hex_codes1[i+1] = hexToDec(json["comm" + String(i) + "_HEX1"].as<String>());
      dev->hex_codes2[i+1] = hexToDec(json["comm" + String(i) + "_HEX2"].as<String>());
    }
  jsonBuffer.clear();                                                           // Clear the json buffer to frre up memory
  dev_setup.close();                                                            // Close the device config file 
  return true;
}
//---------------------- process_proc_form() ------------------------------
void process_proc_form(){
    Serial.println("server.on /proc for procedure " + server.arg("procedure")); // Process the html form sent by the procedureX.htm page to update procedure config
    int i;                                                                      // this function will save the parameters in the form to the corresponding procedure struct
    procedure *proc;                                                            // with the procedure config data and then save all this data to the .json file in the  
    if ( server.arg("procedure") == "1" ) proc = &procedure1;                   // SPIFFS filesystem. This function uses a pointer to the procedure struct to avoid copying 
    if ( server.arg("procedure") == "2" ) proc = &procedure2;                   // variable data and hence being more processing and memory efficient.
    if ( server.arg("procedure") == "3" ) proc = &procedure3;
    proc->keyword = server.arg("keyword");
    proc->keyword.toLowerCase();
    for ( i=0 ; i < MAX_ARGUMENTS ; i++ ){
      proc->device[i]   =   server.arg("step" + String(i+1) + "_dev");
      proc->device[i].toLowerCase();                                            // Device and command are converted to lowercase for compatibility with commands received..
      proc->command[i]  =   server.arg("step" + String(i+1) + "_comm");         // which will also be converted to lowercase
      proc->command[i].toLowerCase();
      proc->repeats[i]  =   server.arg("step" + String(i+1) + "_repeat").toInt();
      proc->delay[i]    =   server.arg("step" + String(i+1) + "_delay").toInt();
    }
    save_proc(proc);                                                            // Save the form data from the struct variable to the .json file in the SPIFFS filesystem        
    server.send(200, "text/html", "OK\r\n");                                    // Send OK response to the webserver client  
}
//---------------------- req_proc() ---------------------------------------
void req_proc(){                                                                // Gets all the procedure configuration from the device (ESP) and then the html page
  Serial.println("server.on /req_proc for procedure " + server.arg(0));         // populates all that info in the form fields
  int i;
  procedure *proc;
  if ( server.arg(0) == "1") proc = &procedure1;                                // HTML form has a hidden input with the Device number, that is received by the webserver
  if ( server.arg(0) == "2") proc = &procedure2;                                // as argument(0), this is used to decide to which variable we are copying the config data
  if ( server.arg(0) == "3") proc = &procedure3;  
  DynamicJsonBuffer jsonBuffer(2000);                                           // Build json object of program data
  JsonObject& json = jsonBuffer.createObject();                                 // Define json buffer
  json["keyword"] = proc->keyword;
  for ( i=0 ; i < MAX_ARGUMENTS ; i++ ){                                        // Go through the 10 steps of the procedure and convert it to json data
    json["step" + String(i+1) + "_dev"]   = String(proc->device[i]);            
    json["step" + String(i+1) + "_comm"]  = String(proc->command[i]);
    json["step" + String(i+1) + "_repeat"]= proc->repeats[i];
    json["step" + String(i+1) + "_delay"] = proc->delay[i];
  }
  char jsonchar[2000];
  json.printTo(jsonchar);                                                       // Print to char array, takes more memory but sends in one piece
  //Serial.println(jsonchar);                                                   // Use for debugging purposes, prints all the configuration in json format to Serial
  server.send(200, "application/json", jsonchar);                               // Send response to the webserver client with config data
  jsonBuffer.clear();                                                           // Clear the buffer and free up memory  
}
//---------------------- save_proc() --------------------------------------
bool save_proc(procedure *proc){
  int i;                                                                        // This function saves a procedure in a json file after the user pressed save in the webpage
  DynamicJsonBuffer jsonBuffer(2000);                                           // Build json object of program data
  JsonObject &json = jsonBuffer.createObject();                                 // Define json buffer
  File proc_setup = SPIFFS.open("/procedure" + server.arg("procedure") + ".json", "w");
  if (!proc_setup) {
    Serial.println("Failed to open procedure" + server.arg("procedure") + " json file for writing");
    return false;
  }
  json["keyword"] = proc->keyword;
  for ( i=0 ; i < MAX_ARGUMENTS ; i++ ){                                     
    json["step" + String(i+1) + "_dev"]   = String(proc->device[i]);
    json["step" + String(i+1) + "_comm"]  = String(proc->command[i]);
    json["step" + String(i+1) + "_repeat"]= proc->repeats[i];
    json["step" + String(i+1) + "_delay"] = proc->delay[i];
  }
  json.printTo(proc_setup);                                                     // Send json variables to the file
  jsonBuffer.clear();                                                           // Clear the json buffer to frre up memory
  Serial.println("Procedure " + server.arg("procedure") + " configuration saved in file " + String(proc_setup.name()) );
  proc_setup.close();                                                           // Close the device config file 
  return true;
}
//---------------------- load_proc() --------------------------------------
bool load_proc(procedure *proc, String num){                                    // Loads a procedure saved in a json file in the SPIFFS filesystem to the corresponding
  int i;                                                                        // global variables.
  Serial.println("Loading Procedure " + num + " Setup from json file...");
  File proc_setup = SPIFFS.open("/procedure" + num + ".json", "r");
  if (!proc_setup) {
    Serial.println("Failed to open procedure" + num + ".json file for reading");
    return false;
  }
  size_t size = proc_setup.size();
  std::unique_ptr<char[]> buf(new char[size]);                                  // Allocate a buffer to store contents of the file.
  proc_setup.readBytes(buf.get(), size);                                        // Read bytes and copy them into the buffer (buf)
  StaticJsonBuffer<2000> jsonBuffer;                                            // Allocate bytes to json buffer
  JsonObject& json = jsonBuffer.parseObject(buf.get());                         // Parse the config parameters
  if (!json.success()) {
    Serial.println("Failed to parse config file");
    return false;
  }
    proc->keyword = json["keyword"].as<String>();
    for ( i=0 ; i < MAX_ARGUMENTS ; i++ ){
      proc->device[i]   =   json["step" + String(i+1) + "_dev"].as<String>();
      proc->command[i]  =   json["step" + String(i+1) + "_comm"].as<String>();
      proc->repeats[i]  =   json["step" + String(i+1) + "_repeat"].as<String>().toInt();
      proc->delay[i]    =   json["step" + String(i+1) + "_delay"].as<String>().toInt();
    }
  jsonBuffer.clear();                                                           // Clear the json buffer to frre up memory
  proc_setup.close();                                                           // Close the device config file 
  return true;
}
//---------------------- exec_procedure() ---------------------------------
void exec_procedure (procedure *proc){                                           // Executes a preset procedure (a series of commands) 
  int i, j, k;                                                                            
  device *dev; 
  Serial.println("Executing procedure " + proc->keyword );
  for ( i = 0 ; i < MAX_ARGUMENTS ; i++ ){
    if ( proc->device[i] == device1.device_name ) dev = &device1;
    if ( proc->device[i] == device2.device_name ) dev = &device2;
    if ( proc->device[i] == device3.device_name ) dev = &device3;
    if ( proc->device[i] == device4.device_name ) dev = &device4;
    if ( proc->device[i] == device5.device_name ) dev = &device5;
    if ( proc->device[i] == device6.device_name ) dev = &device6;
    if ( proc->device[i] != "" && proc->repeats[i] == 1 )Serial.println("Command " + proc->command[i] +" to " + proc->device[i] + ", repeat once");
    if ( proc->device[i] != "" && proc->repeats[i] >  1 )Serial.println("Command " + proc->command[i] +" to " + proc->device[i] + ", repeat " + String(proc->repeats[i]) + " times");
    for ( j = 0 ; j < NUM_HEX_CODES ; j++ ){
      if ( proc->command[i] == dev->commands[j]){
        for ( k = 0 ; k < proc->repeats[i] ; k++ ){
          send_ir(dev->ir_protocol, dev->hex_codes1[j]);
          send_ir(dev->ir_protocol, dev->hex_codes2[j]);
        }
        if( proc->delay[i] > 0 ){
          Serial.println("Delaying " + String(proc->delay[i]) + " seconds");
          delay ( proc->delay[i] * 1000 );                                        // Delay was defined by user in seconds so multiplying by 1000 converts to ms.
        } 
      }
    }
  }
}
//------------------------ send_ir() -----------------------------------
void send_ir(String protocol, int hex_code){
  if ( hex_code != 0 ){                                                           // Sends an IR signal with the correct ir protocol
    if ( protocol == "NEC")       irsend.sendNEC        (hex_code, 32);        
    if ( protocol == "SONY")      irsend.sendSony       (hex_code, 32);                                  
    if ( protocol == "PANASONIC") irsend.sendPanasonic  (hex_code, 32);                 
    if ( protocol == "SAMSUNG")   irsend.sendSAMSUNG    (hex_code, 32);                 
    if ( protocol == "LG")        irsend.sendLG         (hex_code, 32);                 
    if ( protocol == "JVC")       irsend.sendJVC        (hex_code, 32);                 
    if ( protocol == "DENON")     irsend.sendDenon      (hex_code, 32);                 
    Serial.print("Sending hex code: ");
    Serial.print(hex_code, HEX);                                                  // Print the command being sent
    Serial.println(" with protocol " + protocol);
  }
}
//-------------------------- receive_ir() ------------------------------
void receive_ir(){
  Serial.println("server.on /rec_ir for device " + server.arg(0));                // Receives an ir code from the remote control and sends the code and the protocol back to
  decode_results results;                                                         // the client html web page, updating the textbox with the code, then user has to press save to store.
  DynamicJsonBuffer jsonBuffer(200);                                              // Build json object of program data
  JsonObject& json = jsonBuffer.createObject();                                   // Define json buffer
  int flag = 0, timeout = 0;
  unsigned long lastmillis = millis();
  irrecv.resume();
  
  while ( flag == 0 && timeout == 0){
    if (irrecv.decode(&results)) {                                                // Check if a code has been received
      flag = 1;
      Serial.print(resultToHumanReadableBasic(&results));
      json["ir_protocol"] = typeToString(results.decode_type);
      json["ir_code1"] = decToHex(results.value, 8);
      irrecv.resume();
    }
    if( (millis()-lastmillis) > REMOTE_BTN_TIMEOUT )timeout = 1;
    yield();                                                                      // Attend wdt to avoid reset
  }
  if ( flag == 0 ) json["ir_protocol"] = String(server.arg(1));
  if ( flag == 0 ) json["ir_code1"] = decToHex(0, 8);
  json["ir_code2"] = decToHex(0, 8);
  
  char jsonchar[200];
  json.printTo(jsonchar);                                                         // Print to char array, takes more memory but sends in one piece
  Serial.println(jsonchar);                                                       // Use for debugging purposes, prints all the configuration in json format to Serial
  server.send(200, "application/json", jsonchar);                                 // Send response to the webserver client with config data
  jsonBuffer.clear();                                                             // Clear the buffer and free up memory  
}
//--------------------- mqtt_connect()  -----------------------------------
void mqtt_connect() {                                                             // Connects to the mqtt server using the saved parameters
  mqtt_client.setServer(mqtt_config.mqtt_server.c_str(), *(&mqtt_config.mqtt_port));
  mqtt_client.setCallback(mqtt_msg_recv);                                         // this function is non-blocking to permit the rest of the code to execute even if the
  Serial.print("Connecting to MQTT Broker...");                                   // mqtt is not connected, becaus the device also accepts http messages or commands
  if(!mqtt_client.connect(HOSTNAME, mqtt_config.mqtt_user.c_str(), mqtt_config.mqtt_pass.c_str())){
    Serial.println("Failed!");                                                    // If connection fails, wait 1 sec and will retry in the next iteration of the main loop()
    delay(1000);
    return;
  }
  String topic = mqtt_config.mqtt_user + mqtt_config.mqtt_feed;
  mqtt_client.subscribe(topic.c_str());                                           // Subscribe to the mqtt topic or feed
  Serial.println("Connected!"); 
}
//--------------------- mqtt_msg_recv() ----------------------------------
void mqtt_msg_recv (char* topic, byte* payload, unsigned int length) {            // Receives an command via MQTT and sends tem via IR    
  digitalWrite(MODE_LED, LOW);                                                    // Blink led to indicate an mqtt message has been received (even when the commands may be incorrect)
  delay(IR_DELAY);
  digitalWrite(MODE_LED, HIGH);
  int ir_code;                                                                    // Flag to indicate if all commands were incorrect (-1) or at least one was correct (1)
  payload[length] = '\0';                                                         // Add \0 at the end of byte array to convert to srting
  String command = String((char *)payload);                                       // String received from the MQTT server with all the commands sent separated by spaces
  Serial.println("New MQTT Command: " + command);
  parse_string(command);                                                          // Parse the command string delimiter is SPACE     
  ir_code = search_ir_command ();                                                 // This function interprets and sends IR codes accordingly
  if (ir_code != -1) Serial.println("Commands that are correct, have been sent... ");
  else Serial.println("All commands are invalid..."); 
}
//-------------------------- http_msg_recv() ---------------------------
void http_msg_recv(){                                                             // Receives an command via HTTP and sends tem via IR 
  digitalWrite(MODE_LED, LOW);                                                    // Blink led to indicate an mqtt message has been received (even when the commands may be incorrect)
  delay(IR_DELAY);
  digitalWrite(MODE_LED, HIGH);
  int ir_code;                                                                    // Flag to indicate if all commands were incorrect (-1) or at least one was correct (1)
  String command = server.arg(0);                                                 // String received via http server with all the commands sent separated by spaces
  server.send(200, "text/html", "OK\r\n");                                        // OK response to client before command is executed to avoid http timeouts, 
  command.toLowerCase();                                                          // Lowercase because some of google assistant commands came in uppercase
  Serial.println("New HTTP Command: " + command);                                 // because some procedures may take some seconds to execute.
  parse_string(command);                                                          // Parse the command string delimiter is SPACE     
  ir_code = search_ir_command ();                                                 // This function interprets and sends IR codes accordingly
  if (ir_code != -1) Serial.println("Commands that are correct, have been sent... ");
  else Serial.println("All commands are invalid..."); 
}
//--------------------- Converting from Hex to Decimal (by Ben Rugg) --------
int hexToDec(String hexString) {                                                   // This function a four digit hex string and converts it to an integer   
  int decValue = 0; 
  int nextInt;
  for (int i = 0; i < hexString.length(); i++) {
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    decValue = (decValue * 16) + nextInt;
  }
  return decValue;
}
//---------------------- Converting from Decimal to Hex (by Ben Rugg) -------
String decToHex(unsigned long decValue, byte desiredStringLength) {                 // This function converts an integer to a string in HEX format and will pad it
  String hexString = String(decValue, HEX);                                         // with 0's (on the left) if it is less than the desired string length.
  while (hexString.length() < desiredStringLength) hexString = "0" + hexString;    
  hexString.toUpperCase();
  return hexString;
}
//-------------------------- process_led_form() -----------------------
 void process_led_form (){                                                          // Receives the color configuration from the webpage, stores it into the global variables
    Serial.println("server.on /led");                                               // and saves the configuration to the json file.
    if (server.hasArg("mode")) {
      led_mode = server.arg("mode");
      led_intensity = server.arg("intensity").toInt();
      led_R = server.arg("R").toInt();
      led_G = server.arg("G").toInt();
      led_B = server.arg("B").toInt();
      Serial.println("Mode: " + led_mode + " / Intensity: " + led_intensity + " / R: " + String(led_R) + " / G: " + String(led_G) + " / B: " + String(led_B));
      save_led();
    }
    server.send(200, "text/html", "OK\r\n");
    set_led();
  } //process_led_form
//-------------------------- req_led() --------------------------------
 void req_led (){                                                               // Get mqtt configuration from global variables and send data back to the client webpage
  Serial.println("server.on /req_led");                                         // Used when the mqtt html page loads and populates the current configuration in the form
  StaticJsonBuffer<200> jsonBuffer;                                             // Define json buffer
  JsonObject& json = jsonBuffer.createObject();                                 // Build json object of program data
  json["mode"] = led_mode;
  json["intensity"] = led_intensity;
  json["R"] = led_R;
  json["G"] = led_G;
  json["B"] = led_B;
  char jsonchar[500];
  json.printTo(jsonchar); //print to char array, takes more memory but sends in one piece
  Serial.println(jsonchar);
  server.send(200, "application/json", jsonchar);                               // Send OK response to http client
  jsonBuffer.clear();                                                           // Clear buffer to free used memory
 }  
//-------------------------- save_led() --------------------------------
 bool save_led (){                                                                // Saves led config to the json file
  StaticJsonBuffer<200> jsonBuffer;                                               // Define json buffer
  JsonObject &json = jsonBuffer.createObject();                                   // Build json object of program data
  json["mode"] = led_mode;
  json["intensity"] = led_intensity;
  json["R"] = led_R;
  json["G"] = led_G;
  json["B"] = led_B;
  File led = SPIFFS.open("/led.json", "w");                                       // Open json config file for writting
  if (!led) {
    Serial.println("Failed to open led.json file for writing");
    return false;
  }
  json.printTo(led);                                                              // Print json json data to the mqtt_setup file
  jsonBuffer.clear();                                                             // Clear the buffer memory
  Serial.println("LED config Saved in file " + String(led.name()));
  led.close();                                                                    // Close json config file
  return true;
 }
 //--------------------- load_led() -------------------------------------
 bool load_led (){                                                            // Loads the mqtt configuration from the json file during startup and stores into 
  Serial.println("Loading LED Setup from json file...");               // the global variable. 
  File led = SPIFFS.open("/led.json", "r");
  if (!led) {
    Serial.println("Failed to open led.json file for reading");
    return false;
  }
  size_t size = led.size();
  std::unique_ptr<char[]> buf(new char[size]);                                  // Allocate a buffer to store contents of the file.
  led.readBytes(buf.get(), size);                                               // Read bytes and copy them into the buffer (buf)
  StaticJsonBuffer<200> jsonBuffer;                                             // Allocate bytes to json buffer
  JsonObject& json = jsonBuffer.parseObject(buf.get());                         // Parse the config parameters
  if (!json.success()) {
    Serial.println("Failed to parse config file");
    return false;
  }
  led_mode = json["mode"].as<String>();
  led_intensity = json["intensity"].as<String>().toInt();
  led_R = json["R"].as<String>().toInt();
  led_G = json["G"].as<String>().toInt();
  led_B = json["B"].as<String>().toInt();
  jsonBuffer.clear();                                                           // Clear the buffer memory
  led.close();                                                                  // Close json config file
  Serial.println("LED mode: " + led_mode + " / LED intensity: " + String(led_intensity) + " / R: " + String(led_R) + " / G: " + String(led_G) + " / B: " + String(led_B));
  set_led();
  return true;
}
 //--------------------- set_led() -------------------------------------
 void set_led(){
    if ( led_mode == "ON" ){
      analogWrite(LED_STRIP_R, (led_R/( 6 - led_intensity )));   
      analogWrite(LED_STRIP_G, (led_G/( 6 - led_intensity )));
      analogWrite(LED_STRIP_B, (led_B/( 6 - led_intensity )));
    }
    if ( led_mode == "OFF" ){
      analogWrite(LED_STRIP_R, 0);   
      analogWrite(LED_STRIP_G, 0);
      analogWrite(LED_STRIP_B, 0);      
    }
}

