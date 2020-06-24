/****************************************************
 * R. Kinnett, 2020
 * 
 * ESP32 2-way wireless audio interface
 * 
 * For use with VBAN Voicemeeter
 * https://www.vb-audio.com/Voicemeeter/index.htm
 * 
 *  Listens for VBAN packets over WiFi and checks
 *  packets for non-zero data received from host PC.
 *  If a VBAN packet contains non-zero data, then
 *  the ESP32 transitions to TRANSMIT mode, triggers 
 *  Push-to-talk (PTT), and begins writing the packet
 *  data to an i2s buffer which then streams the 
 *  data to the built-in digital-to-analog converter.
 *  
 *  When the ESP32 either receives no VBAN packets
 *  or receives VBAN packets containing all zeroes,
 *  the ESP32 transitions to RECEIVE mode, releases
 *  PTT, and begins sampling analog audio and
 *  streaming the digitized signal over VBAN to the
 *  host PC.
 *   
 * *************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUDP.h>
#include <driver/dac.h>
#include <driver/i2s.h>
#include "vban.h"
#include "esp_wifi.h"
#include "esp_err.h"
// #include "my_wifi.h"  // this defines my wifi ssid and password


/***************************************************************
 * If you want the ESP32 to connect to your home wifi network,
 * then uncomment the following 2 lines and enter your wifi credentials.
 * Or, include your own my_wifi.h with the following 2 lines.
 * Also, set I_AM_WIFI_ACCESS_POINT to false.
 * ***********************************************************/
// const char* ssid = "my_home_network_ssid";
// const char* password = "my_home_network_pw";

const bool I_AM_WIFI_ACCESS_POINT = true;
const char *ap_ssid = "esp32";
bool station_connected = false;



WiFiUDP udpIn, udpOut;
uint16_t udpListenPort = 6981;
uint16_t udpDestPort = 6980;
IPAddress destIP(192,168,1,19);
IPAddress myIP;
IPAddress my_static_IP(192,168,2,21);
IPAddress netmask(255,255,255,0);
bool udp_in_running = false;
bool udp_out_running = false;
bool udp_sending_packet = false;


// IMPORTANT Note:
// VBAN adjusts the number of samples per packet according to sample rate.
// Assuming 16-bit PCM mono, sample rates 11025, 22050, 44100, and 88200 yield 
// packets containing 64, 128, 256, and 256 samples per packet, respectively.
// The even-thousands sample rates below 48000 yield non-power-of-2 lengths.
// For example, sample rate 24000 yields 139 samples per packet.
// This VBAN->DMA->DAC method seems to require the dma buffer length be set
// equal to the number of samples in each VBAN packet.
// ESP32 I2S/DMA does not seem to handle non-power-of-2 buffer lengths well.
// Sample rate 24000 doesn't work reliably at all.
// Sample rate 32000 is stable but stutters.
// Recommend selecting from sample rates 11025, 22050, 44100, or anything above.
// And set samplesPerPacket to 64 for 11025, 128 for 22050, or 256 for all else.


const uint32_t initialSampleRate        = 11025;  // samples/sec; this must match VBAN Outgoing Stream SampleRate parameter
const uint16_t initialSamplesPerPacket  = 64;   // Use 64 for sample rate 11025 Hz, 128 for 22050 Hz, or 256 for 44100 and above.

const uint8_t ptt_pin = 32;

VBan vbanIn, vbanOut;

adc1_channel_t adc_channel = ADC1_CHANNEL_5; //GPIO33  (SET THIS FOR YOUR HARDWARE) ANALOG-IN PIN
const uint8_t  vban_out_sample_rate_selector = SAMPLE_RATE_11025_HZ;   // SET ADC SAMPLE RATE TO VALID VBAN RATE HERE
const uint16_t adc_sample_freq = VBanSRList[vban_out_sample_rate_selector];


i2s_port_t i2s_port  = I2S_NUM_0; // i2s port number (Built-in DAC functions are only supported on I2S0 for current ESP32 chip, purportedly)

const uint16_t i2s_in_dma_buffer_len = 64;
const uint16_t i2s_read_buffer_len = i2s_in_dma_buffer_len;
uint16_t* i2s_read_buff = (uint16_t*)calloc(i2s_read_buffer_len, sizeof(uint16_t));

i2s_config_t i2s_in_config, i2s_out_config;
bool i2s_out_running = false;
bool i2s_in_running = false;
bool i2s_in_reading = false;
uint16_t noDataToTransmitCounter = 0;
boolean streaming = true;
boolean sampling  = true;

const uint16_t idleTransmissionSignal[VBAN_PACKET_MAX_SAMPLES] = {32768};

unsigned long millisStartTransmission;

unsigned long prevPacketNbr = 0;
uint16_t packetCountThisTransmission = 0;



TaskHandle_t audioReadTaskHandle;
TaskHandle_t audioWriteTaskHandle;
#define AUDIO_READ_TASK_PRI 4
#define AUDIO_WRITE_TASK_PRI 4
static void audioReadTask(void * pvParameters);
static void audioWriteTask(void * pvParameters);
static void vbanSendPacket();
static void vbanReceivePacket();


enum TransmissionStates {
  RECEIVING,
  TRANSMITTING
};
uint8_t transmissionState;
char *strMode[] = {"RECEIVING", "TRANSMITTING"};


void initialize_i2s_configs(){
  /*   I2S "OUT":  digital-to-analog interface, from esp32 to radio */
  i2s_out_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = initialSampleRate,    // This must match VBAN Outgoing Stream SampleRate parameter
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,   // apparently built-in DAC only enabled on I2S0 which only allows 16bit?
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 1, // default interrupt priority
    .dma_buf_count = 32,
    .dma_buf_len = initialSamplesPerPacket,
    .use_apll = false,
//    .tx_desc_auto_clear = true
  };
  ESP_ERROR_CHECK( i2s_set_pin(i2s_port, NULL) );
  ESP_ERROR_CHECK( i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN)  );  // RIGHT=GPIO25, LEFT=GPIO26


  /*   I2S "IN":  analog-to-digital interface, from radio to esp32   */
  i2s_in_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),  // I2S receive mode with ADC
    .sample_rate = adc_sample_freq,                                               // set I2S ADC sample rate
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,                                 // 16 bit I2S (even though ADC is 12 bit)
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,                                 // handle adc data as single channel (right)
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S,               // I2S format
    .intr_alloc_flags = 0,                                                        // 
    .dma_buf_count = 32,                                                           // number of DMA buffers >=2 for fastness
    .dma_buf_len = i2s_in_dma_buffer_len,                                                // number of samples per buffer
    .use_apll = false,                                                            // no Audio PLL - buggy and not well documented
//    .tx_desc_auto_clear = true
  };
  adc1_config_channel_atten(adc_channel, ADC_ATTEN_11db);
  adc1_config_width(ADC_WIDTH_12Bit);
  ESP_ERROR_CHECK( i2s_set_adc_mode(ADC_UNIT_1, adc_channel) );
}


//////////////////////////   CONFIGURE VBAN PACKET   ////////////////////////////////
/* VBAN "out":  from esp32 to PC */
void configure_vban_out() {
  // Set vban packet header, counter, and data frame pointers to respective parts of packet:
  vbanOut.hdr           = (VBanHeader*) &vbanOut.packet[0];
  vbanOut.packet_number = (uint32_t*)   &vbanOut.packet[VBAN_PACKET_HEADER_BYTES];
  vbanOut.data_frame    = (int16_t*)    &vbanOut.packet[VBAN_PACKET_HEADER_BYTES + VBAN_PACKET_COUNTER_BYTES];
  
  // Setup the packet header:
  strncpy(vbanOut.hdr->preamble, "VBAN", 4);
  vbanOut.hdr->sample_rate      = VBAN_PROTOCOL_AUDIO | vban_out_sample_rate_selector;  // Set protocol and sample rate
  vbanOut.hdr->num_samples      = i2s_in_dma_buffer_len-1;                      // actual length = num_samples +1
  vbanOut.hdr->num_channels     = 0;                                            // actual channels = num_channels +1
  vbanOut.hdr->sample_format    = VBAN_BITFMT_16_INT | VBAN_CODEC_PCM;          // int16 PCM
  strncpy(vbanOut.hdr->stream_name, "ESP32_AUDIO_STRM", 16);

  *vbanOut.packet_number = 0;  // initialize packet counter

  vbanOut.data_bytes = (vbanOut.hdr->num_samples+1) * (vbanOut.hdr->num_channels+1) * ((vbanOut.hdr->sample_format & VBAN_BIT_RESOLUTION_MASK)+1);
  vbanOut.total_bytes = VBAN_PACKET_HEADER_BYTES + VBAN_PACKET_COUNTER_BYTES + vbanOut.data_bytes;
}



void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Station connected");
  station_connected = true;

  // Get IP address of remote station:
  wifi_sta_list_t wifi_sta_list;
  tcpip_adapter_sta_list_t adapter_sta_list;
  memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
  memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));
  esp_wifi_ap_get_sta_list(&wifi_sta_list);
  tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);
  tcpip_adapter_sta_info_t station = adapter_sta_list.sta[0];
  destIP.fromString(ip4addr_ntoa(&station.ip));
  Serial.println(destIP);
  delay(200);

  // Start UDP-in:
  Serial.print("initializing udp-in.. ");
  udp_in_running = (udpIn.begin(udpListenPort)==1);
  udpIn.flush();
  Serial.println(udp_in_running);

  // Start UDP-out and begin receiving:
  modeTransition(RECEIVING);
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Station disconnected");
  udpIn.stop();
  udp_in_running = false;
  udpOut.stop();
  udp_out_running = false;
}






void listConnectedStations(){
  wifi_sta_list_t wifi_sta_list;
  tcpip_adapter_sta_list_t adapter_sta_list;
  memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
  memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));
  esp_wifi_ap_get_sta_list(&wifi_sta_list);
  tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);
  Serial.printf("%i stations connected",adapter_sta_list.num);
  if(adapter_sta_list.num){
    Serial.print(":");
    for (int i = 0; i < adapter_sta_list.num; i++) {
      Serial.print(" ");
      tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];   
      Serial.print(ip4addr_ntoa(&(station.ip)));
    }
  }
  Serial.println();
}





//////////////////////////   SETUP   ////////////////////////////////////////
void setup() {
  Serial.begin(500000);

  pinMode(ptt_pin,OUTPUT);
  digitalWrite(ptt_pin, LOW);

  // Setup Wifi:
  if(I_AM_WIFI_ACCESS_POINT){
    WiFi.onEvent(PrintWiFiEvent);
    WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_AP_STAIPASSIGNED);
    WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_AP_STADISCONNECTED);
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid);
    delay(100); // necessary wait https://github.com/espressif/arduino-esp32/issues/985
    WiFi.softAPConfig(my_static_IP, my_static_IP, netmask);
    myIP = WiFi.softAPIP();
  } else {
    WiFi.begin(ssid, password);     //Connect to your WiFi router
    Serial.println("");
    while (WiFi.status() != WL_CONNECTED) {  // Wait for connection
      delay(500);
      Serial.print(".");
    }
    myIP = WiFi.localIP();
  }


  Serial.print("AP IP address: ");
  Serial.println(myIP);

  Serial.println("initializing i2s");
  initialize_i2s_configs();
  
  configure_vban_out();

  // If Wifi is not in AP mode then start receiving now; otherwise wait for client to connect to this AP
  if(!I_AM_WIFI_ACCESS_POINT){
    modeTransition(RECEIVING);  
  }

  Serial.println("Staring read/write tasks");
  xTaskCreatePinnedToCore(audioWriteTask, "audioWriteTask", 80000, NULL, AUDIO_WRITE_TASK_PRI, &audioWriteTaskHandle, 1 );
  xTaskCreatePinnedToCore(audioReadTask,  "audioReadTask",  40000, NULL, AUDIO_READ_TASK_PRI,  &audioReadTaskHandle,  1 );

  Serial.println("Setup done");
}


// This utility is very useful for troubleshooting
// and developing Wifi event-driven behaviors.
// It's not strictly necessary in a final build
// but may still be useful for debugging.
void PrintWiFiEvent(WiFiEvent_t event) {
    switch (event) {

    case SYSTEM_EVENT_WIFI_READY:
        Serial.println("SYSTEM_EVENT_WIFI_READY");
        break;

    case SYSTEM_EVENT_SCAN_DONE:
        Serial.println("SYSTEM_EVENT_SCAN_DONE");
        break;

    case SYSTEM_EVENT_STA_START:
        Serial.println("SYSTEM_EVENT_STA_START");
        break;

    case SYSTEM_EVENT_STA_STOP:
        Serial.println("SYSTEM_EVENT_STA_STOP");
        break;

    case SYSTEM_EVENT_STA_CONNECTED://or STARTED ?
        Serial.println("SYSTEM_EVENT_STA_CONNECTED");
        break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("SYSTEM_EVENT_STA_DISCONNECTED");
        break;

    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
        Serial.println("SYSTEM_EVENT_STA_AUTHMODE_CHANGE");
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("SYSTEM_EVENT_STA_GOT_IP");
        break;

    case SYSTEM_EVENT_STA_LOST_IP:
        Serial.println("SYSTEM_EVENT_STA_LOST_IP");
        break;

    case SYSTEM_EVENT_AP_START:
        Serial.println("SYSTEM_EVENT_AP_START");
        break;

    case SYSTEM_EVENT_AP_STOP:
        Serial.println("SYSTEM_EVENT_AP_STOP");
        break;

    case SYSTEM_EVENT_AP_STACONNECTED:
        Serial.println("SYSTEM_EVENT_AP_STACONNECTED");
        break;

    case SYSTEM_EVENT_AP_STADISCONNECTED:
        Serial.println("SYSTEM_EVENT_AP_STADISCONNECTED");
        break;

    case SYSTEM_EVENT_AP_STAIPASSIGNED:
        Serial.println("SYSTEM_EVENT_AP_STAIPASSIGNED");
        break;

    case SYSTEM_EVENT_AP_PROBEREQRECVED:
        Serial.println("SYSTEM_EVENT_AP_PROBEREQRECVED");
        break;

    default:
        Serial.println("UNKNOWN EVENT: " + event);
        break;
    }
}



void loop() {
  // Note: udp receiving and dma -> i2s -> dac streaming is handled in dedicated loop
  // Reserve primary loop for infrastructural things you might want to check periodically.
  /*
  Serial.print( eTaskGetState(audioReadTaskHandle) );
  Serial.print(" ");
  Serial.print( eTaskGetState(audioWriteTaskHandle) );
  Serial.println();
  */

  /*
  if(I_AM_WIFI_ACCESS_POINT){
    Serial.print("Stations connected: ");
    Serial.println(WiFi.softAPgetStationNum());
    listConnectedStations();
    delay(5000);
  }
  */
  
  delay(500);
  //Serial.println();
}



void triggerPTT(){
  digitalWrite(ptt_pin, HIGH);
}

void releasePTT(){
  digitalWrite(ptt_pin, LOW);
}



void modeTransition(uint8_t toMode) {
  size_t    bytesOut;

  Serial.printf("%s mode\n", strMode[toMode]);

  // Start i2s:
  switch(toMode) {
 
    case RECEIVING:
      // STOP TRANSMITTING:
      Serial.println("  Releasing PTT");
      releasePTT();
      if(i2s_out_running){        
        Serial.println("  Stopping i2s-out");
        i2s_out_running = false;
        ESP_ERROR_CHECK( i2s_stop(i2s_port) );
        ESP_ERROR_CHECK( i2s_zero_dma_buffer(i2s_port) );
        ESP_ERROR_CHECK( i2s_driver_uninstall(i2s_port) );
      }

      // BEGIN SAMPLING:
      Serial.println("  Starting i2s-in");
//      Serial.println("    i2s install");
      ESP_ERROR_CHECK( i2s_driver_install(i2s_port, &i2s_in_config, 0, NULL) );   //install and start i2s driver
//      Serial.println("    adc enable");
//      ESP_ERROR_CHECK( i2s_set_adc_mode(ADC_UNIT_1, adc_channel) );
//      Serial.println("    i2s clearing buffer");
      ESP_ERROR_CHECK( i2s_zero_dma_buffer(i2s_port) );
//      Serial.println("    adc enable");
      ESP_ERROR_CHECK( i2s_adc_enable(i2s_port) );
//      Serial.println("    i2s running");
//      vTaskDelay(5);
      i2s_in_running = true;
      vTaskDelay(50);

      // BEGIN UDP-OUT
      Serial.print("  Starting udp-out..");
      udpOut.flush();
      udp_out_running = udpOut.begin(udpDestPort);
      Serial.println(udp_out_running);
      break;




    case TRANSMITTING:
      // STOP UDP-OUT:
      udp_out_running = false;
      Serial.print("Stopping udp-out");
      while(udp_sending_packet){
        Serial.print(".");
        vTaskDelay(1);
      }
      Serial.println();
      udpOut.stop();
      udpOut.flush();
      
      // STOP SAMPLING
      Serial.print("  Stopping i2s-in");
      i2s_in_running = false;
      while(i2s_in_reading){
        Serial.print(".");
        vTaskDelay(1);
      }
      Serial.println();
      ESP_ERROR_CHECK( i2s_stop(i2s_port) );
      ESP_ERROR_CHECK( i2s_zero_dma_buffer(i2s_port) );
      ESP_ERROR_CHECK( i2s_adc_disable(i2s_port) );
      ESP_ERROR_CHECK( i2s_driver_uninstall(i2s_port) );
      vTaskDelay(1);


      // SETUP i2s-OUT:
      Serial.println("  Starting i2s-out");
      Serial.println("    i2s install");
      ESP_ERROR_CHECK( i2s_driver_install(i2s_port, &i2s_out_config, 0, NULL)  );   //install and start i2s driver

//      Serial.println("    stopping i2s");
//      ESP_ERROR_CHECK( i2s_stop(i2s_port) );              //stop i2s temporarily while we pre-populate the buffer.
//      Serial.println("    clearing i2s buffer");
//      ESP_ERROR_CHECK( i2s_zero_dma_buffer(i2s_port) );   // clear dma buffer

      vTaskDelay(1);
//      Serial.println("    restarting i2s-out");
//      ESP_ERROR_CHECK( i2s_start(i2s_port) );             //restart i2s
//      vTaskDelay(2);
      i2s_out_running = true;
      
      // BEGIN TRANSMITTING:
      Serial.println("  Triggering PTT");
      triggerPTT();
 //     vTaskDelay(2);
//      ESP_ERROR_CHECK( i2s_write( i2s_port, &idleTransmissionSignal, i2s_out_config.dma_buf_len*sizeof(uint16_t), &bytesOut, portMAX_DELAY ) );
//      vTaskDelay(2);
//      ESP_ERROR_CHECK( i2s_write( i2s_port, &idleTransmissionSignal, i2s_out_config.dma_buf_len*sizeof(uint16_t), &bytesOut, portMAX_DELAY ) );
//      vTaskDelay(2);
//      ESP_ERROR_CHECK( i2s_write( i2s_port, &idleTransmissionSignal, i2s_out_config.dma_buf_len*sizeof(uint16_t), &bytesOut, portMAX_DELAY ) );
                    
//      ESP_ERROR_CHECK( i2s_write( i2s_port, &idleTransmissionSignal, i2s_out_config.dma_buf_len*sizeof(uint16_t), &bytesOut, portMAX_DELAY ) );
//      ESP_ERROR_CHECK( i2s_write( i2s_port, &idleTransmissionSignal, i2s_out_config.dma_buf_len*sizeof(uint16_t), &bytesOut, portMAX_DELAY ) );

      //vTaskDelay(10);  // FIXME: parameterize    
  }

  transmissionState = toMode;
  Serial.printf("%s Mode\n", strMode[toMode]);
}




/////////////////////   SAMPLING TASK AND FUNCTIONS  ////////////////////
static void audioReadTask(void * pvParameters){
  size_t bytes_read;
  Serial.println("Receiving task started");
  for( ;; ){
    if(transmissionState==RECEIVING && i2s_in_running){
      i2s_in_reading = true;
      ESP_ERROR_CHECK( i2s_read(i2s_port, (void*)i2s_read_buff, i2s_read_buffer_len*sizeof(uint16_t), &bytes_read, portMAX_DELAY) );
      if(udp_out_running && I2S_EVENT_RX_DONE && bytes_read>0){
        vbanSendPacket();
      } else {
        Serial.print(".");
      }
      vTaskDelay(1);
      i2s_in_reading = false;
    } else {
      Serial.print("x");
      vTaskDelay(1);
    }

    // To-do:  consider calculating longer delay on-the-fly
  }
}


static void vbanSendPacket(){
  int16_t adc_data[VBAN_PACKET_MAX_SAMPLES];

  //// Per esp32.com forum topic 11023, esp32 swaps even/odd samples,
  ////   i.g. samples 0 1 2 3 4 5 are stored as 1 0 3 2 5 4 ..
  ////   Have to deinterleave manually; use xor "^1" to leap frog indices
  //// Also need to mask upper 4 bits which contain channel info (see gitter chat between me-no-dev and bzeeman)  
  for(int i=0; i<VBAN_PACKET_MAX_SAMPLES; i++){  // caution: this is not robust to odd buffer lens
    adc_data[i^1]   = (int16_t)(i2s_read_buff[i] & 0x0FFF) - 2048;
  }
  memcpy(vbanOut.data_frame, adc_data, vbanOut.data_bytes);

  // Send packet

  if(udp_out_running){
    udp_sending_packet = true;
    udpOut.beginPacket(destIP, udpDestPort);
    udpOut.write((uint8_t*)&vbanOut.packet, vbanOut.total_bytes);
    udpOut.endPacket();  
    udp_sending_packet = false;
  }
  
  (*vbanOut.packet_number)++;   // increment packet counter
}



////////////////////////////    VBAN-RX and ANALOG-OUT FUNCTIONS    /////////////////////////////////
static void audioWriteTask(void * pvParameters){
  // uint16_t delayMsec = (uint16_t) (1000.0 * i2s_out_config.dma_buf_len / i2s_out_config.sample_rate);
  Serial.println("Started audio-out task");
  for( ;; ){
    if(udp_in_running){
      vbanReceivePacket(); 
    } else {
      vTaskDelay(2);
    }
    vTaskDelay(2);
  }
}


void vbanReceivePacket(){
  char      udpIncomingPacket[VBAN_PACKET_MAX_LEN_BYTES];
  uint16_t  i2sOutBuf[VBAN_PACKET_MAX_SAMPLES+1];
  uint16_t  udpPacketLen;
  uint8_t   vbanSampleRateIdx;
  uint32_t  vbanSampleRate;
  bool      receivedNonZeroData;
  uint32_t  deltaPacketNbr;
  size_t    bytesOut;

  vbanIn.packet_number  = (uint32_t*)&udpIncomingPacket[VBAN_PACKET_HEADER_BYTES];
  vbanIn.data_frame     = (int16_t*)&udpIncomingPacket[VBAN_PACKET_HEADER_BYTES+VBAN_PACKET_COUNTER_BYTES];

  // read UDP buffer and process if non-empty:
  if (udp_in_running && udpIn.parsePacket()) {

    // receive incoming UDP packet
    udpPacketLen = udpIn.read(udpIncomingPacket, VBAN_PACKET_MAX_LEN_BYTES);
    //Serial.printf("%d B\n", vbanIn.total_bytes);

    // Check if packet length meets VBAN specification:
    if (udpPacketLen<=(VBAN_PACKET_HEADER_BYTES+VBAN_PACKET_COUNTER_BYTES) || udpPacketLen>VBAN_PACKET_MAX_LEN_BYTES) {
      Serial.printf("Error: packet length %u bytes\n", udpPacketLen);
      return;
    }
    
    // Check if preamble matches VBAN format:
    if(strncmp("VBAN",udpIncomingPacket,4)!=0){
      Serial.printf("Unrecognized preamble %.4s\n", udpIncomingPacket);
      return;
    }
    

    vbanIn.total_bytes  = udpPacketLen;
    vbanIn.data_bytes   = vbanIn.total_bytes - (VBAN_PACKET_HEADER_BYTES+VBAN_PACKET_COUNTER_BYTES);
    vbanIn.sample_count = vbanIn.data_bytes/2;
    vbanIn.packet_number = (uint32_t*)&udpIncomingPacket[VBAN_PACKET_HEADER_BYTES];
    
    vbanSampleRateIdx = udpIncomingPacket[4] & VBAN_SR_MASK;
    vbanSampleRate    = VBanSRList[vbanSampleRateIdx];
    //Serial.printf("%u %u %u\n", vbanSampleRate, vbanIn.sample_count, i2s_out_config.dma_buf_len);

    // Just to be safe, re-check sample count against max sample count to avoid overrunning i2sOutBuf later
    if(vbanIn.sample_count > VBAN_PACKET_MAX_SAMPLES){
      Serial.printf("error: unexpected packet size: %u\n",vbanIn.sample_count);
      return;
    }

    // Important notes from:  https://esp32.com/viewtopic.php?t=8234:  
    // 1. The I2S needs 16-bit samples.
    // 2. The I2S reads the samples as 32-bit words and outputs the high 16-bit first and the low 16-bit second. 
    //      This means that if you want to send it 16-bit samples, you'll need to swap the even and odd 16-bit words.
    // 3. The DAC only plays the high byte of the 16-bit sample as an unsigned value (0..255, not -128..127).

    //Serial.printf("%08X\n", *vbanIn.pkt_nbr);

    receivedNonZeroData = false;
    for(int i=0; i<vbanIn.sample_count; i++){
      // Input stream is signed int16, with min -32768 and max +32768.
      // Output needs to be uint16 with audio range in upper 8 bits,
      // Add 2^16/2 to remap from 0 to 65535.
      // Note:  index i^1 (XOR 1) swaps even and odd samples, per note above.
      i2sOutBuf[i^1] = (vbanIn.data_frame[i] + 32768);
      if(vbanIn.data_frame[i]!=0){
        receivedNonZeroData = true;
      }
      //Serial.printf("\t%X", (uint8_t)(i2sOutBuf[i^1]>>8));
    }
    //Serial.println();

    // Write to DAC only if received non-zero data:
    if(receivedNonZeroData){
      noDataToTransmitCounter = 0;  // reset no-data counter
      // If necessary, transition to TRANSMITTING and/or reconfigre I2S to match VBAN format:
      if(transmissionState==RECEIVING || vbanSampleRate!=i2s_out_config.sample_rate || vbanIn.sample_count!=i2s_out_config.dma_buf_len){
        Serial.printf("Configuring i2s-out with sample rate %u Hz and buffer length %u bytes\n", vbanSampleRate, vbanIn.sample_count);
        i2s_out_config.sample_rate = vbanSampleRate; 
        i2s_out_config.dma_buf_len = vbanIn.sample_count;
        modeTransition(TRANSMITTING);

        Serial.println("    pre-populating i2s buffer");
        for(int i=0; i<16; i++){
          //vTaskDelay(1);
          ESP_ERROR_CHECK( i2s_write( i2s_port, &idleTransmissionSignal, i2s_out_config.dma_buf_len*sizeof(uint16_t), &bytesOut, portMAX_DELAY ) );
          Serial.println(bytesOut);
        }
        prevPacketNbr = *vbanIn.packet_number;
      }
      
    } else {
      //Serial.print("0");
     
      noDataToTransmitCounter++;
      checkIfReadyToTransitionToReceiving();
    }
    

    if(transmissionState==TRANSMITTING && i2s_out_running){
      //Serial.println(*vbanIn.packet_number);
      ESP_ERROR_CHECK( i2s_write( i2s_port, &i2sOutBuf, vbanIn.sample_count*sizeof(uint16_t), &bytesOut, portMAX_DELAY ) );
      //Serial.printf("pkt %04ul, %u vals, %i, %i, %i\n",vbanIn.pkt_nbr, vbanIn.data_vals, pkt_data_min_val, pkt_data_max_val, bytesOut);

      // Report missing packets:
      deltaPacketNbr = *vbanIn.packet_number - prevPacketNbr;
      if(deltaPacketNbr > 1){
        Serial.printf("delta packet nbr: %u\n",deltaPacketNbr);
      }
    }

    // to-do:  report gaps in vban frame numbers

    //Serial.println(noDataToTransmitCounter);

    prevPacketNbr = *vbanIn.packet_number;
    

  } else {  // handle no-data
    //Serial.print("x");
    noDataToTransmitCounter++;
    checkIfReadyToTransitionToReceiving();
    //vTaskDelay(1);
  }
}



void checkIfReadyToTransitionToReceiving(){
  if(noDataToTransmitCounter>=15){
    if(transmissionState==TRANSMITTING){
      //Serial.println("Initiating transition to receiving");
      modeTransition(RECEIVING);
    }
    noDataToTransmitCounter = 0; //reset counter
  }
}
