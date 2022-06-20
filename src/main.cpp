#include <Arduino.h>
#include "Config.h"
#include "Storage.h"
#include "Signature.h"
#include "AzureDpsClient.h"
#include "CliMode.h"

// Libraries
#include "TFT_eSPI.h" //TFT LCD library
#include "SPI.h"

#include <PZEM004Tv30.h>
#include <Free_Fonts.h>
#include <Seeed_FS.h>

#include <rpcWiFiClientSecure.h>
#include <PubSubClient.h>

#include <WiFiUdp.h>
#include <NTP.h>

#include <azure/core/az_json.h>
#include <azure/core/az_result.h>
#include <azure/core/az_span.h>
#include <azure/iot/az_iot_hub_client.h>

#define MQTT_PACKET_SIZE 1024

// Pzem Definitions
#define uart Serial1

// Initializations
TFT_eSPI tft;                        // Initializing TFT LCD
TFT_eSprite spr = TFT_eSprite(&tft); // Initializing buffer

PZEM004Tv30 pzem;

int page = 1; // Khai bao so trang hien thi
int dem;

int i = 0; // la bien dem mac dinh cho thu tu chu so trong password
int k = 0;
int pwd[3] = {1, 2, 3}; // khai bao mat khau
int currentpwd[3] = {0, 0, 0};
int checkpwd = 0;

char dataStr[100] = "";
char buffer[7];

int yea, mon, da, hou, minu, sec;// bien hien thi thoi gian
float a,b,s; // Khai bao bien do nang luong
float v,c,p,e,f,pf; // Khai bao bien do cua Pzem
float v_SD = 0,c_SD = 0,p_SD = 0,f_SD = 0,pf_SD = 0, m = 0; // khai bao bien luu gia tri do duoc vao the nho



unsigned long startMillisPZEM;
unsigned long currentMillisPZEM;
const unsigned long delayPZEM = 1000;

void readDataPZEM();
void Screen1();
void Screen2();
void Screen3();
void Screen4();
void checkPassword();

const char *ROOT_CA_BALTIMORE =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ\n"
    "RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD\n"
    "VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX\n"
    "DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y\n"
    "ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy\n"
    "VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr\n"
    "mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr\n"
    "IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK\n"
    "mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu\n"
    "XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy\n"
    "dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye\n"
    "jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1\n"
    "BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3\n"
    "DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92\n"
    "9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx\n"
    "jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0\n"
    "Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz\n"
    "ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS\n"
    "R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp\n"
    "-----END CERTIFICATE-----";

WiFiClientSecure wifi_client;
PubSubClient mqtt_client(wifi_client);
WiFiUDP wifi_udp;
NTP ntp(wifi_udp);

std::string HubHost;
std::string DeviceId;

#define AZ_RETURN_IF_FAILED(exp)         \
    do                                   \
    {                                    \
        az_result const _result = (exp); \
        if (az_result_failed(_result))   \
        {                                \
            return _result;              \
        }                                \
    } while (0)

////////////////////////////////////////////////////////////////////////////////
//

#define DLM "\r\n"

static String StringVFormat(const char *format, va_list arg)
{
    const int len = vsnprintf(nullptr, 0, format, arg);
    char str[len + 1];
    vsnprintf(str, sizeof(str), format, arg);

    return String{str};
}

static void Abort(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    String str{StringVFormat(format, arg)};
    va_end(arg);

    Serial.print(String::format("ABORT: %s" DLM, str.c_str()));

    while (true)
    {
    }
}

static void Log(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    String str{StringVFormat(format, arg)};
    va_end(arg);

    Serial.print(str);
}

////////////////////////////////////////////////////////////////////////////////
// Display
// 
static void DisplayPrintf(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    String str{StringVFormat(format, arg)};
    va_end(arg);

    Log("%s\n", str.c_str());
}

////////////////////////////////////////////////////////////////////////////////
// Azure IoT DPS

static AzureDpsClient DpsClient;
static unsigned long DpsPublishTimeOfQueryStatus = 0;

static void MqttSubscribeCallbackDPS(char *topic, byte *payload, unsigned int length);

static int RegisterDeviceToDPS(const std::string &endpoint, const std::string &idScope, const std::string &registrationId, const std::string &symmetricKey, const uint64_t &expirationEpochTime, std::string *hubHost, std::string *deviceId)
{
    std::string endpointAndPort{endpoint};
    endpointAndPort += ":";
    endpointAndPort += std::to_string(8883);

    if (DpsClient.Init(endpointAndPort, idScope, registrationId) != 0)
        return -1;

    const std::string mqttClientId = DpsClient.GetMqttClientId();
    const std::string mqttUsername = DpsClient.GetMqttUsername();

    const std::vector<uint8_t> signature = DpsClient.GetSignature(expirationEpochTime);
    const std::string encryptedSignature = GenerateEncryptedSignature(symmetricKey, signature);
    const std::string mqttPassword = DpsClient.GetMqttPassword(encryptedSignature, expirationEpochTime);

    const std::string registerPublishTopic = DpsClient.GetRegisterPublishTopic();
    const std::string registerSubscribeTopic = DpsClient.GetRegisterSubscribeTopic();

    Log("DPS:" DLM);
    Log(" Endpoint = %s" DLM, endpoint.c_str());
    Log(" Id scope = %s" DLM, idScope.c_str());
    Log(" Registration id = %s" DLM, registrationId.c_str());
    Log(" MQTT client id = %s" DLM, mqttClientId.c_str());
    Log(" MQTT username = %s" DLM, mqttUsername.c_str());
    // Log(" MQTT password = %s" DLM, mqttPassword.c_str());

    wifi_client.setCACert(ROOT_CA_BALTIMORE);
    mqtt_client.setBufferSize(MQTT_PACKET_SIZE);
    mqtt_client.setServer(endpoint.c_str(), 8883);
    mqtt_client.setCallback(MqttSubscribeCallbackDPS);
    DisplayPrintf("Connecting to Azure IoT Hub DPS...");
    if (!mqtt_client.connect(mqttClientId.c_str(), mqttUsername.c_str(), mqttPassword.c_str()))
        return -2;

    mqtt_client.subscribe(registerSubscribeTopic.c_str());
    mqtt_client.publish(registerPublishTopic.c_str(), "{payload:{\"modelId\":\"" IOT_CONFIG_MODEL_ID "\"}}");

    while (!DpsClient.IsRegisterOperationCompleted())
    {
        mqtt_client.loop();
        if (DpsPublishTimeOfQueryStatus > 0 && millis() >= DpsPublishTimeOfQueryStatus)
        {
            mqtt_client.publish(DpsClient.GetQueryStatusPublishTopic().c_str(), "");
            Log("Client sent operation query message" DLM);
            DpsPublishTimeOfQueryStatus = 0;
        }
    }

    if (!DpsClient.IsAssigned())
        return -3;

    mqtt_client.disconnect();

    *hubHost = DpsClient.GetHubHost();
    *deviceId = DpsClient.GetDeviceId();

    Log("Device provisioned:" DLM);
    Log(" Hub host = %s" DLM, hubHost->c_str());
    Log(" Device id = %s" DLM, deviceId->c_str());

    return 0;
}

static void MqttSubscribeCallbackDPS(char *topic, byte *payload, unsigned int length)
{
    Log("Subscribe:" DLM " %s" DLM " %.*s" DLM, topic, length, (const char *)payload);

    if (DpsClient.RegisterSubscribeWork(topic, std::vector<uint8_t>(payload, payload + length)) != 0)
    {
        Log("Failed to parse topic and/or payload" DLM);
        return;
    }

    if (!DpsClient.IsRegisterOperationCompleted())
    {
        const int waitSeconds = DpsClient.GetWaitBeforeQueryStatusSeconds();
        Log("Querying after %u  seconds..." DLM, waitSeconds);

        DpsPublishTimeOfQueryStatus = millis() + waitSeconds * 1000;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Azure IoT Hub

static az_iot_hub_client HubClient;

static int SendCommandResponse(az_iot_hub_client_method_request *request, uint16_t status, az_span response);
static void MqttSubscribeCallbackHub(char *topic, byte *payload, unsigned int length);

static int ConnectToHub(az_iot_hub_client *iot_hub_client, const std::string &host, const std::string &deviceId, const std::string &symmetricKey, const uint64_t &expirationEpochTime)
{
    static std::string deviceIdCache;
    deviceIdCache = deviceId;

    const az_span hostSpan{az_span_create((uint8_t *)&host[0], host.size())};
    const az_span deviceIdSpan{az_span_create((uint8_t *)&deviceIdCache[0], deviceIdCache.size())};
    az_iot_hub_client_options options = az_iot_hub_client_options_default();
    options.model_id = AZ_SPAN_LITERAL_FROM_STR(IOT_CONFIG_MODEL_ID);
    if (az_result_failed(az_iot_hub_client_init(iot_hub_client, hostSpan, deviceIdSpan, &options)))
        return -1;

    char mqttClientId[128];
    size_t client_id_length;
    if (az_result_failed(az_iot_hub_client_get_client_id(iot_hub_client, mqttClientId, sizeof(mqttClientId), &client_id_length)))
        return -4;

    char mqttUsername[256];
    if (az_result_failed(az_iot_hub_client_get_user_name(iot_hub_client, mqttUsername, sizeof(mqttUsername), NULL)))
        return -5;

    char mqttPassword[300];
    uint8_t signatureBuf[256];
    az_span signatureSpan = az_span_create(signatureBuf, sizeof(signatureBuf));
    az_span signatureValidSpan;
    if (az_result_failed(az_iot_hub_client_sas_get_signature(iot_hub_client, expirationEpochTime, signatureSpan, &signatureValidSpan)))
        return -2;
    const std::vector<uint8_t> signature(az_span_ptr(signatureValidSpan), az_span_ptr(signatureValidSpan) + az_span_size(signatureValidSpan));
    const std::string encryptedSignature = GenerateEncryptedSignature(symmetricKey, signature);
    az_span encryptedSignatureSpan = az_span_create((uint8_t *)&encryptedSignature[0], encryptedSignature.size());
    if (az_result_failed(az_iot_hub_client_sas_get_password(iot_hub_client, expirationEpochTime, encryptedSignatureSpan, AZ_SPAN_EMPTY, mqttPassword, sizeof(mqttPassword), NULL)))
        return -3;

    Log("Hub:" DLM);
    Log(" Host = %s" DLM, host.c_str());
    Log(" Device id = %s" DLM, deviceIdCache.c_str());
    Log(" MQTT client id = %s" DLM, mqttClientId);
    Log(" MQTT username = %s" DLM, mqttUsername);
    // Log(" MQTT password = %s" DLM, mqttPassword);

    wifi_client.setCACert(ROOT_CA_BALTIMORE);
    mqtt_client.setBufferSize(MQTT_PACKET_SIZE);
    mqtt_client.setServer(host.c_str(), 8883);
    mqtt_client.setCallback(MqttSubscribeCallbackHub);

    if (!mqtt_client.connect(mqttClientId, mqttUsername, mqttPassword))
        return -6;

    mqtt_client.subscribe(AZ_IOT_HUB_CLIENT_METHODS_SUBSCRIBE_TOPIC);
    mqtt_client.subscribe(AZ_IOT_HUB_CLIENT_C2D_SUBSCRIBE_TOPIC);

    return 0;
}

static az_result SendTelemetry()
{
    char telemetry_topic[128];
    if (az_result_failed(az_iot_hub_client_telemetry_get_publish_topic(&HubClient, NULL, telemetry_topic, sizeof(telemetry_topic), NULL)))
    {
        Log("Failed az_iot_hub_client_telemetry_get_publish_topic" DLM);
        return AZ_ERROR_NOT_SUPPORTED;
    }

    az_json_writer json_builder;
    char telemetry_payload[200];
    AZ_RETURN_IF_FAILED(az_json_writer_init(&json_builder, AZ_SPAN_FROM_BUFFER(telemetry_payload), NULL));
    AZ_RETURN_IF_FAILED(az_json_writer_append_begin_object(&json_builder));
    AZ_RETURN_IF_FAILED(az_json_writer_append_property_name(&json_builder, AZ_SPAN_LITERAL_FROM_STR("v")));
    AZ_RETURN_IF_FAILED(az_json_writer_append_double(&json_builder,v, 1));
    AZ_RETURN_IF_FAILED(az_json_writer_append_property_name(&json_builder, AZ_SPAN_LITERAL_FROM_STR("c")));
    AZ_RETURN_IF_FAILED(az_json_writer_append_double(&json_builder,c, 3));
    AZ_RETURN_IF_FAILED(az_json_writer_append_property_name(&json_builder, AZ_SPAN_LITERAL_FROM_STR("p")));
    AZ_RETURN_IF_FAILED(az_json_writer_append_double(&json_builder,p, 3));
    AZ_RETURN_IF_FAILED(az_json_writer_append_property_name(&json_builder, AZ_SPAN_LITERAL_FROM_STR("e")));
    AZ_RETURN_IF_FAILED(az_json_writer_append_double(&json_builder,e, 3));
    AZ_RETURN_IF_FAILED(az_json_writer_append_property_name(&json_builder, AZ_SPAN_LITERAL_FROM_STR("f")));
    AZ_RETURN_IF_FAILED(az_json_writer_append_double(&json_builder,f, 1));
    AZ_RETURN_IF_FAILED(az_json_writer_append_property_name(&json_builder, AZ_SPAN_LITERAL_FROM_STR("pf")));
    AZ_RETURN_IF_FAILED(az_json_writer_append_double(&json_builder,pf, 2));
    AZ_RETURN_IF_FAILED(az_json_writer_append_end_object(&json_builder));
    const az_span out_payload{az_json_writer_get_bytes_used_in_destination(&json_builder)};

    static int sendCount = 0;
    if (!mqtt_client.publish(telemetry_topic, az_span_ptr(out_payload), az_span_size(out_payload), false))
    {
        DisplayPrintf("ERROR: Send telemetry %d", sendCount);
    }
    else
    {
        ++sendCount;
        DisplayPrintf("Sent telemetry %d", sendCount);
    }

    return AZ_OK;
}

static void HandleCommandMessage(az_span payload, az_iot_hub_client_method_request *command_request)
{
    int command_res_code = 200;
    az_result rc = AZ_OK;

    if (az_span_is_content_equal(AZ_SPAN_LITERAL_FROM_STR(COMMAND_RING_BUZZER), command_request->name))
    {
        // Parse the command payload (it contains a 'duration' field)
        Log("Processing command 'ringBuzzer'" DLM);
        char buffer[32];
        az_span_to_str(buffer, 32, payload);
        Log("Raw command payload: %s" DLM, buffer);

        az_json_reader json_reader;
        uint32_t duration;
        if (az_json_reader_init(&json_reader, payload, NULL) == AZ_OK)
        {
            if (az_json_reader_next_token(&json_reader) == AZ_OK)
            {
                if (az_result_failed(rc = az_json_token_get_uint32(&json_reader.token, &duration)))
                {
                    Log("Couldn't parse JSON token res=%d" DLM, rc);
                }
                else
                {
                    Log("Duration: %dms" DLM, duration);
                }
            }

            // Invoke command
            analogWrite(WIO_BUZZER, 128);
            delay(duration);
            analogWrite(WIO_BUZZER, 0);

            int rc;
            if (az_result_failed(rc = SendCommandResponse(command_request, command_res_code, AZ_SPAN_LITERAL_FROM_STR("{}"))))
            {
                Log("Unable to send %d response, status 0x%08x" DLM, command_res_code, rc);
            }
        }
    }
    else
    {
        // Unsupported command
        Log("Unsupported command received: %.*s." DLM, az_span_size(command_request->name), az_span_ptr(command_request->name));

        int rc;
        if (az_result_failed(rc = SendCommandResponse(command_request, 404, AZ_SPAN_LITERAL_FROM_STR("{}"))))
        {
            printf("Unable to send %d response, status 0x%08x\n", 404, rc);
        }
    }
}

static int SendCommandResponse(az_iot_hub_client_method_request *request, uint16_t status, az_span response)
{
    az_result rc = AZ_OK;
    // Get the response topic to publish the command response
    char commands_response_topic[128];
    if (az_result_failed(rc = az_iot_hub_client_methods_response_get_publish_topic(&HubClient, request->request_id, status, commands_response_topic, sizeof(commands_response_topic), NULL)))
    {
        Log("Unable to get method response publish topic" DLM);
        return rc;
    }

    Log("Status: %u\tPayload: '", status);
    char *payload_char = (char *)az_span_ptr(response);
    if (payload_char != NULL)
    {
        for (int32_t i = 0; i < az_span_size(response); i++)
        {
            Log("%c", *(payload_char + i));
        }
    }
    Log("'" DLM);

    // Send the commands response
    if (mqtt_client.publish(commands_response_topic, az_span_ptr(response), az_span_size(response), false))
    {
        Log("Sent response" DLM);
    }

    return rc;
}

static void MqttSubscribeCallbackHub(char *topic, byte *payload, unsigned int length)
{
    az_span topic_span = az_span_create((uint8_t *)topic, strlen(topic));
    az_iot_hub_client_method_request command_request;

    if (az_result_succeeded(az_iot_hub_client_methods_parse_received_topic(&HubClient, topic_span, &command_request)))
    {
        DisplayPrintf("Command arrived!");
        // Determine if the command is supported and take appropriate actions
        HandleCommandMessage(az_span_create(payload, length), &command_request);
    }

    Log(DLM);
}

////////////////////////////////////////////////////////////////////////////////
// setup and loop

void setup()
{
    ////////////////////
    // Load storage

    Storage::Load();

    ////////////////////
    // Init I/O

    Serial.begin(115200);

    pinMode(WIO_BUZZER, OUTPUT);
    pinMode(WIO_5S_DOWN, INPUT_PULLUP);
    pinMode(WIO_5S_UP, INPUT_PULLUP);
    pinMode(WIO_5S_RIGHT, INPUT_PULLUP);
    pinMode(WIO_5S_LEFT, INPUT_PULLUP);
    pinMode(WIO_5S_PRESS, INPUT_PULLUP);

    ////////////////////
    // Enter configuration mode

    pinMode(WIO_KEY_A, INPUT_PULLUP);
    pinMode(WIO_KEY_B, INPUT_PULLUP);
    pinMode(WIO_KEY_C, INPUT_PULLUP);

    if (digitalRead(WIO_KEY_A) == LOW &&
        digitalRead(WIO_KEY_B) == LOW &&
        digitalRead(WIO_KEY_C) == LOW)
    {
        DisplayPrintf("In configuration mode");
        CliMode();
    }

    ////////////////////
    // Connect Wi-Fi

    DisplayPrintf("Connecting to SSID: %s", IOT_CONFIG_WIFI_SSID);
    do
    {
        Log(".");
        WiFi.begin(IOT_CONFIG_WIFI_SSID, IOT_CONFIG_WIFI_PASSWORD);
        delay(500);
    } while (WiFi.status() != WL_CONNECTED);
    DisplayPrintf("Connected");
    analogWrite(WIO_BUZZER, 128);
    delay(500);
    analogWrite(WIO_BUZZER, 0);


    ////////////////////
    // Sync time server
    ntp.ruleDST("CEST", Last, Sun, Mar, 2, 420); // last sunday in march 2:00, timetone +120min (+1 GMT + 1h summertime offset)
    ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60); // last sunday in october 3:00, timezone +60min (+1 GMT)
    ntp.begin();

    ////////////////////

    // LCD setup
    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(TFT_WHITE);


    //Pzem setup
    pzem.begin(&uart);
    pzem.setAddress(0x42);

    //pzemtimẻ
    startMillisPZEM = millis();

  
    // Provisioning

#if defined(USE_CLI) || defined(USE_DPS)

    if (RegisterDeviceToDPS(IOT_CONFIG_GLOBAL_DEVICE_ENDPOINT, IOT_CONFIG_ID_SCOPE, IOT_CONFIG_REGISTRATION_ID, IOT_CONFIG_SYMMETRIC_KEY, ntp.epoch() + TOKEN_LIFESPAN, &HubHost, &DeviceId) != 0)
    {
        Abort("RegisterDeviceToDPS()");
    }

#else

    HubHost = IOT_CONFIG_IOTHUB;
    DeviceId = IOT_CONFIG_DEVICE_ID;

#endif // USE_CLI || USE_DPS
}

void loop()
{
    //time
    ntp.update();

    //pzem
    readDataPZEM();
  
    if(digitalRead(WIO_KEY_C) == LOW){
        page = page + 1; 
        if(page == 4){
          page = 1;
        }
        delay(500);
    }
  
    if(page == 1){
    Screen1();
    }
    if(page == 2){
    Screen2(); 
    }
    if(page == 3){
        Screen3();
    }
    // if(page == 4){
    // Screen4();
    // }
    //  if(page == 5){
    //    Screen5();
    //  }

    //alarmPower();
    

    static uint64_t reconnectTime;
    if (!mqtt_client.connected())
    {
        DisplayPrintf("Connecting to Azure IoT Hub...");
        const uint64_t now = ntp.epoch();
        if (ConnectToHub(&HubClient, HubHost, DeviceId, IOT_CONFIG_SYMMETRIC_KEY, now + TOKEN_LIFESPAN) != 0)
        {
            DisplayPrintf("> ERROR.");
            Log("> ERROR. Status code =%d. Try again in 5 seconds." DLM, mqtt_client.state());
            delay(5000);
            return;
        }

        DisplayPrintf("> SUCCESS.");

        analogWrite(WIO_BUZZER, 128);
        delay(500);
        analogWrite(WIO_BUZZER, 0);
        delay(100);
        analogWrite(WIO_BUZZER, 128);
        delay(500);
        analogWrite(WIO_BUZZER, 0);

        reconnectTime = now + TOKEN_LIFESPAN * 0.85;
    }
    else
    {
        if ((uint64_t)ntp.epoch() >= reconnectTime)
        {
            DisplayPrintf("Disconnect");
            mqtt_client.disconnect();
            return;
        }

        mqtt_client.loop();

        static unsigned long nextTelemetrySendTime = 0;
        if (millis() > nextTelemetrySendTime)
        {
            SendTelemetry();
            nextTelemetrySendTime = millis() + TELEMETRY_FREQUENCY_MILLISECS;
        }

    }

}

void readDataPZEM(){
    currentMillisPZEM = millis();
    if ((currentMillisPZEM - startMillisPZEM) >= 1000){
        v  = pzem.voltage();
        c  = pzem.current();
        p  = pzem.power();
        e  = pzem.energy();
        f  = pzem.frequency();
        pf = pzem.pf();
        startMillisPZEM = currentMillisPZEM;
    }

}

void Screen1(){

  //time
    spr.createSprite(320, 60);
    spr.fillSprite(TFT_WHITE);

    spr.setTextColor(TFT_BLACK);
    spr.setTextSize(2);
    spr.drawCentreString(ntp.formattedTime("%c"),160, 0, 1);

    spr.pushSprite(0,0);
    spr.deleteSprite();  
  //voltage
  spr.createSprite(320, 60);
  spr.fillSprite(TFT_WHITE);
  spr.setTextColor(TFT_BLACK);
  spr.setTextSize(1);
  spr.drawFloat(v,1,0,0,7);
  spr.pushSprite(0,60);
  spr.deleteSprite();

  //current
  spr.createSprite(320, 60);
  spr.fillSprite(TFT_WHITE);
  spr.setTextColor(TFT_BLACK);
  spr.setTextSize(1);
  spr.drawFloat(c,3,0,0,7);
  spr.pushSprite(0,120);
  spr.deleteSprite();

  //energy
  spr.createSprite(320, 60);
  spr.fillSprite(TFT_WHITE);
  spr.setTextColor(TFT_BLACK);
  spr.setTextSize(1);
  spr.drawFloat(e,3,0,0,7);
  spr.pushSprite(0,180);
  spr.deleteSprite();
}

void Screen2(){
  //power
  spr.createSprite(320, 80);
  spr.fillSprite(TFT_WHITE);
  spr.setTextColor(TFT_BLACK);
  spr.setTextSize(1);
  spr.drawFloat(p,1,0,0,7);
  spr.pushSprite(0,0);
  spr.deleteSprite();

  //frequency
  spr.createSprite(320, 80);
  spr.fillSprite(TFT_WHITE);
  spr.setTextColor(TFT_BLACK);
  spr.setTextSize(1);
  spr.drawFloat(f,1,0,0,7);
  spr.pushSprite(0,80);
  spr.deleteSprite();

  //power factor
  spr.createSprite(320, 80);
  spr.fillSprite(TFT_WHITE);
  spr.setTextColor(TFT_BLACK);
  spr.setTextSize(1);
  spr.drawFloat(pf,2,0,0,7);
  spr.pushSprite(0,160);
  spr.deleteSprite();
  
}

void Screen3(){
    //spr.setTextSize(2);



    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK);

    // ve cac hinh tam giac
    tft.drawTriangle(50,110,75,85,100,110,TFT_RED);
    tft.drawTriangle(135,110,160,85,185,110,TFT_RED);
    tft.drawTriangle(220,110,245,85,270,110,TFT_RED);
    tft.drawTriangle(50,180,75,205,100,180,TFT_RED);
    tft.drawTriangle(135,180,160,205,185,180,TFT_RED);
    tft.drawTriangle(220,180,245,205,270,180,TFT_RED);

    // kiem tra nut phai co bam hay khong?
    if(digitalRead(WIO_5S_RIGHT) == LOW)
    {
      delay(200);
      i++;
      if(i == 3)
      {
        i = 0;
      }
    }
// Kiem tra nut trai co bam hay khong
    if(digitalRead(WIO_5S_LEFT) == LOW)
    {
      delay(200);
      i--;
      if(i < 0)
      {
        i = 2;
      }
    }
// kiem tra nut len co bam hay khong
    if(digitalRead(WIO_5S_UP) == LOW)
    {
      delay(200);
      currentpwd[i] += + 1;
      if(currentpwd[i] == 10)
      {
        currentpwd[i] = 0;
      }
    }
// kiem tra nut xuong co bam hay khong
    if(digitalRead(WIO_5S_DOWN) == LOW)
    {
      delay(200);
      currentpwd[i] = currentpwd[i] - 1;
      if(currentpwd[i] < 0 )
       {
        currentpwd[i] = 9;
      }
    }
// kiem tra nut giua co bam hay khong
    if(digitalRead(WIO_5S_PRESS) == LOW)
    {
      delay(200);
      checkPassword();
    }

//    if(digitalRead(WIO_KEY_B) == LOW)
//    {
//      delay(200);
//      spr.setTextSize(3);
//      spr.setTextColor(TFT_BLACK);
//      spr.drawCentreString("Setup Password", 160, 60,1);
//
//    }

// to mau cho mui ten o 1 chu so cua password
    switch (i)
    {
    case 0:
      tft.fillTriangle(50,110,75,85,100,110,TFT_RED);
      tft.fillTriangle(50,180,75,205,100,180,TFT_RED);
      break;
    case 1:
      tft.fillTriangle(135,110,160,85,185,110,TFT_RED);
      tft.fillTriangle(135,180,160,205,185,180,TFT_RED);
      break;
    case 2:
      tft.fillTriangle(220,110,245,85,270,110,TFT_RED);
      tft.fillTriangle(220,180,245,205,270,180,TFT_RED);
      break;
    }

    //hien thi test_password 
    tft.setTextSize(1);
    tft.setTextColor(TFT_BLACK);
    tft.drawString(String(currentpwd[0]),60,120,7);
    tft.drawString(String(currentpwd[1]),145,120,7);
    tft.drawString(String(currentpwd[2]),225,120,7);


    // spr.drawCentreString("PRESS BUTTON B TO",160,85,1);
    // spr.drawCentreString("RESET ENERGY",160,135,1);
    // spr.drawCentreString("kWh",160,170,1);
    //tft.setTextSize(3);


    //tft.drawFloat(e,3,120,50);


    //Reset Energy
//    if(digitalRead(WIO_KEY_B) == LOW){
//      delay(20);
//      pzem.resetEnergy();
//      
//    }
}

void checkPassword()
{
  for (int j = 0; j < 3; j++)
  {
    if(currentpwd[i] == pwd[i])
    {
      checkpwd += 1;
    }
  }
  if(checkpwd == 3)
  {
    tft.setTextSize(3);
    tft.setTextColor(TFT_BLUE);
    tft.drawCentreString("Right Password", 160, 225,1);
    pzem.resetEnergy();
    
  }
  else 
  {
    tft.setTextSize(3);
    tft.setTextColor(TFT_RED);
    tft.drawCentreString("Wrong Password", 160, 225,1);
  }
  checkpwd = 0;
  
}
