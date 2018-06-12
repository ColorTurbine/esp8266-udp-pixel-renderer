/*
ESPOPC -- Open Pixel Control server for ESP8266.

Notes
- Gamma should be corrected on sender side

*/

#include <Esp.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <NeoPixelBus.h>
#include <ArduinoOTA.h>

// Wifi credentials
const char *ssid = "";
const char *password = "";

// Actual name will be *.local
const char myDNSName[] = "lights";

#define MAX_PIXEL_COUNT (800)
#define COLORS          (4) // RGB (3) or RGBW (4)
#define BufferCount     (6)

const int PixelPin = 2; // Pin # is decided by the chosen pixel output method

#if OSCDEBUG == 1
#define d(x) Serial.println(x)
#define dn(x) Serial.print(x)
#define df(x, ...) Serial.printf(x, __VA_ARGS__)
#else
#define d(x) {}
#define dn(x) {}
#define df(x, ...) {}
#endif

// You can also use one of these for Esp8266, each having their own restrictions
//
// NOTE: These will ignore the PIN and use GPI03 pin
//NeoPixelBus<NeoGrbFeature, NeoEsp8266Dma800KbpsMethod> strip(MAX_PIXEL_COUNT, PixelPin);
//NeoPixelBus<NeoRgbFeature, NeoEsp8266Dma400KbpsMethod> strip(MAX_PIXEL_COUNT, PixelPin);

// Uart method is good for the Esp-01 or other pin restricted modules
// NOTE: These will ignore the PIN and use GPI02 pin

#if COLORS == 4
NeoPixelBus<NeoRgbwFeature, NeoEsp8266Uart800KbpsMethod> strip(MAX_PIXEL_COUNT, PixelPin);
#else
NeoPixelBus<NeoRgbFeature, NeoEsp8266Uart800KbpsMethod> strip(MAX_PIXEL_COUNT, PixelPin);
#endif

// four element pixels, RGBW
//NeoPixelBus<NeoRgbwFeature, Neo800KbpsMethod> strip(MAX_PIXEL_COUNT, PixelPin);

#if COLORS == 4
RgbwColor Buffer[BufferCount][MAX_PIXEL_COUNT];
#else
RgbColor Buffer[BufferCount][MAX_PIXEL_COUNT];
#endif

uint16_t framelen[BufferCount];

uint8_t FrontBuffer = 0;
uint8_t BufferedFrames = 0;
uint8_t BackBuffer = 0;

WiFiClient client;
WiFiUDP Udp;
MDNSResponder mdns;

#define minsize(x, y) (((x) < (y)) ? (x) : (y))

uint32_t usToTicks(uint32_t us)
{
    return (clockCyclesPerMicrosecond() * us);
}

#if 0
bool loadSettings()
{
    SPIFFS.begin();
    File f = SPIFFS.open("foo", "r");
    if (!f)
    {
        return false;
    }
    // Read f
}

void saveSettings(char* hostname)
{
    // TODO
}
#endif

#define LED (1)
void setup()
{
    Serial.begin(256000);
    d();

    // Reset all the neopixels to an off state
    strip.Begin();
    strip.Show();

    // Connect to WiFi network
    d();
    d();
    dn("Connecting to ");
    d(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        dn(".");
    }
    d("");
    d("WiFi connected");

    // Print the IP address
    Serial.println(WiFi.localIP());

    ArduinoOTA.setHostname(myDNSName);
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("OTA: Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA End");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
            Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
            Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
            Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
            Serial.println("End Failed");
    });
    ArduinoOTA.begin();

    // Start the server listening for incoming client connections
    Udp.begin(7890);
    d("UDP server listening on port 7890");

}

#if OSCDEBUG
unsigned long TotalFrames = 0;
#endif

unsigned long previousFrameTime;
uint8_t framecounter = 0;

inline void paint()
{
    if (BufferedFrames == 0)
    {
        return;
    }

    unsigned long mic = micros();
    if ((mic - previousFrameTime) < 36000 && BufferedFrames < 5)
    {
        return;
    }

    // Display current frame (TODO: use memcpy)
    for (int i = 0; i < framelen[FrontBuffer]; i++)
    {
        strip.SetPixelColor(i, Buffer[FrontBuffer][i]);
    }
    strip.Show();

    // Advance front buffer
    BufferedFrames--;
    FrontBuffer++;
    if (FrontBuffer >= BufferCount)
    {
        FrontBuffer = 0;
    }

#if OSCDEBUG
    // Show frame-frame time
    df("F%i U%lu\n", BufferedFrames, mic - previousFrameTime);
#endif
    previousFrameTime = mic;
}

typedef enum
{
    FRAME_NEW,
    FRAME_IN_PROGRESS,
    FRAME_COMPLETE
} frameState_t;

typedef enum
{
    CMD_NEW_FRAME = 0,
    CMD_FRAME_IN_PROGRESS = 1,
    CMD_INVALID = 0xFF
} command_t;

typedef enum
{
    FORMAT_RAW = 0,
    FORMAT_RLE = 1,
    FORMAT_INVALID = 0xFF
} format_t;

frameState_t FrameState;
uint16_t frame_pixelcount;

#define HEADER_LEN (6)
#define MAX_PACKET_LEN (MAX_PIXEL_COUNT * COLORS + HEADER_LEN)

void parsePacket(uint8_t *udp_packet_buffer, uint16_t len, format_t format, uint16_t currentPixel)
{
    uint16_t current_byte = HEADER_LEN;

    // Drop frames if we've overflowed
    if (BufferedFrames >= BufferCount)
    {
        d("E: Overflow");
        FrameState = FRAME_COMPLETE;
        return;
    }
    
    FrameState = FRAME_IN_PROGRESS;

    switch (format)
    {
    case FORMAT_RAW:
        // Pixel data
        while (current_byte < len &&
            currentPixel < frame_pixelcount &&
            currentPixel < MAX_PIXEL_COUNT) {
            uint8_t g = udp_packet_buffer[current_byte++];
            uint8_t r = udp_packet_buffer[current_byte++];
            uint8_t b = udp_packet_buffer[current_byte++];

    #if COLORS == 4
            uint8_t w = udp_packet_buffer[current_byte++];
            Buffer[BackBuffer][currentPixel] = RgbwColor(r, g, b, w);
    #else
            Buffer[BackBuffer][currentPixel] = RgbColor(r, g, b);
    #endif
            currentPixel++;
        }
    break;
    case FORMAT_RLE: // Untested
        // RLE encoded pixel data
        while (current_byte < len &&
                currentPixel < frame_pixelcount &&
                currentPixel < MAX_PIXEL_COUNT)
        {
            uint8_t count = udp_packet_buffer[current_byte++];

            uint8_t g = udp_packet_buffer[current_byte++];
            uint8_t r = udp_packet_buffer[current_byte++];
            uint8_t b = udp_packet_buffer[current_byte++];

#if COLORS == 4
            uint8_t w = udp_packet_buffer[current_byte++];
#endif

            while (count-- > 0 && current_byte < len && currentPixel < MAX_PIXEL_COUNT)
            {
#if COLORS == 4
            Buffer[BackBuffer][currentPixel] = RgbwColor(r, g, b, w);
#else
            Buffer[BackBuffer][currentPixel] = RgbColor(r, g, b);
#endif
            currentPixel++;
            }
        }
        break;

    case FORMAT_INVALID:
    default:
        break;
    }

    if (currentPixel == frame_pixelcount ||
        currentPixel == MAX_PIXEL_COUNT)
    {
        if (currentPixel == MAX_PIXEL_COUNT)
        {
            d("W: Truncating frame");
        }
        df("T: Frame complete @ %d", currentPixel);

        BufferedFrames++;
        FrameState = FRAME_COMPLETE;
        framelen[BackBuffer] = frame_pixelcount;

        // Advance back buffer
        BackBuffer++;
        if (BackBuffer >= BufferCount)
        {
            BackBuffer = 0;
        }

#if OSCDEBUG
        TotalFrames++;
#endif
    }
}

uint8_t burnLoop = 0;

void loop()
{
    static uint8_t udp_packet_buffer[MAX_PACKET_LEN];
    uint16_t currentPixel;

    paint();
   
    int packetSize = Udp.parsePacket();
    if(packetSize == 0)
    {
        ArduinoOTA.handle();
        return;
    }

    df("Received %d bytes\n", packetSize);

    int len = Udp.read(udp_packet_buffer, MAX_PACKET_LEN);

    // 0: command (0 - new frame, 1 - continuation)
    // 1: format (0 - raw, 1 - RLE)
    // 2-3: unused
    // 4-5: total pixels (for new frame), starting pixel (for existing frame)

    // Parse header
    command_t pktCommand = (command_t)udp_packet_buffer[0];
    format_t pktFormat = (format_t)udp_packet_buffer[1];
    // [2], [3]: unused

    switch (pktCommand)
    {
        case CMD_NEW_FRAME:
            if (FrameState != FRAME_COMPLETE)
            {
                d("W: Overwriting frame in progress");
            }
            FrameState = FRAME_NEW;
            currentPixel = 0;
            frame_pixelcount = (uint16_t)udp_packet_buffer[4] << 8 | udp_packet_buffer[5];

            df("CMD_NEW_FRAME: %d\n", frame_pixelcount);
            parsePacket(udp_packet_buffer, len, pktFormat, currentPixel);
            break;
        case CMD_FRAME_IN_PROGRESS:
            if(FrameState != FRAME_IN_PROGRESS)
            {
                d("E: No frame in progress");
                return;
            }
            currentPixel = (uint16_t)udp_packet_buffer[4] << 8 | udp_packet_buffer[5];

            df("CMD_FRAME_IN_PROGRESS: %d\n", currentPixel);
            parsePacket(udp_packet_buffer, len, pktFormat, currentPixel);
            break;
        default:
            d("E: Unknown command");
            return;
    }

#if OSCDEBUG
    df("T: %lu, t: %d, H: %d\n", TotalFrames, BufferedFrames, ESP.getFreeHeap());
#endif
}
