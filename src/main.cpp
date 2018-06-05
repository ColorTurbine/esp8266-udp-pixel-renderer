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

// Wifi credentials
const char *ssid = "";
const char *password = "";

// Actual name will be *.local
const char myDNSName[] = "devlights";

#define MAX_PIXEL_COUNT (480) // Do I have an off by one error somewhere here? (Ya, probably)
#define COLORS          (3)   // RGB (3) or RGBW (4)
#define BufferCount     (8)

const int PixelPin = 2; // Pin # is decided by the chosen pixel output method

#define OSCDEBUG 0

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

volatile uint8_t FrontBuffer = 0;
volatile uint8_t BufferedFrames = 0;
uint8_t BackBuffer = 0;
static void frame_timer_interrupt() ICACHE_RAM_ATTR;
int timer_ticks = 0;
int one_ms;
volatile bool ready = false;
WiFiClient client;
WiFiUDP Udp;
MDNSResponder mdns;

#define minsize(x, y) (((x) < (y)) ? (x) : (y))

uint32_t usToTicks(uint32_t us)
{
    return (clockCyclesPerMicrosecond() * us);
}

#define LED (1)
void setup()
{
	#if OSCDEBUG
    Serial.begin(115200);
	#else
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	#endif
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
    d(WiFi.localIP());

    // Set up mDNS responder:
    if (!mdns.begin(myDNSName, WiFi.localIP())) {
        d("Error setting up MDNS responder!");
    } else {
        d("mDNS responder started");
        df("My name is [%s]\r\n", myDNSName);
    }

    // Start the server listening for incoming client connections
    Udp.begin(7890);
    d("UDP server listening on port 7890");

    // ESP.wdtDisable();

    one_ms = usToTicks(1000);

    // Setup 30FPS timer
    noInterrupts();
    timer0_isr_init();
    timer0_attachInterrupt(frame_timer_interrupt);
    timer_ticks = usToTicks(1000000 / 30); // FPS
    timer0_write(ESP.getCycleCount() + timer_ticks);
    interrupts();
}

#if OSCDEBUG
unsigned long previousFrameTime;
unsigned long TotalFrames = 0;
#endif
// TODO: Just set a flag here
static void frame_timer_interrupt()
{
    // Set an interrupt for 1/30s from now
    timer0_write(ESP.getCycleCount() + timer_ticks);

    // No frame to show, come again soon
    if (BufferedFrames == 0) {
        return;
    }

    // We're about to overlap the (active) back buffer
    if (FrontBuffer == BackBuffer) {
        return;
    }

    // Display current frame (TODO: use memcpy)
    for (int i = 0; i < MAX_PIXEL_COUNT; i++) {
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
    // Show frame-frame time (probably shouldn't print print here)
    df("BF: %d, us: %lu\r\n", BufferedFrames, micros() - previousFrameTime);
    previousFrameTime = micros();
#endif
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
    FORMAT_INVALID = 0xFF
} format_t;

frameState_t FrameState;
uint16_t frame_pixelcount;

#define HEADER_LEN (6)
#define MAX_PACKET_LEN (MAX_PIXEL_COUNT * COLORS + HEADER_LEN + 100)

void parsePacket(uint8_t *udp_packet_buffer, uint16_t len, uint16_t currentPixel)
{
    uint16_t current_byte = HEADER_LEN;

    // Drop frames if we've overflowed
    if (BufferedFrames >= BufferCount - 1)
    { // (need 1 buffer for the front buffer)
        d("E: Overflow");
        FrameState = FRAME_COMPLETE;
        return;
    }

    if(FrameState == FRAME_NEW)
    {
        // Advance back buffer
        noInterrupts();
        BackBuffer++;
        if (BackBuffer >= BufferCount)
        {
            BackBuffer = 0;
        }
        interrupts();
    }
    
    FrameState = FRAME_IN_PROGRESS;

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

    if (currentPixel == frame_pixelcount ||
        currentPixel == MAX_PIXEL_COUNT)
    {
        if(currentPixel == MAX_PIXEL_COUNT)
        {
            d("W: Truncating frame");
        }
        df("T: Frame complete @ %d", currentPixel);

        noInterrupts();
        BufferedFrames++;
        // Dynamically adjust frame timer
        // if(BufferedFrames > (High + Low) * (3/4)) {
        //     timer_ticks -= one_ms/100; // Overrun, play faster
        // } else if(BufferedFrames < (High + Low) * (1/4)) {
        //     timer_ticks += one_ms/100; // Running low, play slower
        // }
        interrupts();
        FrameState = FRAME_COMPLETE;
#if OSCDEBUG
        TotalFrames++;
#endif
    }
}

void loop()
{
    static uint8_t udp_packet_buffer[MAX_PACKET_LEN];
    uint16_t currentPixel;

    int packetSize = 0;
    while(packetSize == 0)
    {
        packetSize = Udp.parsePacket();
        delay(1);
    }

    df("Received %d bytes\n", packetSize);

    int len = Udp.read(udp_packet_buffer, MAX_PACKET_LEN);

    // 0: command (0 - new frame, 1 - continuation)
    // 1: format (0 - raw)
    // 2-3: unused
    // 4-5: total pixels (for new frame), starting pixel (for existing frame)


    // Parse header
    command_t pktCommand = (command_t)udp_packet_buffer[0];
    format_t pktFormat = (format_t)udp_packet_buffer[1];
    // [2], [3]: unused

    if (pktFormat != FORMAT_RAW)
    {
        d("E: Invalid packet format");
        return;
    }

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
            parsePacket(udp_packet_buffer, len, currentPixel);
            break;
        case CMD_FRAME_IN_PROGRESS:
            if(FrameState != FRAME_IN_PROGRESS)
            {
                d("E: No frame in progress");
                return;
            }
            currentPixel = (uint16_t)udp_packet_buffer[4] << 8 | udp_packet_buffer[5];

            df("CMD_FRAME_IN_PROGRESS: %d\n", currentPixel);
            parsePacket(udp_packet_buffer, len, currentPixel);
        break;
        default:
            d("E: Unknown command");
            return;
    }

#if OSCDEBUG
    df("T: %lu, t: %d, f: %d, H: %d\n", TotalFrames, timer_ticks, BufferedFrames, ESP.getFreeHeap());
#endif
}
