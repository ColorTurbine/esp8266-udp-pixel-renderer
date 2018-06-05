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
const char myDNSName[] = "lights";

#define MAX_PIXEL_COUNT 480 // Do I have an off by one error somewhere here? (Ya, probably)
#define COLORS 4            // RGB (3) or RGBW (4)
#define BufferCount (12)

const int PixelPin = 2; // Pin # is decided by the chosen pixel output method

#define OSCDEBUG (0)

#if OSCDEBUG
#define d(x) Serial.println(x)
#define dn(x) Serial.print(x)
#define df(x, ...) Serial.printf(x, __VA_ARGS__)
#else
#define d(x)
#define dn(x)
#define df(x, ...)
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
        FrontBuffer = 0;

#if OSCDEBUG
    d(BufferedFrames);
    // Show frame-frame time (probably shouldn't print print here)
    df("%lu\r\n", micros() - previousFrameTime);
    previousFrameTime = micros();
    TotalFrames++;
#endif
}

#define MAX_PACKET_LEN (MAX_PIXEL_COUNT * COLORS + 4 + 100)
void loop()
{
    static int packetParseState = 0;
    static uint8_t pktChannel, pktCommand;
    static uint16_t packet_pixel_count;
    static uint8_t udp_packet_buffer[MAX_PACKET_LEN];
    static uint16_t currentPixel;
    uint16_t bytesRead;

    int packetSize = Udp.parsePacket();
    if (!packetSize) {
        return;
    }

    d(packetSize);
    d(" ");
    df("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(udp_packet_buffer, MAX_PACKET_LEN);

    int current_byte = 0;

    // Start of a new packet
    if (packetParseState == 0) {
        pktChannel = udp_packet_buffer[current_byte++];
        pktCommand = udp_packet_buffer[current_byte++];
        packet_pixel_count = ((uint16_t)udp_packet_buffer[current_byte++] << 8 | udp_packet_buffer[current_byte++]) / COLORS;
        currentPixel = 0;

        if ((pktCommand == 0) && (pktChannel <= 1)) {
            d("N");
            packetParseState = 1;

            // Don't allow buffer to overflow; don't drop frames
            // int safety = 0;
            // while (BufferedFrames >= BufferCount - 1) // (need 1 buffer for the front buffer)
            // {
            //     delay(0);
            //     if (safety++ > 100000) {
            //         d("Trig.");
            //         break;
            //     }
            // }

            // Drop frames if we've overflowed
            if (BufferedFrames >= BufferCount - 1) { // (need 1 buffer for the front buffer)
                return;
            }

            // Advance back buffer
            noInterrupts();
            BackBuffer++;
            if (BackBuffer >= BufferCount) {
                BackBuffer = 0;
            }
            interrupts();
        }
    }

    // Pixel data
    if (packetParseState == 1) {
        while (current_byte < len &&
               currentPixel < MAX_PIXEL_COUNT &&
               currentPixel < packet_pixel_count) {
            uint8_t g = udp_packet_buffer[current_byte++]; //GammaLUT[*pixrgbw++];
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

        if (currentPixel == packet_pixel_count || currentPixel == MAX_PIXEL_COUNT) {
            packetParseState = 2;
        }
    }

    // Done getting pixel data
    if (packetParseState == 2) {
        noInterrupts();
        BufferedFrames++;
        // Dynamically adjust frame timer
        // if(BufferedFrames > (High + Low) * (3/4)) {
        //     timer_ticks -= one_ms/100; // Overrun, play faster
        // } else if(BufferedFrames < (High + Low) * (1/4)) {
        //     timer_ticks += one_ms/100; // Running low, play slower
        // }
        interrupts();
        packetParseState = 0;
        d("D");
    }

    d('.');
#if OSCDEBUG
    dn(TotalFrames);
    dn(':');
#endif
    dn(timer_ticks);
    dn(':');
    dn(BufferedFrames);
    dn(':');
    d(ESP.getFreeHeap());
}
