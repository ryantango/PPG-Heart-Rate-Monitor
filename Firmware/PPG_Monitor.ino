#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =====================================================
// OLED DISPLAY CONFIGURATION
// =====================================================

// OLED screen resolution
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// -1 means the OLED does not use a separate reset pin
#define OLED_RESET -1

// I2C scanner determined that this OLED uses address 0x3D
#define OLED_ADDRESS 0x3D

// Create the SSD1306 display object
Adafruit_SSD1306 display(SCREEN_WIDTH,SCREEN_HEIGHT,&Wire,OLED_RESET);

// =====================================================
// PPG INPUT
// =====================================================

// Analog output of the PPG conditioning circuit
// is connected to Arduino analog input A0
#define PPG_PIN A0

// Change to true if heartbeat pulses appear inverted
const bool INVERT_SIGNAL = false;

// =====================================================
// SAMPLING
// =====================================================

// Sample the PPG signal every 20 ms
// 1000 ms / 20 ms = 50 samples per second
const unsigned long SAMPLE_INTERVAL = 20;

// Refresh the OLED 5 times per second
// Slower display updating prevents it from interfering
// with continuous PPG sampling
const unsigned long DISPLAY_INTERVAL = 200;

unsigned long previousSampleTime = 0;
unsigned long previousDisplayTime = 0;

// =====================================================
// SIGNAL VALUES
// =====================================================

// Direct ADC value measured from A0
int rawADC = 0;

// Running estimate of the DC baseline of the PPG signal
float baseline = 512.0;

// Signal after baseline/DC removal
float highPassSignal = 0.0;

// Signal after digital low-pass smoothing
float filteredSignal = 0.0;

// Controls how quickly the estimated baseline follows
// slow changes in finger pressure or ambient light
const float BASELINE_ALPHA = 0.01;

// Controls the amount of digital smoothing
// Lower values create stronger smoothing
const float FILTER_ALPHA = 0.25;

// =====================================================
// RAW SIGNAL THRESHOLD
// =====================================================

// Track the minimum and maximum raw signal values
// over each threshold measurement window
float rawMin = 1000;
float rawMax = -1000;

// Upper threshold detects the beginning of a pulse
// Lower threshold resets the detector afterward
float rawUpperThreshold = 8;
float rawLowerThreshold = 4;

// =====================================================
// FILTERED SIGNAL THRESHOLD
// =====================================================

// Track minimum and maximum filtered signal values
float filtMin = 1000;
float filtMax = -1000;

float filtUpperThreshold = 8;
float filtLowerThreshold = 4;

// Used to determine when adaptive thresholds
// should be recalculated
unsigned long thresholdTimer = 0;

// Recalculate detection thresholds every 2 seconds
const unsigned long THRESHOLD_INTERVAL = 2000;

// Ignore very small fluctuations that are likely noise
const float MIN_SIGNAL_RANGE = 5.0;

// =====================================================
// RAW BPM DETECTION
// =====================================================

// Tracks whether the raw signal is currently
// above the heartbeat detection threshold
bool rawAboveThreshold = false;

// Time of the most recently detected raw beat
unsigned long rawLastBeatTime = 0;

// BPM calculated from the unfiltered signal
int rawBPM = 0;

// =====================================================
// FILTERED BPM DETECTION
// =====================================================

bool filtAboveThreshold = false;

// Time of the most recent filtered heartbeat
unsigned long filtLastBeatTime = 0;

// BPM calculated from the digitally filtered PPG signal
int filteredBPM = 0;

// =====================================================
// BPM LIMITS
// =====================================================

// Ignore additional peaks for 300 ms after a beat
// to prevent one heartbeat from being counted multiple times
const unsigned long REFRACTORY_PERIOD = 300;

// 2000 ms between beats corresponds to 30 BPM
const unsigned long MAX_BEAT_INTERVAL = 2000;

// 300 ms between beats corresponds to 200 BPM
const unsigned long MIN_BEAT_INTERVAL = 300;

// =====================================================
// FILTERED BPM AVERAGING
// =====================================================

// Store several recent filtered BPM values so the final
// displayed BPM is less sensitive to beat-to-beat variation
const byte BPM_BUFFER_SIZE = 4;

int bpmBuffer[BPM_BUFFER_SIZE] = {
  0, 0, 0, 0
};

byte bpmIndex = 0;
byte bpmCount = 0;

// =====================================================
// SETUP
// =====================================================

void setup() {

  // Serial output can be viewed in Serial Monitor
  // or Arduino Serial Plotter for signal analysis
  Serial.begin(115200);

  pinMode(PPG_PIN, INPUT);

  // Start I2C communication with the OLED
  Wire.begin();

  // Initialize OLED using its detected I2C address
  if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS)) {

    Serial.println("OLED failed");

    // Stop execution if the OLED cannot initialize
    while (true);
  }

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(20, 20);
  display.println("PPG FILTER TEST");

  display.setCursor(20, 35);
  display.println("Initializing");

  display.display();

  // ===================================================
  // CALCULATE STARTING BASELINE
  // ===================================================

  // Average the first 100 ADC measurements to estimate
  // the initial DC voltage level of the PPG signal
  long sum = 0;

  for (int i = 0; i < 100; i++) {

    sum += analogRead(PPG_PIN);

    delay(10);
  }

  baseline = sum / 100.0;

  // Begin timing the first adaptive-threshold window
  thresholdTimer = millis();

  delay(500);
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  unsigned long currentTime = millis();

  // Read and process the PPG signal every 20 ms
  if (currentTime - previousSampleTime >= SAMPLE_INTERVAL) {

    previousSampleTime = currentTime;

    processPPG(currentTime);
  }

  // Update the OLED at a slower rate than signal sampling
  if (currentTime - previousDisplayTime >= DISPLAY_INTERVAL) {

    previousDisplayTime = currentTime;

    updateDisplay();
  }
}

// =====================================================
// PROCESS PPG
// =====================================================

void processPPG(unsigned long currentTime) {

  // ---------------------------------------------------
  // 1. RAW ADC
  // ---------------------------------------------------

  // Read the voltage from the analog PPG front end
  // Arduino converts the voltage into a digital ADC value
  rawADC = analogRead(PPG_PIN);

  // ---------------------------------------------------
  // 2. ESTIMATE DC BASELINE
  // ---------------------------------------------------

  // Slowly track the large DC component caused by
  // average tissue absorption, ambient light,
  // and changes in finger pressure
  baseline +=
    BASELINE_ALPHA *
    (rawADC - baseline);

  // ---------------------------------------------------
  // 3. REMOVE BASELINE
  //
  // Similar to high-pass filtering
  // ---------------------------------------------------

  // Subtract the estimated DC component so that
  // the smaller heartbeat-related AC component remains
  highPassSignal =
    rawADC - baseline;

  // ---------------------------------------------------
  // 4. LOW-PASS / SMOOTH SIGNAL
  // ---------------------------------------------------

  // First-order digital low-pass filter
  // reduces rapid noise while preserving slower pulse changes
  filteredSignal +=
    FILTER_ALPHA *
    (highPassSignal - filteredSignal);

  // Reverse the signal if the physical circuit produces
  // downward-going heartbeat peaks
  if (INVERT_SIGNAL) {

    highPassSignal = -highPassSignal;
    filteredSignal = -filteredSignal;
  }

  // ---------------------------------------------------
  // TRACK RAW MIN/MAX
  // ---------------------------------------------------

  // Determine the raw signal amplitude during
  // the current threshold measurement window
  if (highPassSignal > rawMax) {
    rawMax = highPassSignal;
  }

  if (highPassSignal < rawMin) {
    rawMin = highPassSignal;
  }

  // ---------------------------------------------------
  // TRACK FILTERED MIN/MAX
  // ---------------------------------------------------

  if (filteredSignal > filtMax) {
    filtMax = filteredSignal;
  }

  if (filteredSignal < filtMin) {
    filtMin = filteredSignal;
  }

  // ---------------------------------------------------
  // UPDATE ADAPTIVE THRESHOLDS
  // ---------------------------------------------------

  // Recalculate heartbeat detection thresholds based
  // on the actual signal amplitude every 2 seconds
  if (currentTime - thresholdTimer >=
      THRESHOLD_INTERVAL) {

    float rawRange =
      rawMax - rawMin;

    float filtRange =
      filtMax - filtMin;

    // RAW threshold
    if (rawRange >= MIN_SIGNAL_RANGE) {

      // Detect a pulse once the raw signal rises
      // above 65% of its recent amplitude range
      rawUpperThreshold =
        rawMin + 0.65 * rawRange;

      // Lower threshold provides hysteresis so
      // one heartbeat is not counted repeatedly
      rawLowerThreshold =
        rawMin + 0.40 * rawRange;
    }

    // FILTERED threshold
    if (filtRange >= MIN_SIGNAL_RANGE) {

      filtUpperThreshold =
        filtMin + 0.65 * filtRange;

      filtLowerThreshold =
        filtMin + 0.40 * filtRange;
    }

    // Reset min/max values for the next measurement window
    rawMin = highPassSignal;
    rawMax = highPassSignal;

    filtMin = filteredSignal;
    filtMax = filteredSignal;

    thresholdTimer = currentTime;
  }

  // ---------------------------------------------------
  // RAW BPM DETECTION
  // ---------------------------------------------------

  // Calculate heart rate using the signal before
  // digital low-pass smoothing
  detectRawBeat(currentTime);

  // ---------------------------------------------------
  // FILTERED BPM DETECTION
  // ---------------------------------------------------

  // Calculate heart rate using the filtered signal
  // so raw vs. filtered performance can be compared
  detectFilteredBeat(currentTime);

  // ---------------------------------------------------
  // SERIAL OUTPUT
  //
  // Useful for Serial Plotter or saving to Python/MATLAB
  // ---------------------------------------------------

  // Output each processing stage for later analysis:
  // RAW ADC, baseline-removed signal, filtered signal,
  // thresholds, raw BPM, and filtered BPM
  Serial.print(rawADC);
  Serial.print(",");

  Serial.print(highPassSignal);
  Serial.print(",");

  Serial.print(filteredSignal);
  Serial.print(",");

  Serial.print(rawUpperThreshold);
  Serial.print(",");

  Serial.print(filtUpperThreshold);
  Serial.print(",");

  Serial.print(rawBPM);
  Serial.print(",");

  Serial.println(filteredBPM);
}

// =====================================================
// RAW BEAT DETECTOR
// =====================================================

void detectRawBeat(unsigned long currentTime) {

  // Determine whether enough time has passed since the
  // previous heartbeat to accept another detection
  bool refractoryFinished =
    currentTime - rawLastBeatTime >
    REFRACTORY_PERIOD;

  // Detect a heartbeat when the signal crosses
  // the adaptive upper threshold
  if (!rawAboveThreshold &&
      highPassSignal >= rawUpperThreshold &&
      refractoryFinished) {

    rawAboveThreshold = true;

    // Two detected beats are needed before
    // beat-to-beat interval can be calculated
    if (rawLastBeatTime != 0) {

      unsigned long interval =
        currentTime - rawLastBeatTime;

      // Reject intervals outside the expected
      // physiological heart-rate range
      if (interval >= MIN_BEAT_INTERVAL &&
          interval <= MAX_BEAT_INTERVAL) {

        // BPM = 60,000 ms / milliseconds between beats
        rawBPM =
          60000UL / interval;
      }
    }

    // Save timestamp of this heartbeat
    rawLastBeatTime = currentTime;
  }

  // Signal must fall below the lower threshold before
  // another heartbeat can be detected
  if (rawAboveThreshold &&
      highPassSignal <= rawLowerThreshold) {

    rawAboveThreshold = false;
  }
}

// =====================================================
// FILTERED BEAT DETECTOR
// =====================================================

void detectFilteredBeat(unsigned long currentTime) {

  bool refractoryFinished =
    currentTime - filtLastBeatTime >
    REFRACTORY_PERIOD;

  // Detect a heartbeat from the digitally filtered signal
  if (!filtAboveThreshold &&
      filteredSignal >= filtUpperThreshold &&
      refractoryFinished) {

    filtAboveThreshold = true;

    if (filtLastBeatTime != 0) {

      unsigned long interval =
        currentTime - filtLastBeatTime;

      if (interval >= MIN_BEAT_INTERVAL &&
          interval <= MAX_BEAT_INTERVAL) {

        // Calculate instantaneous BPM from
        // the current beat-to-beat interval
        int instantaneousBPM =
          60000UL / interval;

        // Average several BPM measurements
        // to create a more stable final value
        filteredBPM =
          averageBPM(instantaneousBPM);
      }
    }

    filtLastBeatTime = currentTime;
  }

  // Hysteresis prevents repeated detection
  // while the signal remains near its peak
  if (filtAboveThreshold &&
      filteredSignal <= filtLowerThreshold) {

    filtAboveThreshold = false;
  }
}

// =====================================================
// BPM AVERAGING
// =====================================================

int averageBPM(int newBPM) {

  // Add newest BPM measurement to circular buffer
  bpmBuffer[bpmIndex] = newBPM;

  bpmIndex++;

  // Return to the beginning when the buffer is full
  if (bpmIndex >= BPM_BUFFER_SIZE) {
    bpmIndex = 0;
  }

  // Track how many valid values are currently stored
  if (bpmCount < BPM_BUFFER_SIZE) {
    bpmCount++;
  }

  long sum = 0;

  // Average recent BPM measurements
  // to reduce beat-to-beat fluctuations
  for (byte i = 0; i < bpmCount; i++) {

    sum += bpmBuffer[i];
  }

  return sum / bpmCount;
}

// =====================================================
// OLED DISPLAY
// =====================================================

void updateDisplay() {

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // RAW = direct ADC measurement from PPG circuit
  display.setCursor(0, 0);
  display.print("RAW: ");
  display.print(rawADC);

  // HP = signal after DC/baseline removal
  display.setCursor(0, 10);
  display.print("HP:  ");
  display.print(highPassSignal, 1);

  // FILT = signal after digital low-pass smoothing
  display.setCursor(0, 20);
  display.print("FILT:");
  display.print(filteredSignal, 1);

  // THR = adaptive threshold used for filtered beat detection
  display.setCursor(0, 30);
  display.print("THR: ");
  display.print(filtUpperThreshold, 1);

  // Heart rate calculated before digital smoothing
  display.setCursor(0, 42);
  display.print("RAW BPM: ");
  
  if (rawBPM > 0) {
    display.print(rawBPM);
  }
  else {
    display.print("--");
  }

  // Heart rate calculated after digital filtering
  // and BPM averaging
  display.setCursor(0, 54);
  display.print("FLT BPM: ");

  if (filteredBPM > 0) {
    display.print(filteredBPM);
  }
  else {
    display.print("--");
  }

  // Send the completed display buffer to the OLED
  display.display();
}
