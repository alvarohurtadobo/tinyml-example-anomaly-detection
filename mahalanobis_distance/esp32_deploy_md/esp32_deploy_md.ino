/**
 * Use Mahalanobis Distance on real temperature data (MAX6675) to see if it's an anomaly
 * 
 * Author: Shawn Hymel
 * Date: May 6, 2020
 * Modified: Adapted for MAX6675 temperature sensor
 * 
 * License: Beerware
 */

// Library includes
// Note: Using manual SPI implementation for MAX6675 instead of SPI library

// Local includes
#include "md_model-moving.h"

extern "C" {
#include "utils.h"
};

// Set to 1 to output debug info to Serial, 0 otherwise
#define DEBUG 1

// Pins for MAX6675
constexpr int TEMP_CS_PIN = 5;        // Chip Select pin for MAX6675
constexpr int TEMP_SCK_PIN = 18;      // Clock pin for MAX6675 (SPI SCK)
constexpr int TEMP_SO_PIN = 19;       // Serial Out pin for MAX6675 (SPI MISO)
constexpr int BUZZER_PIN = A1;

// Settings
constexpr int NUM_AXES = 1;           // Temperature sensor (1D data)
constexpr int MAX_MEASUREMENTS = 128; // Number of samples to keep
constexpr float MAD_SCALE = 1.4826;   // Scale MAD to be inline with SciPy calcs
constexpr float THRESHOLD = 1.0;     // Any MD over this is an anomaly
constexpr int WAIT_TIME = 1000;       // ms between sample sets
constexpr int SAMPLE_RATE = 200;      // How fast to collect measurements (Hz)

/*******************************************************************************
 * Functions
 */

// Read temperature from MAX6675 sensor
float readMAX6675() {
  uint16_t data = 0;
  
  // Pull CS low to start communication
  digitalWrite(TEMP_CS_PIN, LOW);
  delayMicroseconds(100); // Small delay for stability (MAX6675 requires ~100us)
  
  // Read 16 bits (MSB first)
  for (int i = 15; i >= 0; i--) {
    // Clock starts low, then goes high
    digitalWrite(TEMP_SCK_PIN, LOW);
    delayMicroseconds(1);
    digitalWrite(TEMP_SCK_PIN, HIGH);
    delayMicroseconds(1);
    
    // Read bit after clock goes high
    if (digitalRead(TEMP_SO_PIN)) {
      data |= (1 << i);
    }
  }
  
  // Ensure clock is low after reading
  digitalWrite(TEMP_SCK_PIN, LOW);
  
  // Pull CS high to end communication
  digitalWrite(TEMP_CS_PIN, HIGH);
  
  // Check for open circuit (bit 2)
  if (data & 0x04) {
#if DEBUG
    Serial.println("MAX6675: Open circuit detected!");
#endif
    return -1.0; // Error value
  }
  
  // Extract temperature (bits 15-3)
  // Temperature is in 0.25°C increments
  float temp = (data >> 3) * 0.25;
  
  return temp;
}

/*******************************************************************************
 * Main
 */
void setup() {

  // Initialize Serial port for debugging
#if DEBUG
  Serial.begin(115200);
  while (!Serial);
#endif

  // Initialize MAX6675 pins
  pinMode(TEMP_CS_PIN, OUTPUT);
  pinMode(TEMP_SCK_PIN, OUTPUT);
  pinMode(TEMP_SO_PIN, INPUT);
  
  // Set CS high and SCK low initially
  digitalWrite(TEMP_CS_PIN, HIGH);
  digitalWrite(TEMP_SCK_PIN, LOW);
  
#if DEBUG
  Serial.println("MAX6675 temperature sensor initialized");
#endif

  // Configure buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {

  float sample[MAX_MEASUREMENTS];
  float measurements[MAX_MEASUREMENTS];
  float mad[NUM_AXES];
  float mahal;
  float temperature;

  // Timestamps for collecting samples
  static unsigned long timestamp = millis();
  static unsigned long prev_timestamp = timestamp;

  // Take a given time worth of measurements
  int i = 0;
  while (i < MAX_MEASUREMENTS) {
    if (millis() >= timestamp + (1000 / SAMPLE_RATE)) {
  
      // Update timestamps to maintain sample rate
      prev_timestamp = timestamp;
      timestamp = millis();

      // Read temperature from MAX6675
      temperature = readMAX6675();

      // Add temperature reading to array
      sample[i] = temperature;

      // Update sample counter
      i++;
    }
  }

  // Compute the MAD for temperature (scale up by 1.4826)
  for (int i = 0; i < MAX_MEASUREMENTS; i++) {
    measurements[i] = sample[i];
  }
  mad[0] = MAD_SCALE * calc_mad(measurements, MAX_MEASUREMENTS);

  // Print out MAD calculation
#if DEBUG
  Serial.print("Temperature MAD: ");
  Serial.println(mad[0], 7);
#endif

  // Calculate Mahalanobis distance of temperature sample
  mahal = mahalanobis(mad, model_mu, *model_inv_cov, model_mu_dim1);
#if DEBUG
  Serial.print("Mahalanobis distance: ");
  Serial.println(mahal, 7);
#endif

  // Compare to threshold
  if (mahal > THRESHOLD) {
    digitalWrite(BUZZER_PIN, HIGH);
#if DEBUG
    Serial.println("DANGER!!!");
#endif
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
#if DEBUG
  Serial.println();
#endif

  delay(WAIT_TIME);
}
