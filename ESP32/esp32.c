#include <Arduino.h>

// UART Configuration
#define UART_NUM UART_NUM_0  // Using UART0
#define RX_PIN 36            // ESP32 RX pin (as per your schematic)
#define TX_PIN 43            // ESP32 TX pin (for ACK/NACK)
#define BAUD_RATE 115200

// Protocol Constants
#define START_BYTE 0xAA
#define END_BYTE 0x55
#define ACK_BYTE 0x06
#define NACK_BYTE 0x15

// Command IDs
#define CMD_SET_PAN 0x01
#define CMD_SET_TILT 0x02
#define CMD_MOVE_COMBINED 0x03
#define CMD_CALIBRATE 0x04
#define CMD_STOP 0x05

// Motor control (placeholder - integrate with your PCA9685)
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channels (adjust based on your wiring)
#define PAN_SERVO_CHANNEL 0
#define TILT_SERVO_CHANNEL 1

// Servo pulse width limits (µs)
#define SERVO_MIN 500
#define SERVO_MAX 2500
#define SERVO_CENTER 1500

// Current positions (in centidegrees)
int16_t current_pan = 0;
int16_t current_tilt = 0;

// Message buffer
#define MAX_PAYLOAD 32
uint8_t rx_buffer[MAX_PAYLOAD + 6];  // Start + CMD + LEN + Payload + Checksum + End
uint8_t rx_index = 0;

void setup() {
  Serial.begin(BAUD_RATE);  // USB debug
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);  // UART to Pi
  
  // Initialize I2C for PCA9685
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);  // 50Hz for servos
  
  // Home position
  setServoAngle(PAN_SERVO_CHANNEL, 0);
  setServoAngle(TILT_SERVO_CHANNEL, 0);
  
  Serial.println("ESP32 Motor Controller Ready");
}

void loop() {
  // Check for incoming UART data
  while (Serial2.available()) {
    uint8_t byte = Serial2.read();
    
    if (rx_index == 0 && byte == START_BYTE) {
      // Start of new message
      rx_buffer[rx_index++] = byte;
    }
    else if (rx_index > 0) {
      rx_buffer[rx_index++] = byte;
      
      // Check if we have complete message
      if (rx_index >= 4) {  // At least START + CMD + LEN + END minimum
        uint8_t payload_len = rx_buffer[2];
        uint8_t expected_len = 5 + payload_len;  // Full message length
        
        if (rx_index == expected_len && byte == END_BYTE) {
          // Complete message received
          processMessage(rx_buffer, rx_index);
          rx_index = 0;  // Reset for next message
        }
        else if (rx_index > MAX_PAYLOAD + 6) {
          // Buffer overflow - reset
          Serial.println("Buffer overflow - resetting");
          rx_index = 0;
        }
      }
    }
  }
}

void processMessage(uint8_t* buffer, uint8_t length) {
  // Verify checksum
  uint8_t calculated_checksum = 0;
  for (uint8_t i = 1; i < length - 2; i++) {  // Exclude START and END
    calculated_checksum ^= buffer[i];
  }
  
  uint8_t received_checksum = buffer[length - 2];
  
  if (calculated_checksum != received_checksum) {
    Serial.println("Checksum error");
    sendNACK();
    return;
  }
  
  // Extract command
  uint8_t command = buffer[1];
  uint8_t payload_len = buffer[2];
  uint8_t* payload = &buffer[3];
  
  // Process command
  bool success = false;
  
  switch (command) {
    case CMD_SET_PAN:
      success = handleSetPan(payload, payload_len);
      break;
      
    case CMD_SET_TILT:
      success = handleSetTilt(payload, payload_len);
      break;
      
    case CMD_MOVE_COMBINED:
      success = handleMoveCombined(payload, payload_len);
      break;
      
    case CMD_CALIBRATE:
      success = handleCalibrate();
      break;
      
    case CMD_STOP:
      success = handleStop();
      break;
      
    default:
      Serial.print("Unknown command: 0x");
      Serial.println(command, HEX);
      sendNACK();
      return;
  }
  
  if (success) {
    sendACK();
  } else {
    sendNACK();
  }
}

bool handleSetPan(uint8_t* payload, uint8_t len) {
  if (len != 2) return false;
  
  int16_t angle = (int16_t)(payload[0] | (payload[1] << 8));  // Little-endian
  current_pan = angle;
  
  float degrees = angle / 100.0;
  setServoAngle(PAN_SERVO_CHANNEL, degrees);
  
  Serial.print("Set Pan: ");
  Serial.print(degrees);
  Serial.println("°");
  
  return true;
}

bool handleSetTilt(uint8_t* payload, uint8_t len) {
  if (len != 2) return false;
  
  int16_t angle = (int16_t)(payload[0] | (payload[1] << 8));
  current_tilt = angle;
  
  float degrees = angle / 100.0;
  setServoAngle(TILT_SERVO_CHANNEL, degrees);
  
  Serial.print("Set Tilt: ");
  Serial.print(degrees);
  Serial.println("°");
  
  return true;
}

bool handleMoveCombined(uint8_t* payload, uint8_t len) {
  if (len != 4) return false;
  
  int16_t pan = (int16_t)(payload[0] | (payload[1] << 8));
  int16_t tilt = (int16_t)(payload[2] | (payload[3] << 8));
  
  current_pan = pan;
  current_tilt = tilt;
  
  float pan_deg = pan / 100.0;
  float tilt_deg = tilt / 100.0;
  
  setServoAngle(PAN_SERVO_CHANNEL, pan_deg);
  setServoAngle(TILT_SERVO_CHANNEL, tilt_deg);
  
  Serial.print("Move to Pan: ");
  Serial.print(pan_deg);
  Serial.print("°, Tilt: ");
  Serial.print(tilt_deg);
  Serial.println("°");
  
  return true;
}

bool handleCalibrate() {
  Serial.println("Calibrating - moving to home position");
  current_pan = 0;
  current_tilt = 0;
  setServoAngle(PAN_SERVO_CHANNEL, 0);
  setServoAngle(TILT_SERVO_CHANNEL, 0);
  delay(1000);  // Wait for servos to reach position
  return true;
}

bool handleStop() {
  Serial.println("Emergency stop");
  // Implement your stop logic (disable PWM, etc.)
  return true;
}

void setServoAngle(uint8_t channel, float angle) {
  // Clamp angle to safe limits
  angle = constrain(angle, -90, 90);
  
  // Map angle to pulse width
  float pulse_width = map(angle * 100, -9000, 9000, SERVO_MIN, SERVO_MAX);
  
  // Convert to PWM value (12-bit, 0-4095)
  uint16_t pwm_value = (uint16_t)((pulse_width / 20000.0) * 4096);
  
  pwm.setPWM(channel, 0, pwm_value);
}

void sendACK() {
  uint8_t ack[3] = {START_BYTE, ACK_BYTE, END_BYTE};
  Serial2.write(ack, 3);
  Serial2.flush();
}

void sendNACK() {
  uint8_t nack[3] = {START_BYTE, NACK_BYTE, END_BYTE};
  Serial2.write(nack, 3);
  Serial2.flush();
}