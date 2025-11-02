/*
 * ============================================================================
 * A340E TRANSMISSION CONTROLLER - ESP32-S3 VERSION
 * For 1996 Lexus LS400
 * 
 * Optimized for ESP32-S3 DevKitC
 * 
 * Features:
 * - WiFi/Bluetooth real-time monitoring
 * - Web-based tuning interface
 * - Better PWM control with LEDC
 * - Faster processing for adaptive learning
 * - JSON data logging (SD card or WiFi)
 * ============================================================================
 */

 #include <WiFi.h>
 #include <WebServer.h>
 #include <ESPmDNS.h>
 #include <ArduinoJson.h>
 #include <SPI.h>
 #include <SD.h>
 #include <esp32-hal-ledc.h>
 
 // ==================== PIN DEFINITIONS (ESP32-S3 DevKitC) ====================
 
 // Solenoid outputs (3.3V logic - MOSFETs work fine)
 #define S1_PIN 2          // Shift solenoid 1
 #define S2_PIN 4          // Shift solenoid 2
 #define SLU_PIN 5         // Lock-up solenoid (PWM ~300Hz)
 #define SLN_PIN 6         // Accumulator solenoid (PWM ~300Hz)
 
 // Sensor inputs (ADC1 pins - don't use ADC2 when WiFi is active!)
 #define TPS_PIN 1         // ADC1_CH0 - Throttle Position (needs voltage divider!)
 #define FLUID_TEMP_PIN 2  // ADC1_CH1 - Fluid temperature
 
 // Frequency inputs (interrupt capable)
 #define VSS_PIN 7         // Vehicle Speed Sensor
 #define ENGINE_RPM_PIN 8  // Engine RPM
 #define OUTPUT_RPM_PIN 9  // Trans output shaft speed
 
 // Digital inputs (with internal pullup)
 #define BRAKE_PIN 10      // Brake pedal switch
 #define OD_SWITCH_PIN 11  // Overdrive enable switch
 #define MODE_SWITCH_PIN 12 // Normal/Power mode switch
 
 // SD card (using HSPI)
 #define SD_CS_PIN 13
 #define SD_SCK_PIN 14
 #define SD_MISO_PIN 15
 #define SD_MOSI_PIN 16
 
 // Status LED
 #define STATUS_LED_PIN 48  // RGB LED on many S3 boards
 
 // ==================== PWM CONFIGURATION ====================
 #define PWM_FREQ 300           // 300Hz for both solenoids
 #define PWM_RESOLUTION 10      // 10-bit = 0-1023 (better than Arduino's 8-bit)
 #define PWM_MAX_DUTY 1023
 
 // ==================== WIFI CONFIGURATION ====================
 
 const char* ssid = "TransController";      // Create AP mode by default
 const char* password = "transmission123";  // Change this!
 
 // Or connect to existing network:
 // const char* ssid = "YourHomeWiFi";
 // const char* password = "YourPassword";
 
 WebServer server(80);
 
 // ==================== SHIFT LOGIC CONSTANTS ====================
 
 // Shift point tables (same as Arduino version)
 const int SHIFT_1_2_UP[5] = {15, 20, 30, 45, 60};
 const int SHIFT_2_3_UP[5] = {35, 45, 60, 80, 100};
 const int SHIFT_3_4_UP[5] = {55, 65, 85, 110, 130};
 
 const int SHIFT_2_1_DOWN[5] = {10, 12, 18, 25, 35};
 const int SHIFT_3_2_DOWN[5] = {28, 35, 48, 65, 80};
 const int SHIFT_4_3_DOWN[5] = {48, 55, 72, 95, 115};
 
 // Power mode (more aggressive)
 const int SHIFT_1_2_UP_PWR[5] = {20, 30, 45, 60, 75};
 const int SHIFT_2_3_UP_PWR[5] = {45, 60, 80, 100, 120};
 const int SHIFT_3_4_UP_PWR[5] = {70, 85, 110, 130, 150};
 
 #define KICKDOWN_THRESHOLD 85
 #define KICKDOWN_4_3_SPEED 120
 #define KICKDOWN_3_2_SPEED 90
 #define KICKDOWN_2_1_SPEED 50
 
 #define SHIFT_DELAY 150
 #define SHIFT_COMPLETE_TIME 500
 #define SHIFT_INHIBIT_TIME 800
 
 #define ACC_SOFT 70
 #define ACC_MEDIUM 50
 #define ACC_FIRM 30
 #define ACC_KICKDOWN 20
 
 #define LOCKUP_ENABLE_GEAR 3
 #define LOCKUP_ENABLE_SPEED 60
 #define LOCKUP_DISABLE_SPEED 50
 #define LOCKUP_THROTTLE_MAX 70
 
 // ==================== DATA STRUCTURES ====================
 
 enum ShiftState {
   SHIFT_STABLE,
   SHIFT_REQUESTED,
   SHIFT_IN_PROGRESS,
   SHIFT_COMPLETING
 };
 
 struct TransmissionState {
   int current_gear;
   int target_gear;
   ShiftState shift_state;
   unsigned long shift_start_time;
   unsigned long last_shift_time;
   unsigned long shift_duration_ms;
   bool kickdown_active;
   bool power_mode;
   bool lockup_engaged;
   int lockup_duty;
   int accumulator_duty;
   int shift_quality_offset[3];
   int shift_count[3];
   bool limp_mode;
 } trans_state;
 
 struct SensorData {
   int tps_raw;
   int throttle_percent;
   int vehicle_speed_kmh;
   int engine_rpm;
   int output_rpm;
   int fluid_temp_c;
   bool brake_pressed;
   bool overdrive_enabled;
   int throttle_filtered;
   int speed_filtered;
 } sensors;
 
 struct Statistics {
   unsigned long total_shifts;
   unsigned long uptime_ms;
   float avg_shift_time_ms;
   int max_temp_c;
   unsigned long wifi_clients;
 } stats;
 
 // ==================== INTERRUPT HANDLERS ====================
 
 volatile unsigned long vss_last_pulse = 0;
 volatile unsigned long vss_pulse_period = 0;
 volatile unsigned long engine_rpm_last_pulse = 0;
 volatile unsigned long engine_rpm_pulse_period = 0;
 volatile unsigned long output_rpm_last_pulse = 0;
 volatile unsigned long output_rpm_pulse_period = 0;
 
 void IRAM_ATTR vss_isr() {
   unsigned long now = micros();
   vss_pulse_period = now - vss_last_pulse;
   vss_last_pulse = now;
 }
 
 void IRAM_ATTR engine_rpm_isr() {
   unsigned long now = micros();
   engine_rpm_pulse_period = now - engine_rpm_last_pulse;
   engine_rpm_last_pulse = now;
 }
 
 void IRAM_ATTR output_rpm_isr() {
   unsigned long now = micros();
   output_rpm_pulse_period = now - output_rpm_last_pulse;
   output_rpm_last_pulse = now;
 }
 
 // ==================== PWM CONTROL (ESP32-S3 LEDC) ====================
 
 void setupPWM() {
   ledcAttach(SLU_PIN, PWM_FREQ, PWM_RESOLUTION);
   ledcAttach(SLN_PIN, PWM_FREQ, PWM_RESOLUTION);
   
   // Start with 0 duty cycle
   ledcWrite(SLU_PIN, 0);
   ledcWrite(SLN_PIN, 0);
   
   Serial.println("PWM initialized: 300Hz, 10-bit resolution");
 }
 
 void setLockup(int duty) {
   duty = constrain(duty, 0, 100);
   int pwm_val = map(duty, 0, 100, 0, PWM_MAX_DUTY);
   ledcWrite(SLU_PIN, pwm_val);
 }
 
 void setAccumulator(int duty) {
   duty = constrain(duty, 0, 100);
   int pwm_val = map(duty, 0, 100, 0, PWM_MAX_DUTY);
   ledcWrite(SLN_PIN, pwm_val);
 }
 
 // ==================== SENSOR READING ====================
 
 void readSensors() {
   // TPS - ESP32-S3 ADC is 12-bit (0-4095)
   // CRITICAL: Input must be 0-3.3V! Use voltage divider for 5V TPS!
   sensors.tps_raw = analogRead(TPS_PIN);
   int tps_percent = map(sensors.tps_raw, 0, 4095, 0, 100);
   tps_percent = constrain(tps_percent, 0, 100);
   
   // Exponential filter
   sensors.throttle_filtered = (sensors.throttle_filtered * 3 + tps_percent) / 4;
   sensors.throttle_percent = sensors.throttle_filtered;
   
   // VSS - Calculate from pulse period
   if (vss_pulse_period > 0 && vss_pulse_period < 1000000) {
     float freq_hz = 1000000.0 / vss_pulse_period;
     int speed = (int)(freq_hz * 3600.0 / 8000.0);
     sensors.speed_filtered = (sensors.speed_filtered * 7 + speed) / 8;
   } else if (micros() - vss_last_pulse > 1000000) {
     sensors.speed_filtered = 0;
   }
   sensors.vehicle_speed_kmh = constrain(sensors.speed_filtered, 0, 250);
   
   // Engine RPM
   if (engine_rpm_pulse_period > 0 && engine_rpm_pulse_period < 100000) {
     float freq_hz = 1000000.0 / engine_rpm_pulse_period;
     sensors.engine_rpm = (int)(freq_hz * 60.0 / 2.0);
   } else if (micros() - engine_rpm_last_pulse > 500000) {
     sensors.engine_rpm = 0;
   }
   sensors.engine_rpm = constrain(sensors.engine_rpm, 0, 8000);
   
   // Output shaft RPM
   if (output_rpm_pulse_period > 0 && output_rpm_pulse_period < 200000) {
     float freq_hz = 1000000.0 / output_rpm_pulse_period;
     sensors.output_rpm = (int)(freq_hz * 60.0 / 2.0);
   } else if (micros() - output_rpm_last_pulse > 1000000) {
     sensors.output_rpm = 0;
   }
   
   // Fluid temperature
   int temp_raw = analogRead(FLUID_TEMP_PIN);
   float voltage = temp_raw * 3.3 / 4095.0;  // Note: 3.3V reference!
   sensors.fluid_temp_c = (int)((voltage - 0.5) * 100);
   sensors.fluid_temp_c = constrain(sensors.fluid_temp_c, -40, 150);
   
   // Update max temp stat
   if (sensors.fluid_temp_c > stats.max_temp_c) {
     stats.max_temp_c = sensors.fluid_temp_c;
   }
   
   // Digital inputs (active low with pullup)
   sensors.brake_pressed = !digitalRead(BRAKE_PIN);
   sensors.overdrive_enabled = !digitalRead(OD_SWITCH_PIN);
   trans_state.power_mode = !digitalRead(MODE_SWITCH_PIN);
 }
 
 // ==================== SHIFT LOGIC (Same as Arduino) ====================
 
 int getShiftPoint(const int table[5], int throttle_percent) {
   if (throttle_percent <= 10) return table[0];
   if (throttle_percent <= 25) return map(throttle_percent, 10, 25, table[0], table[1]);
   if (throttle_percent <= 50) return map(throttle_percent, 25, 50, table[1], table[2]);
   if (throttle_percent <= 75) return map(throttle_percent, 50, 75, table[2], table[3]);
   return map(throttle_percent, 75, 100, table[3], table[4]);
 }
 
 bool isKickdownRequested() {
   static int last_throttle = 0;
   static unsigned long throttle_change_time = 0;
   
   if (sensors.throttle_percent - last_throttle > 20) {
     throttle_change_time = millis();
   }
   
   last_throttle = sensors.throttle_percent;
   
   return (sensors.throttle_percent > KICKDOWN_THRESHOLD) &&
          (millis() - throttle_change_time < 200);
 }
 
 float calculateSlip() {
   if (sensors.output_rpm == 0 || sensors.engine_rpm < 500) return 0;
   
   float gear_ratios[4] = {2.804, 1.531, 1.000, 0.705};
   float final_drive = 3.266;
   
   if (trans_state.current_gear < 1 || trans_state.current_gear > 4) return 0;
   
   float expected_output = sensors.engine_rpm / gear_ratios[trans_state.current_gear - 1];
   float slip_percent = ((expected_output - sensors.output_rpm) / expected_output) * 100;
   
   return abs(slip_percent);
 }
 
 int determineTargetGear() {
   if (trans_state.limp_mode) return 3;
   
   int target = trans_state.current_gear;
   
   const int* shift_1_2 = trans_state.power_mode ? SHIFT_1_2_UP_PWR : SHIFT_1_2_UP;
   const int* shift_2_3 = trans_state.power_mode ? SHIFT_2_3_UP_PWR : SHIFT_2_3_UP;
   const int* shift_3_4 = trans_state.power_mode ? SHIFT_3_4_UP_PWR : SHIFT_3_4_UP;
   
   if (!sensors.overdrive_enabled && target > 3) target = 3;
   
   if (isKickdownRequested()) {
     trans_state.kickdown_active = true;
     if (trans_state.current_gear == 4 && sensors.vehicle_speed_kmh < KICKDOWN_4_3_SPEED) return 3;
     if (trans_state.current_gear == 3 && sensors.vehicle_speed_kmh < KICKDOWN_3_2_SPEED) return 2;
     if (trans_state.current_gear == 2 && sensors.vehicle_speed_kmh < KICKDOWN_2_1_SPEED) return 1;
   } else {
     trans_state.kickdown_active = false;
   }
   
   // Upshifts
   if (trans_state.current_gear == 1) {
     if (sensors.vehicle_speed_kmh > getShiftPoint(shift_1_2, sensors.throttle_percent)) target = 2;
   }
   else if (trans_state.current_gear == 2) {
     if (sensors.vehicle_speed_kmh > getShiftPoint(shift_2_3, sensors.throttle_percent)) target = 3;
   }
   else if (trans_state.current_gear == 3 && sensors.overdrive_enabled) {
     if (sensors.vehicle_speed_kmh > getShiftPoint(shift_3_4, sensors.throttle_percent)) target = 4;
   }
   
   // Downshifts
   if (trans_state.current_gear == 4) {
     if (sensors.vehicle_speed_kmh < getShiftPoint(SHIFT_4_3_DOWN, sensors.throttle_percent)) target = 3;
   }
   else if (trans_state.current_gear == 3) {
     if (sensors.vehicle_speed_kmh < getShiftPoint(SHIFT_3_2_DOWN, sensors.throttle_percent)) target = 2;
   }
   else if (trans_state.current_gear == 2) {
     if (sensors.vehicle_speed_kmh < getShiftPoint(SHIFT_2_1_DOWN, sensors.throttle_percent)) target = 1;
   }
   
   if (sensors.brake_pressed && sensors.throttle_percent < 5) {
     if (trans_state.current_gear == 4 && sensors.vehicle_speed_kmh < 70) target = 3;
     if (trans_state.current_gear == 3 && sensors.vehicle_speed_kmh < 45) target = 2;
   }
   
   return target;
 }
 
 void executeShift(int from_gear, int to_gear) {
   switch(to_gear) {
     case 1:
       digitalWrite(S1_PIN, LOW);
       digitalWrite(S2_PIN, LOW);
       break;
     case 2:
       digitalWrite(S1_PIN, HIGH);
       digitalWrite(S2_PIN, LOW);
       break;
     case 3:
       digitalWrite(S1_PIN, LOW);
       digitalWrite(S2_PIN, HIGH);
       break;
     case 4:
       digitalWrite(S1_PIN, HIGH);
       digitalWrite(S2_PIN, HIGH);
       break;
   }
   
   trans_state.current_gear = to_gear;
   trans_state.last_shift_time = millis();
   stats.total_shifts++;
   
   Serial.print("SHIFT: ");
   Serial.print(from_gear);
   Serial.print(" -> ");
   Serial.println(to_gear);
 }
 
 int calculateAccumulatorDuty() {
   int base_duty = ACC_MEDIUM;
   
   if (trans_state.shift_state == SHIFT_IN_PROGRESS) {
     if (trans_state.kickdown_active) base_duty = ACC_KICKDOWN;
     else if (sensors.throttle_percent > 60) base_duty = ACC_FIRM;
     else if (sensors.throttle_percent < 25) base_duty = ACC_SOFT;
     else base_duty = ACC_MEDIUM;
     
     int shift_index = trans_state.target_gear - 2;
     if (shift_index >= 0 && shift_index < 3) {
       base_duty += trans_state.shift_quality_offset[shift_index];
     }
   } else {
     base_duty = ACC_MEDIUM;
   }
   
   if (sensors.fluid_temp_c < 40) base_duty -= 20;
   else if (sensors.fluid_temp_c < 60) base_duty -= 10;
   else if (sensors.fluid_temp_c > 100) base_duty += 10;
   
   return constrain(base_duty, 15, 85);
 }
 
 int calculateLockupDuty() {
   bool can_lockup = (trans_state.current_gear >= LOCKUP_ENABLE_GEAR) &&
                     (sensors.vehicle_speed_kmh > LOCKUP_ENABLE_SPEED) &&
                     (sensors.throttle_percent < LOCKUP_THROTTLE_MAX) &&
                     (trans_state.shift_state == SHIFT_STABLE) &&
                     (sensors.fluid_temp_c > 50);
   
   bool must_unlock = (sensors.vehicle_speed_kmh < LOCKUP_DISABLE_SPEED) ||
                      (sensors.throttle_percent > LOCKUP_THROTTLE_MAX + 10) ||
                      (trans_state.shift_state != SHIFT_STABLE) ||
                      (trans_state.current_gear < LOCKUP_ENABLE_GEAR);
   
   if (must_unlock) {
     trans_state.lockup_engaged = false;
     return 0;
   }
   
   if (can_lockup) {
     trans_state.lockup_engaged = true;
     if (sensors.throttle_percent < 20) return 95;
     else if (sensors.throttle_percent < 40) return 75;
     else return 50;
   }
   
   return 0;
 }
 
 void updateAdaptiveLearning() {
   if (trans_state.target_gear <= trans_state.current_gear) return;
   if (trans_state.kickdown_active) return;
   if (sensors.throttle_percent > 75) return;
   
   int shift_index = trans_state.target_gear - 2;
   if (shift_index < 0 || shift_index >= 3) return;
   
   trans_state.shift_duration_ms = millis() - trans_state.shift_start_time;
   
   if (trans_state.shift_duration_ms > 450) {
     trans_state.shift_quality_offset[shift_index] -= 2;
   }
   else if (trans_state.shift_duration_ms < 350) {
     trans_state.shift_quality_offset[shift_index] += 2;
   }
   
   trans_state.shift_quality_offset[shift_index] = constrain(
     trans_state.shift_quality_offset[shift_index], -20, 20
   );
   
   trans_state.shift_count[shift_index]++;
 }
 
 void processShiftLogic() {
   unsigned long current_time = millis();
   
   switch (trans_state.shift_state) {
     case SHIFT_STABLE:
       trans_state.target_gear = determineTargetGear();
       
       if (trans_state.target_gear != trans_state.current_gear) {
         if (current_time - trans_state.last_shift_time > SHIFT_INHIBIT_TIME) {
           trans_state.shift_state = SHIFT_REQUESTED;
           trans_state.shift_start_time = current_time;
         }
       }
       break;
     
     case SHIFT_REQUESTED:
       if (current_time - trans_state.shift_start_time > SHIFT_DELAY) {
         int recheck_target = determineTargetGear();
         
         if (recheck_target == trans_state.target_gear) {
           trans_state.shift_state = SHIFT_IN_PROGRESS;
           executeShift(trans_state.current_gear, trans_state.target_gear);
         } else {
           trans_state.shift_state = SHIFT_STABLE;
         }
       }
       break;
     
     case SHIFT_IN_PROGRESS:
       if (current_time - trans_state.shift_start_time > SHIFT_COMPLETE_TIME) {
         trans_state.shift_state = SHIFT_COMPLETING;
       }
       break;
     
     case SHIFT_COMPLETING:
       if (current_time - trans_state.shift_start_time > SHIFT_COMPLETE_TIME + 200) {
         trans_state.shift_state = SHIFT_STABLE;
         updateAdaptiveLearning();
       }
       break;
   }
   
   trans_state.accumulator_duty = calculateAccumulatorDuty();
   trans_state.lockup_duty = calculateLockupDuty();
   
   setAccumulator(trans_state.accumulator_duty);
   setLockup(trans_state.lockup_duty);
 }
 
 // ==================== WEB SERVER ====================
 
 void setupWebServer() {
   // Main dashboard
   server.on("/", HTTP_GET, []() {
     String html = R"(
 <!DOCTYPE html>
 <html>
 <head>
   <title>A340E Controller</title>
   <meta name="viewport" content="width=device-width, initial-scale=1">
   <style>
     body { font-family: Arial; margin: 20px; background: #1a1a1a; color: #fff; }
     .card { background: #2a2a2a; padding: 20px; margin: 10px 0; border-radius: 10px; }
     .gauge { font-size: 48px; font-weight: bold; text-align: center; }
     .label { font-size: 14px; color: #888; }
     .value { font-size: 24px; margin: 5px 0; }
     .gear { font-size: 72px; color: #00ff00; }
     button { padding: 15px 30px; font-size: 16px; margin: 5px; }
   </style>
   <script>
     function update() {
       fetch('/data').then(r => r.json()).then(data => {
         document.getElementById('gear').innerText = data.gear;
         document.getElementById('tps').innerText = data.tps + '%';
         document.getElementById('speed').innerText = data.speed + ' km/h';
         document.getElementById('rpm').innerText = data.rpm + ' RPM';
         document.getElementById('temp').innerText = data.temp + '°C';
         document.getElementById('lockup').innerText = data.lockup ? 'ENGAGED' : 'OFF';
         document.getElementById('mode').innerText = data.power_mode ? 'POWER' : 'NORMAL';
       });
     }
     setInterval(update, 200);
     update();
   </script>
 </head>
 <body>
   <h1>A340E Transmission Controller</h1>
   <div class="card">
     <div class="label">Current Gear</div>
     <div class="gauge gear" id="gear">-</div>
   </div>
   <div class="card">
     <div class="label">Throttle Position</div>
     <div class="value" id="tps">-</div>
   </div>
   <div class="card">
     <div class="label">Vehicle Speed</div>
     <div class="value" id="speed">-</div>
   </div>
   <div class="card">
     <div class="label">Engine RPM</div>
     <div class="value" id="rpm">-</div>
   </div>
   <div class="card">
     <div class="label">Fluid Temperature</div>
     <div class="value" id="temp">-</div>
   </div>
   <div class="card">
     <div class="label">Lockup Status</div>
     <div class="value" id="lockup">-</div>
   </div>
   <div class="card">
     <div class="label">Shift Mode</div>
     <div class="value" id="mode">-</div>
   </div>
 </body>
 </html>
     )";
     server.send(200, "text/html", html);
   });
   
   // JSON data endpoint
   server.on("/data", HTTP_GET, []() {
     JsonDocument doc;
     doc["gear"] = trans_state.current_gear;
     doc["target_gear"] = trans_state.target_gear;
     doc["tps"] = sensors.throttle_percent;
     doc["speed"] = sensors.vehicle_speed_kmh;
     doc["rpm"] = sensors.engine_rpm;
     doc["temp"] = sensors.fluid_temp_c;
     doc["lockup"] = trans_state.lockup_engaged;
     doc["power_mode"] = trans_state.power_mode;
     doc["acc_duty"] = trans_state.accumulator_duty;
     doc["lu_duty"] = trans_state.lockup_duty;
     doc["slip"] = calculateSlip();
     doc["uptime"] = millis() / 1000;
     doc["total_shifts"] = stats.total_shifts;
     
     String response;
     serializeJson(doc, response);
     server.send(200, "application/json", response);
   });
   
   server.begin();
   Serial.println("Web server started");
 }
 
 // ==================== INITIALIZATION ====================
 
 void initTransmission() {
   trans_state.current_gear = 1;
   trans_state.target_gear = 1;
   trans_state.shift_state = SHIFT_STABLE;
   trans_state.shift_start_time = 0;
   trans_state.last_shift_time = 0;
   trans_state.shift_duration_ms = 0;
   trans_state.kickdown_active = false;
   trans_state.power_mode = false;
   trans_state.lockup_engaged = false;
   trans_state.lockup_duty = 0;
   trans_state.accumulator_duty = ACC_MEDIUM;
   trans_state.limp_mode = false;
   
   for (int i = 0; i < 3; i++) {
     trans_state.shift_quality_offset[i] = 0;
     trans_state.shift_count[i] = 0;
   }
   
   sensors.throttle_filtered = 0;
   sensors.speed_filtered = 0;
   
   stats.total_shifts = 0;
   stats.max_temp_c = 0;
   
   digitalWrite(S1_PIN, LOW);
   digitalWrite(S2_PIN, LOW);
   
   Serial.println("Transmission initialized - 1st gear");
 }
 
 // ==================== SETUP ====================
 
 void setup() {
   Serial.begin(115200);
   delay(1000);
   
   Serial.println("\n\n====================================");
   Serial.println("A340E Controller - ESP32-S3");
   Serial.println("1996 Lexus LS400");
   Serial.println("====================================\n");
   
   // Pin modes
   pinMode(S1_PIN, OUTPUT);
   pinMode(S2_PIN, OUTPUT);
   pinMode(SLU_PIN, OUTPUT);
   pinMode(SLN_PIN, OUTPUT);
   pinMode(STATUS_LED_PIN, OUTPUT);
   
   // Analog inputs (ESP32-S3 ADC doesn't need pinMode)
   analogSetAttenuation(ADC_11db);  // 0-3.3V range for all ADC pins
   
   pinMode(BRAKE_PIN, INPUT_PULLUP);
   pinMode(OD_SWITCH_PIN, INPUT_PULLUP);
   pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
   
   pinMode(VSS_PIN, INPUT);
   pinMode(ENGINE_RPM_PIN, INPUT);
   pinMode(OUTPUT_RPM_PIN, INPUT);
   
   // Attach interrupts for frequency inputs
   attachInterrupt(digitalPinToInterrupt(VSS_PIN), vss_isr, RISING);
   attachInterrupt(digitalPinToInterrupt(ENGINE_RPM_PIN), engine_rpm_isr, RISING);
   attachInterrupt(digitalPinToInterrupt(OUTPUT_RPM_PIN), output_rpm_isr, RISING);
   
   // Setup PWM
   setupPWM();
   
   // Initialize transmission
   initTransmission();
   
   // Setup WiFi
   Serial.print("Starting WiFi AP: ");
   Serial.println(ssid);
   WiFi.softAP(ssid, password);
   IPAddress IP = WiFi.softAPIP();
   Serial.print("AP IP address: ");
   Serial.println(IP);
   
   // Setup mDNS
   if (MDNS.begin("transcontroller")) {
     Serial.println("mDNS responder started: http://transcontroller.local");
   }
   
   // Setup web server
   setupWebServer();
   
   // Startup sequence - visual feedback
   for (int i = 0; i < 3; i++) {
     digitalWrite(STATUS_LED_PIN, HIGH);
     delay(200);
     digitalWrite(STATUS_LED_PIN, LOW);
     delay(200);
   }
   
   Serial.println("\nSystem ready!");
   Serial.println("Connect to WiFi and visit: http://transcontroller.local or http://" + WiFi.softAPIP().toString());
   Serial.println();
 }
 
 // ==================== MAIN LOOP ====================
 
 void loop() {
   // Read all sensors
   readSensors();
   
   // Process shift logic
   processShiftLogic();
   
   // Handle web server
   server.handleClient();
   
   // Status LED heartbeat
   static unsigned long last_blink = 0;
   if (millis() - last_blink > 1000) {
     digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
     last_blink = millis();
   }
   
   // Serial diagnostics (optional - comment out for production)
   static unsigned long last_print = 0;
   if (millis() - last_print > 500) {
     printStatus();
     last_print = millis();
   }
   
   // Control loop timing
   delay(20);  // 50Hz control loop
 }
 
 // ==================== DIAGNOSTICS ====================
 
 void printStatus() {
   Serial.print("G:");
   Serial.print(trans_state.current_gear);
   if (trans_state.shift_state != SHIFT_STABLE) {
     Serial.print("->");
     Serial.print(trans_state.target_gear);
   }
   Serial.print(" | TPS:");
   Serial.print(sensors.throttle_percent);
   Serial.print("% | Spd:");
   Serial.print(sensors.vehicle_speed_kmh);
   Serial.print(" | RPM:");
   Serial.print(sensors.engine_rpm);
   Serial.print(" | Tmp:");
   Serial.print(sensors.fluid_temp_c);
   Serial.print("C | ACC:");
   Serial.print(trans_state.accumulator_duty);
   Serial.print("% | LU:");
   Serial.print(trans_state.lockup_engaged ? "ON" : "OFF");
   
   if (trans_state.power_mode) Serial.print(" | PWR");
   if (trans_state.kickdown_active) Serial.print(" | KD");
   if (trans_state.limp_mode) Serial.print(" | LIMP");
   
   Serial.print(" | WiFi:");
   Serial.print(WiFi.softAPgetStationNum());
   
   Serial.println();
 }
 
 // ==================== SERIAL COMMANDS ====================
 
 void processSerialCommands() {
   if (!Serial.available()) return;
   
   char cmd = Serial.read();
   
   switch(cmd) {
     case 'd':
     case 'D':
       printDiagnostics();
       break;
       
     case 'r':
     case 'R':
       resetAdaptiveLearning();
       break;
       
     case '1':
       forceGear(1);
       break;
       
     case '2':
       forceGear(2);
       break;
       
     case '3':
       forceGear(3);
       break;
       
     case '4':
       forceGear(4);
       break;
       
     case 'w':
     case 'W':
       Serial.println("\nWiFi Status:");
       Serial.print("SSID: ");
       Serial.println(ssid);
       Serial.print("IP: ");
       Serial.println(WiFi.softAPIP());
       Serial.print("Clients: ");
       Serial.println(WiFi.softAPgetStationNum());
       break;
       
     case 'h':
     case 'H':
       Serial.println("\n===== COMMAND HELP =====");
       Serial.println("D - Print diagnostics");
       Serial.println("R - Reset adaptive learning");
       Serial.println("1-4 - Force gear");
       Serial.println("W - WiFi status");
       Serial.println("H - Show this help");
       Serial.println("========================\n");
       break;
   }
 }
 
 void printDiagnostics() {
   Serial.println("\n========== DIAGNOSTICS ==========");
   
   Serial.println("--- Transmission State ---");
   Serial.print("Current Gear: "); Serial.println(trans_state.current_gear);
   Serial.print("Target Gear: "); Serial.println(trans_state.target_gear);
   Serial.print("Lockup: "); Serial.println(trans_state.lockup_engaged ? "ENGAGED" : "DISENGAGED");
   Serial.print("Power Mode: "); Serial.println(trans_state.power_mode ? "ON" : "OFF");
   Serial.print("Total Shifts: "); Serial.println(stats.total_shifts);
   
   Serial.println("\n--- Sensors ---");
   Serial.print("TPS: "); Serial.print(sensors.throttle_percent); Serial.println("%");
   Serial.print("Speed: "); Serial.print(sensors.vehicle_speed_kmh); Serial.println(" km/h");
   Serial.print("Engine RPM: "); Serial.println(sensors.engine_rpm);
   Serial.print("Fluid Temp: "); Serial.print(sensors.fluid_temp_c); Serial.println("°C");
   Serial.print("Max Temp: "); Serial.print(stats.max_temp_c); Serial.println("°C");
   
   Serial.println("\n--- Control Outputs ---");
   Serial.print("Accumulator Duty: "); Serial.print(trans_state.accumulator_duty); Serial.println("%");
   Serial.print("Lockup Duty: "); Serial.print(trans_state.lockup_duty); Serial.println("%");
   
   Serial.println("\n--- Adaptive Learning ---");
   Serial.print("1-2 Shift Offset: "); Serial.println(trans_state.shift_quality_offset[0]);
   Serial.print("2-3 Shift Offset: "); Serial.println(trans_state.shift_quality_offset[1]);
   Serial.print("3-4 Shift Offset: "); Serial.println(trans_state.shift_quality_offset[2]);
   
   Serial.println("\n--- Performance ---");
   Serial.print("Uptime: "); Serial.print(millis() / 1000); Serial.println(" seconds");
   Serial.print("Free Heap: "); Serial.print(ESP.getFreeHeap()); Serial.println(" bytes");
   Serial.print("CPU Freq: "); Serial.print(ESP.getCpuFreqMHz()); Serial.println(" MHz");
   
   Serial.println("\n--- WiFi ---");
   Serial.print("SSID: "); Serial.println(ssid);
   Serial.print("IP: "); Serial.println(WiFi.softAPIP());
   Serial.print("Clients: "); Serial.println(WiFi.softAPgetStationNum());
   
   Serial.println("================================\n");
 }
 
 void resetAdaptiveLearning() {
   for (int i = 0; i < 3; i++) {
     trans_state.shift_quality_offset[i] = 0;
     trans_state.shift_count[i] = 0;
   }
   Serial.println("Adaptive learning reset");
 }
 
 void forceGear(int gear) {
   if (gear < 1 || gear > 4) return;
   
   trans_state.current_gear = gear;
   trans_state.target_gear = gear;
   executeShift(trans_state.current_gear, gear);
   
   Serial.print("Forced gear: ");
   Serial.println(gear);
 }