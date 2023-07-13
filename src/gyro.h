// #include <Arduino.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_SSD1306.h>

// Adafruit_MPU6050 gyro;

// // static to ensure local file scope
// // don't use #define, that will make it global to all files i think

// static const int SCREEN_WIDTH = 128;
// static const int SCREEN_HEIGHT = 64;
// static const int OLED_RESET = -1; // This display does not have a reset pin accessible
// Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// int calibration_runs = 5000;
// int slow_calibration_seconds = 15;
// double gyro_offsets[] = {0, 0, 0}; 
// double angle = 0;
// double angle_time = 0; // start angle time on first run>

// void display_text(String text) {
//   display_handler.clearDisplay();
//   display_handler.setTextSize(1);
//   display_handler.setTextColor(SSD1306_WHITE);
//   display_handler.setCursor(0,0);
//   display_handler.println(text);
//   display_handler.display();
// }

// double * display_read_gyro() {
//   sensors_event_t a, g, temp;
//   gyro.getEvent(&a, &g, &temp);

//   static double gyro_readings[] = {g.gyro.x - gyro_offsets[0], g.gyro.y - gyro_offsets[1], g.gyro.z - gyro_offsets[2]};

//   String gyro_report = "Rotation X: " + String(g.gyro.x - gyro_offsets[0]) + ", Rotation Y: " + String(g.gyro.y - gyro_offsets[1]) + ", Rotation Z: " + String(g.gyro.z - gyro_offsets[2]) + " rad/s";
//   display_text(gyro_report);
//   return gyro_readings;
// }

// double * read_gyro() {
//   sensors_event_t a, g, temp;
//   gyro.getEvent(&a, &g, &temp);

//   static double gyro_readings[] = {g.gyro.x - gyro_offsets[0], g.gyro.y - gyro_offsets[1], g.gyro.z - gyro_offsets[2]};

//   // String gyro_report = "Rotation X: " + String(g.gyro.x - gyro_offsets[0]) + ", Rotation Y: " + String(g.gyro.y - gyro_offsets[1]) + ", Rotation Z: " + String(g.gyro.z - gyro_offsets[2]) + " rad/s";
//   // display_text(gyro_report);
//   return gyro_readings;
// }

// void velocity_calibrate() {
//   double coord_sums[] = {0, 0, 0};
//   double *p;

//   for(int i = 0; i < calibration_runs; i++){
//       p = read_gyro();
//       for(int i = 0; i < 3; i++) {
//         coord_sums[i] += *(p + i);
//       }
//   }

//   for(int i = 0; i < 3; i++) {
//     gyro_offsets[i] = coord_sums[i] / calibration_runs;
//   }
// }

// double slow_calibrate() {
//   sensors_event_t a, g, temp;
//   gyro.getEvent(&a, &g, &temp);

//   double initial_value = g.gyro.z - gyro_offsets[2];
//   delay(slow_calibration_seconds * 1000);
  
//   gyro.getEvent(&a, &g, &temp);
//   double final_value = g.gyro.z - gyro_offsets[2];
  
//   return (final_value - initial_value) / slow_calibration_seconds;
// }

// double calculate_angle(double drift) {
//   // double *p;
//   // p = read_gyro();

//   sensors_event_t a, g, temp;
//   gyro.getEvent(&a, &g, &temp);

//   double dt = (millis() - angle_time) / 1000;
//   angle += (g.gyro.z - gyro_offsets[2] - drift) * dt;
//   angle_time = millis();

//   String angle_text = "Angle: " + String(angle);
//   display_text(angle_text);
//   return angle;
// }

// int drift;

// void setup() {
//   display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);

//   gyro.begin();
//   // gyro.setAccelerometerRange(MPU6050_RANGE_8_G);
//   // gyro.setGyroRange(MPU6050_RANGE_500_DEG);
//   // gyro.setFilterBandwidth(MPU6050_BAND_21_HZ);

//   velocity_calibrate();
//   display_text("fast calibration complete!");
//   delay(2000);

//   drift = slow_calibrate();
//   display_text("slow calibration complete!");
//   delay(2000);
// }

// void loop() {
//   // display_read_gyro();
//   // delay(100);
//   calculate_angle(drift);
// }
