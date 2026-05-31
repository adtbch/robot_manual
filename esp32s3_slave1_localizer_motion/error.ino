
void systemError(int code, String text) {
  // Simplified error handling for production
  // Critical error - system stops
  // if (code < 100) {
  //   Serial.print("CRITICAL ERROR - Code: ");
  //   Serial.print(code);
  //   Serial.print(" - ");
  //   Serial.println(text);
        
  //   while(true) {
  //     delay(1000);
  //   }
  // }
  
  // For warnings (code >= 100), just continue
}

void systemWarning(int code, String text) {
  // Serial.print("WARNING - Code: ");
  // Serial.print(code);
  // Serial.print(" - ");
  // Serial.println(text);
}

// Non-fatal error function - logs error but continues operation
void logError(int code, String text) {
  Serial.print("WARNING - Code: ");
  Serial.print(code);
  Serial.print(" - ");
  Serial.println(text);

  // Could also briefly show on LCD without stopping system
  // For now, just log to Serial
}


/**
 * Buzzer error function - sounds alarm when obstacle detected or error
 */
//  void buzzerError() {
//     // Buzzer alarm pattern - 3 short beeps
//     // for (int i = 0; i < 3; i++) {
//     //   digitalWrite(BUZZER_PIN, HIGH);
//     //   delay(100);
//     //   digitalWrite(BUZZER_PIN, LOW);
//     //   delay(100);
//     // }
//   }