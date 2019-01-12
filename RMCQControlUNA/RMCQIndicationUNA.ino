/*RMCQ Control Module UNA Feedback Indication Unit - Module UNA
  By David Lowe 20181101
  See Description for further detail

            Route         Point
              __________ L2   P1
   T200 _____/__________ L1   M1


   T201A ____ __________ L3   M2
   T201B     \__________ L4   P2
   T201C      \_________ L5   P3
               \________ L6   P4

  /define the LED output pins to the expander (MCP pins)
  #define RIUNAL1 4    //M1
  #define RIUNAL2 3    //P1
  #define RIUNAL3 5    //M2
  #define RIUNAL4 6    //P2
  #define RIUNAL5 7    //P3
  #define RIUNAL6 8    //P4

*/

void RMCQIndicationUNA() {
  //do a reset of all LEDs
  mcp.digitalWrite(RIUNAL1, LOW); mcp.digitalWrite(RIUNAL2, LOW); mcp.digitalWrite(RIUNAL3, LOW); mcp.digitalWrite(RIUNAL4, LOW); mcp.digitalWrite(RIUNAL5, LOW); mcp.digitalWrite(RIUNAL6, LOW);
  //Check for each condition and set the output accordingly
  //for L1
  if (!mcp.digitalRead(TI200)) {
    mcp.digitalWrite(RIUNAL1, HIGH);
  } else {
    mcp.digitalWrite(RIUNAL2, HIGH);
  }

  //for L3
  if (!mcp.digitalRead(TI201A)) {
    mcp.digitalWrite(RIUNAL3, HIGH);
  }
  //for L4
  if (!mcp.digitalRead(TI201A) && !mcp.digitalRead(TI201B)) {
    mcp.digitalWrite(RIUNAL4, HIGH);
  }
  //for L5
  if (!mcp.digitalRead(TI201A) && !mcp.digitalRead(TI201B) && mcp.digitalRead(TI201C)) {
    mcp.digitalWrite(RIUNAL5, HIGH);
  }
  //for L6
  if (!mcp.digitalRead(TI201A) && !mcp.digitalRead(TI201B) && !mcp.digitalRead(TI201C)) {
    mcp.digitalWrite(RIUNAL6, HIGH);
  }
}
