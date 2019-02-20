// RMCQ Control Module 
 
 void RMCQIndicationUNA ()
 {
  //do a resetof All LED'S
  mcp.digitalWrite(mainMN1LEDPin, LOW);
  mcp.digitalWrite(passingPN1LEDPin, LOW);
  mcp.digitalWrite(mainMN2LEDPin, LOW);
  mcp.digitalWrite(passingPN2LEDPin,LOW);
  mcp.digitalWrite(passingPN3LEDPin,LOW);
  mcp.digitalWrite(passingPN4LEDPin,LOW);
  // for mainLoopMN1
  if (mcp.digitalRead(t200Pin)){
    mcp.digitalWrite(mainMN1LEDPin,HIGH);
  }
  //for passingPN1
  if(!mcp.digitalRead(t200Pin))
  { 
    mcp.digitalWrite(passingPN1LEDPin,HIGH);
  }
  //For MN2
  if(!mcp.digitalRead(t201APin))
  {
    mcp.digitalWrite(mainMN2LEDPin, HIGH);
  }
  //ForPN2
  if(mcp.digitalRead(t201APin) && !mcp.digitalRead(t201BPin)){
    mcp.digitalWrite(passingPN2LEDPin, HIGH);
  }
  // ForPN3
  if(mcp.digitalRead(t201APin) && mcp.digitalRead(t201BPin) && !mcp.digitalRead(t201CPin))
  {
    mcp.digitalWrite(passingPN3LEDPin,HIGH);
  }
  //For PN4
  if(mcp.digitalRead(t201APin) && mcp.digitalRead(t201BPin) &&mcp.digitalRead(t201CPin))
  {
    mcp.digitalWrite(passingPN4LEDPin ,HIGH);
  }
 }

