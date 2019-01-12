/* RMCQ Control and Indication Module LHA1
  By David Lowe 20181101

  This control unit is to control by switch 4 Servo controlled turnouts. The switch is 5 momentary Normal push-button
  type used to control the turnout direction. Point numbering is determined by the HO Layout design documents.
  These numbers are based on the sequential number within the scope of the panel that immediately address the point.

  Theory of Operation:
  The system on start up with set the position of each servo into a known initial position. In most cases this will
  set each point to Mains route or Normal condition. This is needed so the system is aware of the position.

  On command to move the point position, the system will determine if the servo needs to be moved. In other words it
  checks to see if the commanded position is the current position. If not, the servo will be moved into the commanded
  position.

  The control input and outputs are as follows:

  Pin Configuration:
  T100: Servo output = D10, T110: Servo output = D11, T101: Servo output = D12, T102: Servo output = D13,
  pushbutton input = D2 (Yard Out), pushbutton input = D3 (Departure), pushbutton input = D4 (Arrival),
  pushbutton input = D5 (Main Inner), pushbutton input = D6 (Main Outer),
  
  The feedback switches incorporated into the servo mounts are routed as inputs to the PortA of MCP23017. These
  inputs are read on each loop of the main program and the outputs updated. These are incorporated into MCP23017
  PortB. MCP Pin configuration are IAW RMCQIndicationA1.
  
  Credits:
  Program Debounce created 21 November 2006,  by David A. Mellis, modified 30 Aug 2011, by Limor Fried modified 28 Dec 2012,
  by Mike Walters

  Change log:
  RMCQ Control Module A1-2 Updated: 20151130
  - Amended point numbering and turnout throw labelling
  - Signals programing removed
  RMCQ Control Module A1-3 Updated: 20160412
  - Changed rogram to deal with toggling of points instead of going from one side to the other. This will allow setting the
  throw value to allow for
  physical position of the servo assembly compared to the point direction.
  RMCQ Control Module A1-4 Updated: 20160914
  - Change the servo change procedure to add the servo attach and detach.
  RMCQ Control Module A1-5 Update 20170729
  - Change program to be inline with Control Template which includes the RMCQ library.
  - Change to libraries to make referening them simpler by removing the current and end positions
  statements. Now the library is aware and auto calculates the correct starting and end positions
  based on the initial servo variables within the program.
  Update 20180914
  - New install to combine servo control and indication
  - Added MCP23017 I2C add 0x20 (Default)
  - Added MCP23017 to accept control panel servo feedback microswitch inputs (PortA)
  - Added MCP23017 to drive LED outputs for control panel indication (PortB)
  - New PCB constructed to to add MCP23017 and additional connectors
  - Added update routine to read servo feedback and switch panel LEDs on each loop
  Update 20181101
  - Added RMCQ Programming convention Standards and labels
*/

/*The program to control the logic indication of track routes (LEDs) based on the 
  feedback inputs (micro switches mounted on the turnout mount)  of turnouts. The 
  feedback signals are a logic LOW (taken to GND) when active. The route table below 
  show the indications as per the inputs received. 
  
  Turnout     100 110 101 102
  Main Out    -   -   -   1       1 = turont closed
  Main Inner  -   1   1   -       0 = turnout thrown
  Yard Out    1   -   -   -
  Depart      0   0   1   -
  Arrive      0   0   0   0
  
  This program differs from LogicIndicatorA1 in that it uses an I/O Port Expader
  componet MCP23017 (16 configurable I/O). Using the MCP we are going to use the 
  Adafruit-MCP23017 library 
  
  Configuration
  Connect pin #12 of the expander to Analog 5 (i2c clock) Arduino Pro Mini (APM)
  Connect pin #13 of the expander to Analog 4 (i2c data) APM
  Connect pins #15, 16 and 17 of the expander to ground (address selection = 0)
  Connect pin #9 of the expander to 5V (power)
  Connect pin #10 of the expander to ground (common ground)
  Connect pin #18 through a ~10kohm resistor to 5V (reset pin, active low)
  
  Inputs (active LOW, Port A (GPIOA))
  Connect pin #21 (OP 0) of the expander to Turnout 100 micro switch (PortA Bit0)
  Connect pin #22 (OP 1) of the expander to Turnout 110 micro switch (PortA Bit1)
  Connect pin #23 (OP 2) of the expander to Turnout 101 micro switch (PortA Bit2)
  Connect pin #24 (OP 3) of the expander to Turnout 102 micro switch (PortA Bit3)
  
  Outputs (active LOW, Port B (GPIOB))
  Connect pin #1 (OP 8) of the expander to yardOut LED (PortB Bit0)
  Connect pin #2 (OP 9) of the expander to mainOuter LED (PortB Bit1)
  Connect pin #3 <OP 10) of the expander to mainInner LED (PortB Bit2)
  Connect pin #4 (OP 11) of the expander to depart LED (PortB Bit3)
  Connect pin #5 (OP 12) of the expander to arrive LED (PortB Bit4)

*/
