 /* UNA 1 MCP23017
 *  Created by Glenn Black 08/11/2018
 *  Program debounce created  21 November , by David.A.Mellis
 *  Modified 30 August 2011 by Limor Fried
 *  Modified 28 December 2012 by Mike Walters
 *  RMCQControl Module A1 Servo Control created  by David.A.Lowe 20160330 on 01/09/18
 *  LogicIndicationA1_1 created by David Lowe 30971 on 01/09/18
  */
  /*Arduino   IDE Set up
   * Board Arduino Pro mini
   * Processor ATMega328P
   */
  /* THEORY OF OPERATION   2 Components  A/Logic Coontrol
   *                                     B/ Logic Indication
   *  
   *  Panel UNA Turnout Configeration
   *        PN1
   *  _____/______MN1
   *      200
   *        MN2   PN2     PN3     
   *  ____/______/_______/_____PN4
   *     201A   201B    201C
   *     
   *   THEORY OF OPERATION  (Logic Control)
   *   
   *  The system will start with the set position of each Servo in a known initial position 
   *  In most cases this will set each Turnout to the Closed (C) condition
   *  This is a requirement of the system to be aware of this position 
   *  On the COMMAND to MOVE (the Turnout position), 
   *  The system will determine if:-
   *  A/ The Servo needs to be moved 
   *  ie Checks to see if the command position is the current position
   *  OR
   *  B/ If NOT(The Current Position),  the servo will be moved into the command position
   *  
   *  ROUTE TABLE
   *  
   *  Closed (C) = 1    
   *  Thrown (T) = 0
   *  
   *  Turnout                200     201A    201B    201C
   *  
   *  Main Loop      (MN1)    1                                                                        
   *  
   *  Passing Loop   (PN1)    0
   *  
   *  Main   (MN2)                    0        
   *  
   *  Passing Loop 2 (PN2)            1        0       
   *  
   *  Passing Loop 3 (PN3)            1        1      0
   *  
   *  Passing Loop 4 (PN4)            1        1      1
   *  
   *  
   *   PIN Configuration:-
   *   
   *   OUTPUTS
   *   T200  Servo output = D10
   *   T201A Servo output = D11
   *   T201B Servo output = D12
   *   T201C Servo output = D13
   *   
   *   INPUTS
   *   Main Loop MN1    pushButton input = D2
   *   Passing Loop PN1 pushButtom input = D3
   *   Main Loop MN2    pushButton input = D4
   *   Passing Loop PN2 pushButton input = D5
   *   Passing Loop PN3 pushButton input = D6
   *   Passing Loop PN4 pushButton input = D7
   */
   /*THEORY OF OPERATION (Logic Indication) MCP23017
    * The program to control the LOGIC INDICATION  of the track routes (LED's)with
    * input  from the MICRO SWITCHES mounted on the Turnouts
    * The feedback signals are Logic Low when active (Turned On)
    * The Route Table below shows the indications as per the inputs received
    * 
    *   ROUTE TABLE
    *  Closed (C) = 1    
    *  Thrown (T) = 0
    
      Turnout                200     201A    201B    201C
   *  
   *  Main Loop      (MN1)    1                                                                        
   *  
   *  Passing Loop   (PN1)    0
   *  
   *  Main Loop 2   (MN2)             0        
   *  
   *  Passing Loop 2 (PN2)            1        0       
   *  
   *  Passing Loop 3 (PN3)            1        1      0
   *  
   *  Passing Loop 4 (PN4)            1        1      1
   *  
  
    * Description
    * This program usesI/O Port Expander(16 configurable I/O)
    * Libruary Adafruit-MCP23017
    * 
    * CONFIGURATION:-
    * Arduino PROMINI (APM) AnalogPin 5 (i2c Clock) to Expander pin#12
    * Arduino (APM) Analog Pin4 (i2c Data) to Expnder Pin#13
    * Expander Pin#15,#16,#17 to GROUND
    * Expander Pin #9 to 5V Power
    * Expander #10 to Ground Common
    * Expander pin #18 through a 10kohm resistor to 5V(Reset pin, active LOW)
    * 
    * OUTPUTS (Active LOW,Port B (GPIO B)
    * Connect  Pin#1(OP8)of the Expander to Passing Loop (PN1) LED (Port B Bit 0)
    * Connect  Pin#2(OP9)of the Expander to Main (MN1) LED (Port B Bit 1)
    * Connect  Pin#3(OP10)of the Expander to Main(MN2) LED (Port B Bit 2)
    * Connect  Pin#4(OP11)of the Expander to Passing Loop(PN2) LED (Port B Bit 3)
    * Connect  Pin#5(OP12)of the Expander to Passing Loop (PN3) LED (Port B Bit 4)
    * Connect  Pin#6(OP13)of the Expander to Passing Loop (PN4) LED (Port B Bit 5)
    *
    * INPUTS (Active LOW,Port A (GPIO A ) 
    * Connect pin#21(OP 0)Of the Expander  to  Turnout 200 Microswitch (Port A Bit 0) 
    * Connect pin#22(OP 1)Of the Expander  to  Turnout 201A Microswitch (Port A Bit 1)
    * Connect pin#23(OP 2)Of the Expander  to  Turnout 201B Microswitch (Port A Bit 2)
    * Connect pin#24(OP 3)Of the Expander  to  Turnout201C Microswitch (Port A Bit 3)
    */

    //  SCOPE :-

    #include <Servo.h>
    #include <RMCQControl.h>
    #include <Wire.h>
    #include <Adafruit_MCP23017.h>

    // RMCQ Stardard Identification

    #define progName "RMCQ Control and Identification Module UHA"
    #define version  "0.1 alpha"
    #define date "12 Nov 2018"
    #define author " G Black"

   //Create Servo object to control servo
    Servo servo200;
    Servo servo201A;
    Servo servo201B;
    Servo servo201C;

    //Constants
    //Main Loop(MN1)
    const int inputMainLoopMN1 = 6;

    //Passing Loop(PN1)
    const int inputPassingLoopPN1 = 7;

    //Main Loop(MN2)
    const int inputMainLoopMN2 = 5 ;

    // Passing Loop (PN2)
    const int inputPassingLoopPN2 =4;

    //Passing Loop (PN3)
    const  int inputPassingLoopPN3 = 3;

    //Passing Loop(PN4)
    const int inputPassingLoopPN4 = 2;
    

    //Constructor

    RMCQControl rmcqControl = RMCQControl();

    boolean currentServoState ;

    //Servo Global variables
    
    int startPos = 80;
    int throwNormal = 60;
    int throwReverse = 30;
    int timeBetweenServoChange = 20;
    int servoRateOfChange =30;

    /* Turnout Normal (Closed), and Reverse (Thrown) position for each servo used
     *  Set to each individual  Servo and physical setup
     */

     // Declare and Initialize Outputs

     int opPin200 = 10;
     int opPin201A = 11;
     int opPin201B = 12;
     int opPin201C = 13;

     //Servo Variables
     /* The Turnout is in the Normal  (Closed) state(HIGH)
      *                       Reverse (Thrown) state (LOW)
      */

      boolean currentServoState200 = HIGH;
      boolean currentServoState201A =HIGH;
      boolean currentServoState201B = HIGH;
      int offSet = 10;
      boolean currentServoState201C = HIGH;

      //VARIABLES

      /*Main LoopMN1 The current reading of the Input*/
      int inputStateMainLoopMN1 ;
      int lastInputStateMainLoopMN1 = HIGH;

      //Passing LoopPN1 input reading
      int inputStatePassingLoopPN1;
      int lastInputStatePassingLoopPN1= HIGH;

      //Main Loop MN2 input reading
      int inputStateMainLoopMN2 ;
      int lastInputStateMainLoopMN2 = HIGH;

      // Passing LoopPN2 input reading
      int inputStatePassingLoopPN2 ;
      int lastInputStatePassingLoopPN2 = HIGH;

      //Passing LoopPN3 input reading

      int inputStatePassingLoopPN3 ;
      int lastInputStatePassingLoopPN3 = HIGH;

      //Passing LoopPN4 input reading

      int inputStatePassingLoopPN4;
      int lastInputStatePassingLoopPN4 = HIGH ;

      //Time variable ( primitive Long due to millisecs time measurement)

      long lastDebounceTime = 0 ;

      //Debounce Time (increase if output flickers)

      long debounceDelay = 30 ;
      

      //RMCQControl Libruary Functions

     /* void RMCQControl :: changeNormal (boolean currentState, Servo  servoname, int outputPin)
      * void RMCQControl :: changeReverse(boolean currentState, Servo  servoname, int outputPin) 
      */

      // Set  Functions
      
      /* N =Normal (C)
       *  R = Reverse (T = Thrown)
       */

      void setMainLoopMN1 ()
      // On Main Loop MN1 : 200 N ,
      // Turnout 200 to Normal
      {rmcqControl.changeNormal(currentServoState200,servo200,opPin200);
      //  change to Normal
      currentServoState200 = HIGH;
      }

      void setPassingLoopPN1()
      /*On Passing LoopPN1 : 200 R,
       * Turnout 200 to Thrown
       */
       {rmcqControl.changeReverse (currentServoState200,servo200,opPin200);
       //change to Thrown
       currentServoState200 = LOW;
       }

       void setMainLoopMN2 ()
       /* On Main LoopMN2: 201A : R ,
        *  Turnout 201A to Thrown
        */
        {rmcqControl.changeReverse(currentServoState201A,servo201A,opPin201A);
        //change to Thrown
        currentServoState201A =LOW;
        }

        void setPassingLoopPN2 ()
        // On Passing LoopPN2 :201A N,: 201B R
        //Turnout 201A to Normal
        {rmcqControl.changeNormal(currentServoState201A,servo201A,opPin201A);
        currentServoState201A = HIGH;
        //Turnout 201B to Thrown
        rmcqControl.changeReverse(currentServoState201B,servo201B,opPin201B);
        currentServoState201B = LOW ;
        }

        void setPassingLoopPN3()
        //On Passing LoopPN3 :201A N: 201B N:201C R
        //Turnout 201A to Normal
        {rmcqControl.changeNormal(currentServoState201A,servo201A,opPin201A);
        currentServoState201A= HIGH;
        //Turnout 201B  to Normal
        rmcqControl.changeNormal(currentServoState201B,servo201B,opPin201B);
        currentServoState201B =HIGH;
        //Turnout 201C to Thrown
        rmcqControl.changeReverse(currentServoState201C,servo201C,opPin201C);
        currentServoState201C =LOW;
        }

        void setPassingLoopPN4()
        //On Passing LoopPN4 :201A N, 201B N, 201C N
        //Turnout201A to Normal
        {rmcqControl.changeNormal(currentServoState201A,servo201A,opPin201A);
        currentServoState201A =HIGH;
        //Turnout201B to Normal
        rmcqControl.changeNormal(currentServoState201B,servo201B,opPin201B);
        currentServoState201B = HIGH ;
        //Turnout201C to Normal
        rmcqControl.changeNormal(currentServoState201C,servo201C,opPin201C);
        currentServoState201C = HIGH;
        }

        // LOGIC iNDICATION
        /*
         * Define the micro Switch INPUT (PORT A GPA?) pins at the Expander MCP23017
         */ 
         #define t200Pin 0
         #define t201APin 1
         #define t201BPin 2
         #define t201CPin 3

         //define the LED OUTPUT (PORT B GPB?) pins to the Expander MCP23017

         #define mainMN1LEDPin 9
         #define passingPN1LEDPin 8
         #define mainMN2LEDPin 10
         #define passingPN2LEDPin 11
         #define passingPN3LEDPin 12
         #define passingPN4LEDPin 13

         //Constructor- Create an instance of the MCP23017 Libruary
         
         Adafruit_MCP23017 mcp; 
         
         
      void setup()
      {
       Serial.begin (9600);
       Serial.println("RMCQ Standard Identification Information");
       Serial.print ("Running");
       Serial.println(progName);
       Serial.print("version :");
       Serial.println(version);
       Serial.print(" Last Updated : ");
       Serial.println (date);
       Serial.print ("Programer: ");
       Serial.println  (author);

       //Set Servo Global Variables

       rmcqControl.setUp(startPos,throwNormal,throwReverse,timeBetweenServoChange,servoRateOfChange);

       //Set Pin configuration 

       pinMode(inputMainLoopMN1,INPUT_PULLUP);
       pinMode(inputPassingLoopPN1,INPUT_PULLUP);
       pinMode(inputMainLoopMN2,INPUT_PULLUP);
       pinMode(inputPassingLoopPN2,INPUT_PULLUP);
       pinMode(inputPassingLoopPN3,INPUT_PULLUP);
       pinMode(inputPassingLoopPN4,INPUT_PULLUP);

       // Set the INITIAL State of the Servo's

       //Set Turnout 200

       rmcqControl.center(servo200,startPos,opPin200);
       rmcqControl.changeNormal(LOW,servo200,opPin200);
       //Set to Normal State
       currentServoState200 = HIGH;

       //Set Turnout 201A

       rmcqControl.center(servo201A,startPos,opPin201A);
       rmcqControl.changeNormal(LOW,servo201A,opPin201A);
       //Set Normal State
       currentServoState201A =HIGH;

       //Set Turnout 201B

       rmcqControl.center(servo201B,startPos,opPin201B);
       rmcqControl.changeNormal(LOW,servo201B,opPin201B);
       //Set NormalState
       currentServoState201B =HIGH;

       //Set Turnout 201C

       rmcqControl.center(servo201C,startPos,opPin201C);
       rmcqControl.changeNormal(LOW,servo201C,opPin201C);
       //Set Normal State
       currentServoState201C = HIGH;


       // LOGIC INDICATION

       //Use default address 0
       mcp.begin();

       //Set Port A as INPUTS and 100k internal PULLUP

       for(uint8_t pin = 0; pin<8; pin++)
       {
        mcp.pinMode(pin, INPUT);
        mcp.pullUp (pin, HIGH);
       } 

       // Set Port B as OUTPUTS and turn all the LED's on LOW

       for(uint8_t pin= 8; pin <16 ; pin++) 
       {  
        mcp.pinMode(pin, OUTPUT);
        mcp.digitalWrite (pin, LOW);
       }
        //Turn the LEDs off
        delay (200);
        for(uint8_t  pin = 8 ;pin <16 ; pin++)
       {
        mcp.digitalWrite (pin, HIGH);
          
       }
        }
        

     /*Panel UNA Turnout Configeration
     *        PN1
     *  _____/______MN1
     *      200
     *        MN2   PN2     PN3     
     *  ____/______/_______/_____PN4
     *     201A   201B    201C
     *     
     *  Closed = C= N
     *  Thrown = T =R
      */
     

    void loop()
    
    // Read inputMainLoopMN1 Pin 6 
    // On MainLoopMN1 : 200 N
     {
      int inputMainLoopMN1State =digitalRead(inputMainLoopMN1);

     
      if(inputMainLoopMN1State != lastInputStateMainLoopMN1)
      {

       //reset the Debounce Timer
       lastDebounceTime =millis();
       Serial.println(" MainLoopMN1 Actived");
       }

       if((millis() - lastDebounceTime) > debounceDelay)
       {

        /*Checks to see if the Command Position is the Current Position
           * Current read of MainLoopMN1 State (NOT EQUAL TO) previous read 
           * of the MainLoopMN1 State 
            */
       if (inputMainLoopMN1State != inputStateMainLoopMN1)
       {
        inputStateMainLoopMN1 = inputMainLoopMN1State;
        if (inputMainLoopMN1State =LOW)
        {
          setMainLoopMN1 ();
               }
            }
          }
        lastInputStateMainLoopMN1 =inputMainLoopMN1State;


        // Read PassingLoopPN1 Pin 7
        //On PassingLoopPN1 : 200 R (T) ,
        
        int inputPassingLoopPN1State =digitalRead (inputPassingLoopPN1);

        if (inputPassingLoopPN1State != lastInputStatePassingLoopPN1)
        {
         //reset the Debounce Time
         lastDebounceTime =millis();
        }
        Serial.println(" Passing LoopPN1 Activated");

        if((millis() -lastDebounceTime) > debounceDelay)
        {
          /*Checks to see if the Command Position is the Current Position
           * Current read of PassingLoopPN1 State (NOT EQUAL TO) previous read 
           * of the PassingLoopPN1 State 
            */
         if(inputPassingLoopPN1State != inputStatePassingLoopPN1)
         {
         inputStatePassingLoopPN1 = inputPassingLoopPN1State ;

         if (inputPassingLoopPN1State == LOW)
         {
         setPassingLoopPN1 ();
             }
            }
          }
         lastInputStatePassingLoopPN1 = inputPassingLoopPN1State ;


          // Read inputMainLoopMN2 Pin 5
    // On Main Loop MN2 : 201A R (T)
     {
      int inputMainLoopMN2State =digitalRead(inputMainLoopMN2);

      //current read of MainLoopMN2State (NOT EQUAL TO) last assigned read of the MainLoopMN2
      if(inputMainLoopMN2State != lastInputStateMainLoopMN2)
      {

       //reset the Debounce Timer
       lastDebounceTime =millis();
       Serial.println(" MainLoopMN2 Actived");
       }

       if((millis() - lastDebounceTime) > debounceDelay)
       {
       if (inputMainLoopMN2State != inputStateMainLoopMN2)
       {
        inputStateMainLoopMN2 = inputMainLoopMN2State;
        if (inputMainLoopMN2State =LOW)
        {
          setMainLoopMN2 ();
               }
            }
          }
        lastInputStateMainLoopMN2 =inputMainLoopMN2State;

     }

       // Read PassingLoopPN2 Pin 4
        //On PassingLoopPN2 : 201A N, 201B R (T) ,
        
        int inputPassingLoopPN2State =digitalRead (inputPassingLoopPN2);

        if (inputPassingLoopPN2State != lastInputStatePassingLoopPN2)
        {
         //reset the Debounce Time
         lastDebounceTime =millis();
        }
        Serial.println(" Passing LoopPN2 Activated");

        if((millis() -lastDebounceTime) > debounceDelay)
        {
          /*Checks to see if the Command Position is the Current Position
           * Current read of PassingLoopPN2 State (NOT EQUAL TO) previous read 
           * of the PassingLoopPN2 State 
            */
         if(inputPassingLoopPN2State != inputStatePassingLoopPN2)
         {
         inputStatePassingLoopPN2 = inputPassingLoopPN2State ;

         if (inputPassingLoopPN2State == LOW)
         {
         setPassingLoopPN2 ();
             }
            }
          }
         lastInputStatePassingLoopPN2 = inputPassingLoopPN2State ;


           // Read PassingLoopPN3 Pin 3
        //On PassingLoopPN3 : 201A N , 201B N, 201C R (T)
        
        int inputPassingLoopPN3State =digitalRead (inputPassingLoopPN3);

        if (inputPassingLoopPN3State != lastInputStatePassingLoopPN3)
        {
         //reset the Debounce Time
         lastDebounceTime =millis();
        }
        Serial.println(" Passing LoopPN3 Activated");

        if((millis() -lastDebounceTime) > debounceDelay)
        {
          /*Checks to see if the Command Position is the Current Position
           * Current read of PassingLoopPN3 State (NOT EQUAL TO) previous read 
           * of the PassingLoopPN3 State 
            */
         if(inputPassingLoopPN3State != inputStatePassingLoopPN3)
         {
         inputStatePassingLoopPN3 = inputPassingLoopPN3State ;

         if (inputPassingLoopPN3State == LOW)
         {
         setPassingLoopPN3 ();
             }
            }
          }
         lastInputStatePassingLoopPN3 = inputPassingLoopPN3State ;



           // Read PassingLoopPN4 Pin 2
        //On PassingLoopPN4 : 201A N ,201B N,201C N
        
        int inputPassingLoopPN4State =digitalRead (inputPassingLoopPN4);

        if (inputPassingLoopPN4State != lastInputStatePassingLoopPN4)
        {
         //reset the Debounce Time
         lastDebounceTime =millis();
        }
        Serial.println(" Passing LoopPN4 Activated");

        if((millis() -lastDebounceTime) > debounceDelay)
        {
          /*Checks to see if the Command Position is the Current Position
           * Current read of PassingLoopPN4 State (NOT EQUAL TO) previous read 
           * of the PassingLoopPN2 State 
            */
         if(inputPassingLoopPN4State != inputStatePassingLoopPN4)
         {
         inputStatePassingLoopPN4 = inputPassingLoopPN4State ;

         if (inputPassingLoopPN4State == LOW)
         {
         setPassingLoopPN4 ();
             }
            }
          }
         lastInputStatePassingLoopPN4 = inputPassingLoopPN4State; 
         
         


         //LOGIC INDICATION LOOP CODE

         //Check for the condition and set the output accordingly

         //For mainMN1
         
         if(mcp.digitalRead(t200Pin))
         {
          mcp.digitalWrite(mainMN1LEDPin, LOW);
          
         }

         //For passingLoopPN1

         if(!mcp.digitalRead(t200Pin))
         {
          mcp.digitalWrite(passingPN1LEDPin, LOW);
          
         }

         //For mainMN2

         if(!mcp.digitalRead(t201APin))
         {
          mcp.digitalWrite(mainMN2LEDPin, LOW);
         }

         //For PassingLoopPN2

         if(mcp.digitalRead(t201APin) && !mcp.digitalRead(t201BPin));
         {
          mcp.digitalWrite(passingPN2LEDPin, LOW);
         }

         //ForPassingLoopPN3

         if(mcp.digitalRead(t201APin) && mcp.digitalRead(t201BPin) &&!mcp.digitalRead(t201CPin))
         {
          mcp.digitalWrite(passingPN3LEDPin ,LOW);
         }

         //For PassingLoopPN4
           if (mcp.digitalRead(t201APin) && mcp.digitalRead(t201BPin) && mcp.digitalRead(t201CPin))
          {
            mcp.digitalWrite(passingPN4LEDPin, LOW);
          } //End mcp class 
          } //End Main
         
         
         



       

        





         
         
        
       


















          
        
       
       
    
  
      
     
    
    
  

    
