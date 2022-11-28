//Griffin Heyrich
//BU_MBL 11/18/2022
//the function of this set of code is to utilize the pyfirmata class to read in from file where coordinates
//are being sent and process the vector values using appropraite abg mapping to then calculate a volume for each chanber of the pump
//this enables movement in our soft robot

#include <Firmata.h>
#include <math.h>

//omni device coordinates 
String posX = ""; String posY = ""; String posZ = "";

//omni device coordinates after converted to double
float pos_X = 0.0; float pos_Y = 0.0; float pos_Z = 0.0;


void setup() 
{
  Serial.begin(9600);
  Firmata.attach(STRING_DATA, stringDataCallback);
  Firmata.begin();
}



void loop() 
{
  //delay(100);
  while(Firmata.available() )
  {
    Firmata.processInput(); // this sends incoming data to callback function
  }
}



void stringDataCallback(char* strdata)
{
  processStr(strdata); //send to process string to break into each coordinate
  
  //need to convert to char* before using atof()
  char*Xstr = &posX[0];
  char*Ystr = &posY[0];
  char*Zstr = &posZ[0];

  //converting char* to float
  pos_X = atof(Xstr);
  pos_Y = atof(Ystr);
  pos_Z = atof(Zstr);

  analogWrite(5,ceil(pos_X));
  analogWrite(6,ceil(pos_Y));
  analogWrite(9,ceil(pos_Z));
}


void processStr(char* strdata)
{

  int num_space = 0;
  String str(strdata);
  posX = "";
  posY = "";
  posZ = ""; // reset the string to empty at the beginning of the loop
  
  for (int i = 0; i < str.length(); i++)  
  {
    if (isspace(str[i]))
    {
      num_space+=1;
    }
    if (num_space == 3) 
    {
      return; // if the string has been properly converted all the way, return to call
    }

    if(str[i] == ']') {
      return;
    }
    
    if (!(isspace(str[i])) && (!(str[i] == ',') && (!(str[i] == '[')) && (!(str[i] == ']')) && num_space == 0)) //x
    {
      posX += str[i];
    }
    if (!(isspace(str[i])) && (!(str[i] == ',') && (!(str[i] == '[')) && (!(str[i] == ']')) && num_space == 1))  //y 
    {
      posY +=str[i];
    }
    if (!(isspace(str[i])) && (!(str[i] == ',') && (!(str[i] == '[')) && (!(str[i] == ']')) && num_space == 2)) //z
    {
      posZ +=str[i];
    }
  }
}
