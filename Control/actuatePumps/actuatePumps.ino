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

// abg coordinates "basis" vectors
float a1 = 0; float a2 = 1;
float b1 = -0.866; float b2 = -0.5;
float c1 = 0.866; float c2 = -0.5;

// Nunchuck position in abg coordinates
float alphaCoord = 0; float betaCoord = 0; float gammaCoord = 0;

// Chamber volume values
float alphaVol = 0; float betaVol = 0; float gammaVol = 0;

//chamber levels 
float alphaLevel =0.0; float betaLevel = 0.0; float gammaLevel = 0.0; 



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
  pos_Y = atof(Ystr) + 65.511 ; // bascailly  i am adjusting for the origin
  pos_Z = atof(Zstr) - 10; // note in jacobs he is subtracting the origin here i am adding
  
  alphaCoord = pos_Z;
  betaCoord = (pos_X * b1) + (pos_Z * b2);
  gammaCoord = (pos_X * c1) + (pos_Z * c2);

  //invert coordinates
  alphaCoord = -alphaCoord;
  betaCoord = -betaCoord;
  gammaCoord = -gammaCoord;

  alphaLevel = .3367* pos_Y;
  betaLevel = .3367* pos_Y;
  gammaLevel = .3367* pos_Y;
  
  //map the coordinate system to the appropriate volume 
  alphaVol = alphaLevel + (100*alphaCoord/180); 
  betaVol = betaLevel + (100*betaCoord/220);
  gammaVol = gammaLevel + (100*gammaCoord/220);

  //Constrain volume values from 0 to 100
  alphaVol = constrain(alphaVol, 0, 100);
  betaVol = constrain(betaVol, 0, 100);
  gammaVol = constrain(gammaVol, 0, 100);

  analogWrite(3,ceil(alphaVol));
  analogWrite(5,ceil(betaVol));
  analogWrite(6,ceil(gammaVol));
}



void processStr(char* strdata)
{

  int num_space = 0;
  String str(strdata);
  posX = "";
  posY = "";
  posZ = ""; // reset the string to empty at the beginning of the loo[p
  
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
    
    if (!(isspace(str[i])) && num_space == 0) //x
    {
      posX += str[i];
    }
    if (!(isspace(str[i])) && num_space == 1)  //y 
    {
      posY +=str[i];
    }
    if (!(isspace(str[i])) && num_space == 2) //z
    {
      posZ +=str[i];
    }
  }
}
