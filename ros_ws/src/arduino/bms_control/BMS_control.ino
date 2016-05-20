const float CELL_VOLTS = 3.4;
const float TOTAL_VOLTAGE_CUTOFF = 17.5;
const float vPow = 4.98;//value of vin on board
const int INTERVAL = 10000;

//number of cells - 1 for storage as 0-4 in the array
const int NUM_CELLS = 5;
int Batt_Shutoff = 6;//Battery Shutoff Pin

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

float cells[NUM_CELLS];

const float r1 = 1000000.0;//value of r1
const float r2 = 100000.0;//value of r2

void setup()
{
  Serial.begin(115200);
  pinMode(Batt_Shutoff, OUTPUT);
  digitalWrite(Batt_Shutoff, LOW);//start with robot off
  
  Check_Batt_Shutoff();
}

void loop()
{
  currentMillis = millis();
  
  if(previousMillis - currentMillis >= INTERVAL) 
  {
    previousMillis = currentMillis;
    Check_Batt_Shutoff();
  }
}
 
void Check_Batt_Shutoff()
{
  //keeps track of cells that pass voltage reading
  unsigned int checkPoints = 0;
  
  float totalVolts = readVoltage(0);//get out output 
  
  cells[0] = readVoltage(4);
  cells[1] = readVoltage(3) - cells[0];
  cells[2] = readVoltage(2) - cells[1] - cells[0];
  cells[3] = readVoltage(1) - cells[2] - cells[1] - cells[0]; 
  cells[4] = readVoltage(0) - cells[3]  - cells[2] - cells[1] - cells[0]; 

  /*
   //Display individual cell values
   for(int i = 0; i < NUM_CELLS; i++)
   {
    Serial.print("Cell");
    Serial.print(i+1);
    Serial.print(" = ");
    Serial.println(cells[i]);
  }
  */

  //check each cell against the target value,
  //if within bounds, add it to our score
  for(int i = 0; i < NUM_CELLS; i++)
  {
    if(cells[i] > CELL_VOLTS)
      checkPoints++;
  }

  if((checkPoints >= 4) && (totalVolts > TOTAL_VOLTAGE_CUTOFF))
    digitalWrite(Batt_Shutoff, HIGH);
  else
    digitalWrite(Batt_Shutoff, LOW);

    
  return;
}

//Analog pins do not need to be decleared as Ax, so we just pass an int
float readVoltage(int cellNum)
{
    float voltage;
    float temp;
   
    temp = (analogRead(cellNum) * vPow) / 1023.0;
    voltage = temp / (r2 / (r1 + r2));
 /*   
    Serial.print("Voltage at cell");
    Serial.print(cellNum+1);
    Serial.print(" = ");
    Serial.print(voltage);
    Serial.println(" volts");
   */ 
    return voltage;
}

