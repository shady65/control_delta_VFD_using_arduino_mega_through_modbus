const int inputSwitch = 5;
const int VFDTransmitEnable = 6; // HIGH:Transmitter, LOW:Receiver
const int incoderTransmitEnable = 7; // HIGH:Transmitter, LOW:Receiver
const int StartPin = 11; //start/stop hard wire
const int forwardPin = 10; //fwd/rev hard wire

const double ERROR_VALUE = 0.02;

int incomingByte = 0;
int high = 0;
int low = 0;

bool correction = false;

uint8_t Direction = 0; // 1 => forward , 2 => reverse
unsigned long lastRotationMeasureTime = 0;
unsigned long lastSpeedMeasureTime = 0;
double measurementTime = 0;
double previousMeasurementTime = 0;
double currentAngle = 0;
double previousRotation = 0;
double previousSpeed = 0;
double currentSpeed = 0;
double acceleration = 0;
double deltaTime = 0;
double deltaRotation = 0;
double deltaSpeed = 0;
double distance = 0;
double destinationAngle = 0;

double maxDistance = 0;
double maxFrequency = 0;
double minFrequency=0;
double frequency = 0;
double currentFrequency = 0;
double _delay =40;



double x = 0;
double y= 0 ;
//----start-----

void setup()
{
  Serial.begin(38400);
  Serial1.begin(115200);
  Serial2.begin(38400);

  
  //Serial.println(x);
  pinMode(VFDTransmitEnable, OUTPUT);
  pinMode(incoderTransmitEnable, OUTPUT);
  pinMode(inputSwitch, INPUT);
  pinMode(11, OUTPUT);
  maxDistance = 180;
  destinationAngle = 300.00;

  
  maxFrequency = 5;
  minFrequency = 3.3;
_delay =75;

 writeDoubleParameter(1,9,0.1);
 writeDoubleParameter(1,10,0.1);

 

writeIntParameter(2,1,1);

}

void loop(){
  // delay(1000);
// x=(micros());
// forward();
// delay(_delay);
// Stop();
// y=micros();
// delay(10);
// Serial.println("x: "+String(x));
// Serial.println("y: "+String(y));
// Serial.println(y-x);

 //delay(1000);
// x=(micros());
// moveFwdFast();
// delay(_delay);
// stopFast();
// y=micros();
// delay(10);
// Serial.println("x: "+String(x));
// Serial.println("y: "+String(y));
// Serial.println(y-x);
//   if(digitalRead(inputSwitch)==low){
//   delay(1000);
//   Serial.println(micros());
//   reverse();
//    delay(50);
//    Stop();
//    Serial.println(micros());
//   }
  
//   if(digitalRead(inputSwitch)==HIGH){
//  delay(1000);
//   Serial.println(micros());
//   moveRevFast();
//    delay(50);
//    stopFast();
//  Serial.println(micros());
//   }
//    measureAngle();
//    Serial.println(currentAngle);
 
//   if(digitalRead(inputSwitch)==HIGH){
//  correctingAngle();
// }
  if(digitalRead(inputSwitch)==HIGH){
  
  correctingAngleFast();  
}


}

void move_ToAngle(double destination)
{

  measureAngle();
  bool fwd = forwardIsShorterFromTo(currentAngle, destination);
  if (correction == 0 && fwd)
  {
    if (subtractAngle(destination,currentAngle ) >= ERROR_VALUE)
    {
      forward();
    }
    correction = 1;
  }
  else if (correction == 0 && !fwd)
  {
    if (subtractAngle(currentAngle, destination) >= ERROR_VALUE)
    {
      reverse();
    }
    correction = 1;
  }

  if (deltaRotation >= distance && deltaRotation < 45 && correction == true)
  {
    Stop();
    correctingAngle();
    if (distance <= ERROR_VALUE)
    {
      correction = false;
    }
  }
}

void correctingAngle()
{
  measureAngle();
  
  if(distance>=ERROR_VALUE){
      
   Serial.print("angle: "+String(currentAngle)+"              ");
   Serial.println("distance: "+ String(distance));
  }
 
  //writeFrequency(frequency);

  // Serial.println("======================================================== ");
 
   
   if(distance>30){
      frequency = (maxFrequency);
     if (frequency>maxFrequency)
  {
    Serial.println("frequency > maxFrequency");
    frequency =maxFrequency;
  }
   writeFrequency(frequency);

    if (forwardIsShorterFromTo(currentAngle, destinationAngle))
    {
      forward();
    }
    else
    {
      reverse();
    }
   }
 else if(distance>4){
    frequency = (minFrequency+distance*((maxFrequency-minFrequency)/maxDistance));
     if (frequency>maxFrequency)
  {
    Serial.println("frequency > maxFrequency");
    frequency =maxFrequency;
  }
   writeFrequency(frequency);

  if (forwardIsShorterFromTo(currentAngle, destinationAngle))
    {
      forward();
    }
    else
    {
      reverse();
    }
 }
  else if (distance >= ERROR_VALUE)
  
  {
    if (forwardIsShorterFromTo(currentAngle, destinationAngle))
    {
      forward();
    }
    else
    {
      reverse();
    }
    
    //delaymicros(frequency*(distance)*distance);
  
    delay(_delay);
    Stop();
    delay(10);
  }
  else{
    Stop();
  }
}

void correctingAngleFast()
{
  measureAngle();
  
  if(distance>=0){
      
   Serial.print("angle: "+String(currentAngle)+"              ");
   Serial.println("distance: "+ String(distance));
  }
 
  //writeFrequency(frequency);

  // Serial.println("======================================================== ");
 
   
   if(distance>30){
      frequency = (maxFrequency);
     if (frequency>maxFrequency)
  {
    Serial.println("frequency > maxFrequency");
    frequency =maxFrequency;
  }
   writeFrequency(frequency);

    if (forwardIsShorterFromTo(currentAngle, destinationAngle))
    {
      moveFwdFast();
    }
    else
    {
      moveRevFast();
    }
   }
 else if(distance>3){
    frequency = (minFrequency+distance*((maxFrequency-minFrequency)/maxDistance));
     if (frequency>maxFrequency)
  {
    Serial.println("frequency > maxFrequency");
    frequency =maxFrequency;
  }
   writeFrequency(frequency);

   if (forwardIsShorterFromTo(currentAngle, destinationAngle))
    {
      moveFwdFast();
    }
    else
    {
      moveRevFast();
    }
 }
  else if (distance >= ERROR_VALUE){
  

  
    if (forwardIsShorterFromTo(currentAngle, destinationAngle))
    {
      moveFwdFast();
    }
    else
    {
      moveRevFast();
    }
    
    //delaymicros(frequency*(distance)*distance);
  
    delay(_delay);
    stopFast();
    delay(80);
  }
  else{
    stopFast();
  }
}

//-----------------------Main requests to vfd----------------------------




double subtractAngle(double angle1, double angle2)
{
  if (angle1 >= angle2)
  {
    return angle1 - angle2;
  }
  if (angle1 < angle2)
  {
    return (360 - angle2) + angle1;
  }
}

bool forwardIsShorterFromTo(double angle, double destination)
{
  if (subtractAngle(destination,angle ) <= subtractAngle(angle, destination))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void stopFast(){
     digitalWrite(StartPin,0);
}
void moveFwdFast(){
     digitalWrite(forwardPin,0);
     digitalWrite(StartPin,1);
}
void moveRevFast(){
     digitalWrite(forwardPin,1);
     digitalWrite(StartPin,1);
}

void Stop()
{
  controlRs485(1, 6, 0x20, 0, 1, 1);
}
void forward()
{
  if (Direction != 1)
  {
    controlRs485(1, 6, 0x20, 0, 16, 1);
    Direction = 1;
  }
  controlRs485(1, 6, 0x20, 0, 2, 1);
}
void reverse()
{
  if (Direction != 2)
  {
    controlRs485(1, 6, 0x20, 0, 32, 1);
    Direction = 2;
  }
  controlRs485(1, 6, 0x20, 0, 2, 1);
}
void writeFrequency(double _frequency)
{ 
  if(abs(currentFrequency - _frequency)>=0.5){
  controlRs485(1, 6, 0x20, 1, _frequency, 100);
  currentFrequency = _frequency;
  }
}
void writeDoubleParameter(int high_address, int low_address, double value)
{
  controlRs485(1, 6, high_address, low_address, value, 100);
}
void writeIntParameter(int high_address, int low_address, double value)
{
  controlRs485(1, 6, high_address, low_address, value, 1);
}

//----------------------------------generate request function------------------------

void controlRs485(byte id, byte command, byte highAddress, byte lowAddress, double value, int factor)
{
  if (value > 400)
  {
    value = 400;
  }
  value = value * factor;
  int requestCounter = 3;
  int done = 8;
  while (requestCounter)
  {
    digitalWrite(VFDTransmitEnable, HIGH);

    byte //id, command , highAddress , lowAddress
        highData,
        lowData, highCrc, lowCrc;
    // Serial.println(value);

    int freq = (int)value;
    highData = HighLowFromInt(freq, 0);
    lowData = HighLowFromInt(freq, 1);
    unsigned int crc = ParityCheck(id, command, highAddress, lowAddress, highData, lowData);
    highCrc = HighLowFromInt(crc, 1);
    lowCrc = HighLowFromInt(crc, 0);
    unsigned char bytes[] = {id, command, highAddress, lowAddress, highData, lowData, highCrc, lowCrc};
    int Lbyte = 0;
    int Hbyte = 0;
    Serial2.write(&bytes[0], 8);
    Serial2.flush();
    digitalWrite(VFDTransmitEnable, LOW);
    delay(6);
    while (Serial2.available() > 0)
    {
      done--;
      Hbyte = Lbyte;
      Lbyte = Serial2.read();
      //Serial.println(Lbyte,HEX);
    }
    if (done <= 0)
    {
      //Serial.println("done");
      break;
    }
    else
    {
      requestCounter--;
      Serial.println("the number of bytes that didnt came: "+String(done));
    }
  }
}

//---------------------generating request to vfd sub functions----------------------------

unsigned int ParityCheck(byte id, byte command, byte highAddress, byte lowAddress, byte highData, byte lowData)
{
  unsigned char length;
  byte *data = new byte[2];
  data[0] = id;
  data[1] = command;
  data[2] = highAddress;
  data[3] = lowAddress;
  data[4] = highData;
  data[5] = lowData;
  length = 6;
  int j;
  unsigned int reg_crc = 0xFFFF;
  while (length--)
  {
    reg_crc ^= *data++;
    for (j = 0; j < 8; j++)
    {
      if (reg_crc & 0x01)
      { /* LSB(b0)=1 */
        reg_crc = (reg_crc >> 1) ^ 0xA001;
      }
      else
      {
        reg_crc = reg_crc >> 1;
      }
    }
  }
  return reg_crc;
}

byte HighLowFromInt(int value, bool returnLow)
{
  String _value = convertDecimalToBinary(value);
  byte high = 0;
  byte low = 0x00;
  int num;
  num = binaryStringToInt(_value.substring(0, 8));
  high += num;
  num = binaryStringToInt(_value.substring(8, 16));
  low += num;
  if (!returnLow)
    return high;
  if (returnLow)
    return low;
}
//--------------------------encoder functions----------------------
void measureAngle()
{
  if (Serial.available() == 0)
  {
    digitalWrite(incoderTransmitEnable, HIGH);
    Serial1.write(0x54);
    Serial1.flush();
    digitalWrite(incoderTransmitEnable, LOW);
    delay(10);
  }
  while (Serial1.available() > 0)
  {
    incomingByte = Serial1.read();
    //Serial.println("read");
    //Serial.println(incomingByte,HEX);
    low = high;
    high = incomingByte;
  }
  previousMeasurementTime = measurementTime;
  previousRotation = currentAngle;
  measurementTime = micros();
  currentAngle = encoderResponseToDegree();

  if (forwardIsShorterFromTo(currentAngle, destinationAngle))
  { 
    if(Direction == 1){
    deltaRotation = subtractAngle(currentAngle,previousRotation);
    }
    else{
      deltaRotation = subtractAngle(previousRotation,currentAngle);
      }
      
      distance = subtractAngle(destinationAngle,currentAngle);
  }
  else
  {
    if(Direction == 1){
    deltaRotation = subtractAngle(currentAngle,previousRotation);
    }
    else{
      deltaRotation = subtractAngle(previousRotation,currentAngle);
      }
    distance = subtractAngle(currentAngle, destinationAngle);
    
  }
  deltaTime = (measurementTime - previousMeasurementTime);

  //            Serial.println(deltaTime);
  //            Serial.println(deltaRotation);
  previousSpeed = currentSpeed;
  currentSpeed = deltaRotation /(deltaTime/1000);
  deltaSpeed = (currentSpeed - previousSpeed);
  acceleration = deltaSpeed / deltaTime;
}

double encoderResponseToDegree()
{
  String messegeInBinary = convertDecimalToBinary(high).substring(8, 16) + convertDecimalToBinary(low).substring(8, 16);
  unsigned int messegeValue = binaryStringToInt(messegeInBinary);
  if (checkeEncoderMessege(messegeValue))
  {
    //Serial.println("true");
    double degree = calulateEncoderMessegeValue(messegeValue);
    degree *= 360.00;
    degree /= 16384.00;
    //Serial.println(degree);
    lastRotationMeasureTime = micros();
    return degree;
  }
  else
  {

    lastRotationMeasureTime = micros();
    return currentAngle;
  }
}
int calulateEncoderMessegeValue(int messegeValue)
{
  String binary = convertDecimalToBinary(messegeValue);
  String value = binary.substring(2, 16);
  return binaryStringToInt(value);
}
bool checkeEncoderMessege(int messegeValue)
{
  String binary = convertDecimalToBinary(messegeValue);
  String check = binary.substring(0, 2);
  String even = evenBits(binary.substring(2, 16));
  String odd = oddBits(binary.substring(2, 16));
  String checkOddBit = String(odd[0]);
  for (int i = 1; i < 7; i++)
  {
    if (checkOddBit != String(odd[i]))
    {
      checkOddBit = "1";
    }
    else
    {
      checkOddBit = "0";
    }
  }
  String checkEvenBit = String(even[0]);
  for (int i = 1; i < 7; i++)
  {
    if (checkEvenBit != String(even[i]))
    {
      checkEvenBit = "1";
    }
    else
    {
      checkEvenBit = "0";
    }
  }
  if ((checkOddBit != String(check[0])) and (checkEvenBit != String(check[1])))
  {
    return true;
    //Serial.println("true");
  }
  else
  {
    return false;
    // Serial.println("false");
  }
}

//SMALL general FUNCTIONS
//---------------------------------------------------

int power(int num, int power)
{
  int result = num;
  if (power > 0)
  {
    for (int i = 0; i < power - 1; i++)
    {
      result *= num;
    }
    return result;
  }
  else
  {
    return 1;
  }
}

int binaryStringToInt(String binary)
{
  int sum = 0;
  int digit;
  int p;

  for (int i = 0; i < binary.length(); i++)
  {
    digit = String(binary[binary.length() - i - 1]).toInt();
    p = power(2, i);
    digit = digit * p;
    sum = sum + (digit);
  }
  return sum;
}

String convertDecimalToBinary(int num)
{
  uint8_t bitsCount = sizeof(num) * 8;
  char str[bitsCount + 1];
  uint8_t i = 0;
  while (bitsCount--)
    str[i++] = bitRead(num, bitsCount) + '0';
  str[i] = '\0';
  return str;
}

String oddBits(String binary)
{
  String oddbits;
  for (int i = 0; i < 14; i++)
  {
    if (i % 2 == 0)
    {
      oddbits += binary[i];
    }
  }
  return oddbits;
}

String evenBits(String binary)
{
  String evenbits;
  for (int i = 0; i < 14; i++)
  {
    if (i % 2 != 0)
    {
      evenbits += binary[i];
    }
  }
  return evenbits;
}
