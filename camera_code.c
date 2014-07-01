

void getCamera()
{
  digitalWrite(clockPin, LOW);
  pinMode(syncPin, OUTPUT);
  digitalWrite(syncPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(syncPin, LOW);
  utime = micros();
  digitalWrite(clockPin, LOW);
  for (int j = 0; j < 128; j++)
    {
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
    }
  delayMicroseconds(expose);
  digitalWrite(syncPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(syncPin, LOW);
  utime = micros() - utime;
  digitalWrite(clockPin, LOW);
  for (int j = 0; j < 128; j++)
    {
    delayMicroseconds(20);
    lightVal[j] = analogRead(dataPin);
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
    }
  delayMicroseconds(20);
  for (int j = 0; j < 128; j++)
    {
    itoa(lightVal[j], 'S');
    }
  itime = (int)utime;
  itoa(itime, 'S');
}
