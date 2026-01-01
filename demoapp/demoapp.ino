// simple demo app to blink PB0
void setup()
{
  DDRB |= 1;
}

void loop()
{
  PINB = 1; // toggle PB0
  delay(500);
}
