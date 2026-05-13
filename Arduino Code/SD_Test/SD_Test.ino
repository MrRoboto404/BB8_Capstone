#include <SPI.h>
#include <SD.h>

File myFile;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Serial.println("Starting SD card");

  while(!SD.begin(10)){
    Serial.println("FISH");
    delay(100);
  }

  // checking the write ability
  Serial.println("WE OPENING THE CARD NOW");
  myFile = SD.open("test.csv", FILE_WRITE);
  if(myFile){
    Serial.println("WRITING LIKE SHAKESPEARE");
    myFile.println("3,");
    myFile.println("5,");
    myFile.close();
    Serial.println("Masterpiece created");
  } else {
    Serial.println("I broke my pencil");
  }

  // checking the read ability
  Serial.println("OH MY OPENING THE CARD");
  myFile = SD.open("test.csv");
  if(myFile){
    Serial.println("READ LIKE A BOOKWORM");

    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
  } else {
    Serial.println("I can't read");
  }

  Serial.println("Eject Card Now!");
}

void loop() {
  // put your main code here, to run repeatedly:
}
