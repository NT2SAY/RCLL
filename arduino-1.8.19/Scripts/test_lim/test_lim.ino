#define USE_USBCON

#define dir_x 6
#define dir_y 8
#define dir_z 10

#define pul_x 7
#define pul_y 9
#define pul_z 11

#define lim_x 26
#define lim_y 24
#define lim_z 22


void setup(){
  Serial.begin(57600);
  pinMode(lim_x, INPUT_PULLUP);
  pinMode(lim_y, INPUT_PULLUP);
  pinMode(lim_z, INPUT_PULLUP);
}

void loop(){
 Serial.println("Limit state: " + String(digitalRead(lim_x))+ "\t"+ String(digitalRead(lim_y)) + "\t" + String(digitalRead(lim_z)));
 delay(500); 
}
