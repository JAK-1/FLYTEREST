#include <NewPing.h>
// inlcude mavlink.h file given in the folder as well
NewPing sonar0(2, 3, 300);
NewPing sonar1(4, 5, 300);
NewPing sonar2(6, 7, 300);
NewPing sonar3(8, 9, 300);
NewPing sonar4(10, 11, 100);
unsigned long HT = 0; 
uint16_t Pitch = 0;
uint16_t Roll  = 0;
uint16_t PitchOut = 0;
uint16_t RollOut  = 0;
uint16_t PitchOutTemp = 0;
uint16_t RollOutTemp  = 0;
uint8_t n         = 0;
#define distance     5
#define Distance_C  100  
#define AltMin          70 
#define DistMin         50 
#define Compensation    800
struct Sensors {
  uint16_t Distance[distance]  = {0};
  uint16_t MediaDistance          = 0;
  bool C                        = false;
  bool Active                       = false;
  unsigned long CompensatedTime       = 0;
};
#define NSensors 5
Sensors Sensor[NSensors];
void setup() {
  Serial.begin(57600);
}

void loop() {
  if ( (millis() - HT) > 1000 ) {
    HT = millis();
    FHT();
  }
  FSensors();
  FRCOverride();
}
void FSensors() {
  ShiftArrays();
  MediaSensors();
  MediaDistance();
  CompareDistance();
}
void FRCOverride() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  
  Pitch  = ComparePitch(Pitch);
  Roll   = CompareRoll(Roll);
  
  CompensationInertia();

  if( Pitch != PitchOutTemp || Roll != RollOutTemp ){
    n = 0;
    PitchOutTemp = Pitch;
    RollOutTemp  = Roll;
  }else{
    n += 1;
    if(n == 4){
      RollOut = RollOutTemp;
      PitchOut = PitchOutTemp;
      RCOverride(&msg, len, buf, PitchOut, RollOut);
    }
  }  
}
void ShiftArrays() {
  for (uint8_t i = 0; i < NSensors; i++) {
    for (uint8_t j = distance - 1; j > 0; j--) {
      Sensor[i].Distance[j] = Sensor[i].Distance[j - 1];
    }
  }
}
void MediaSensors() {
  Sensor[0].Distance[0] = sonar0.ping_cm();
  Sensor[1].Distance[0] = sonar1.ping_cm();
  Sensor[2].Distance[0] = sonar2.ping_cm();
  Sensor[3].Distance[0] = sonar3.ping_cm();
  Sensor[4].Distance[0] = sonar4.ping_cm();
}
void MediaDistance() {
  for (uint8_t i = 0; i < NSensors; i++) {
    int Total   = 0;
    uint8_t Num = 0;
    for (uint8_t j = 0; j < distance; j++) {
      if (Sensor[i].Distance[j] != 0  && Sensor[i].Distance[j] < 300) {
        Total += Sensor[i].Distance[j];
        Num += 1;
      }
    }
    if (Num > 3) {
      Sensor[i].MediaDistance = Total / Num;
    } else {
      Sensor[i].MediaDistance = 0;
    }
  }
  void CompareDistance() {
  //Mínimo de 10 para la distancia. existen errores de medida 
  for (uint8_t i = 0; i < NSensors; i++) {
    if (Sensor[i].MediaDistance != 0 && Sensor[i].MediaDistance < Distance_C) {
      Sensor[i].C = true;
    } else {
      Sensor[i].C = false;
    }
  }
}
int16_t ComparePitch(uint16_t Pitch) {
  int16_t Diferencia = Sensor[0].MediaDistance - Sensor[2].MediaDistance;
  if( Sensor[4].MediaDistance > AltMin || Sensor[4].MediaDistance == 0 ) {
    if( abs(Diferencia) > DistMin ) {
   if( Sensor[0].C == true ) {
      if( Sensor[2].C == true ) {
       if( Sensor[0].MediaDistance < Sensor[2].MediaDistance ) {
          return( Pitch = ValorRC( Sensor[0].MediaDistance, 1 ) );
        }else{
          return( Pitch = ValorRC( Sensor[2].MediaDistance, 0 ) );
        }
      }else{
       return( Pitch = ValorRC( Sensor[0].MediaDistance, 1 ) );
      }
    }else {
     if( Sensor[2].C == true ) {
        return( Pitch = ValorRC( Sensor[2].MediaDistance, 0 ) );
      }else{
        return( Pitch = 0 );
      }
    }
  }else if( Sensor[0].C == true && Sensor[2].MediaDistance == 0 ) {
    return( Pitch = ValorRC( Sensor[0].MediaDistance, 1 ) );
    }else if ( Sensor[0].MediaDistance == 0 && Sensor[2].C == true ) {
      return( Pitch = ValorRC( Sensor[2].MediaDistance, 0 ) );
      }else {
        return( Pitch = 0 );
      }
  }else{
    return( Pitch = 0 );
  }
}
uint16_t CompareRoll(uint16_t Roll) {  
  int16_t Diferencia = Sensor[1].MediaDistance - Sensor[3].MediaDistance;
  if( Sensor[4].MediaDistance > AltMin || Sensor[4].MediaDistance == 0 ) {
    if( abs(Diferencia) > DistMin ) {
      if( Sensor[1].C == true ) {
       if( Sensor[3].C == true ) {
         if( Sensor[1].MediaDistance < Sensor[3].MediaDistance ) {
           return( Roll = ValorRC( Sensor[1].MediaDistance, 0 ) );
          }else{
          return( Roll = ValorRC( Sensor[3].MediaDistance, 1 ) );
          }
        }else{
          return( Roll = ValorRC( Sensor[1].MediaDistance, 0 ) );
        }
      }else {
       if( Sensor[3].C == true ) {
         return( Roll = ValorRC( Sensor[3].MediaDistance, 1 ) );
        }else{
        return( Roll = 0 );
        }
      }
    }else if( Sensor[1].C == true && Sensor[3].MediaDistance == 0 ) {
      return( Roll = ValorRC( Sensor[1].MediaDistance, 0 ) );
      }else if ( Sensor[1].MediaDistance == 0 && Sensor[3].C == true ) {
        return( Roll = ValorRC( Sensor[3].MediaDistance, 1 ) );
        }else {
         return( Roll = 0 );
        }
  }else {
    return( Roll = 0 );
  }
}
uint16_t ValorRC( uint16_t Distancia, bool Aumentar ) {
  if( Distancia < 30 ) {
    if( Aumentar == true ) {
      return( 1700 );
    }else{
      return( 1300 );
    }
  }else if( Distancia < 90 ) {
    if( Aumentar == true ) {
      return( 1675 );
    }else{
      return( 1325 );
    }
  }else if( Distancia < 150 ) {
    if( Aumentar == true ) {
      return( 1650 );
    }else{
      return( 1350 );
    }
  }
}
void CompensationInertia(){
  if(PitchOut > 1500 && Sensor[0].Active == false && Sensor[2].Active == false){
    Sensor[0].Active = true;
  }else if(PitchOut < 1500 && PitchOut != 0 && Sensor[2].Active == false && Sensor[0].Active == false){
    Sensor[2].Active = true;
  }else if(PitchOut == 0 && Sensor[0].Active == true && Sensor[0].CompensatedTime == 0){
    Sensor[0].CompensatedTime = millis();
  }else if(PitchOut == 0 && Sensor[2].Active == true && Sensor[2].CompensatedTime == 0){
    Sensor[2].CompensatedTime = millis();
  }
  if(RollOut > 1500 && Sensor[3].Active == false && Sensor[1].Active == false){
    Sensor[3].Active = true;
  }else if(RollOut < 1500 && RollOut != 0 && Sensor[1].Active == false && Sensor[3].Active == false){
    Sensor[1].Active = true;
  }else if(RollOut == 0 && Sensor[1].Active == true && Sensor[1].CompensatedTime == 0){
    Sensor[1].CompensatedTime = millis();
  }else if(RollOut == 0 && Sensor[3].Active == true && Sensor[3].CompensatedTime == 0){
    Sensor[3].CompensatedTime = millis();
  }
  for(int i = 0; i < 4; i++){
    if(Sensor[i].CompensatedTime != 0 && (Sensor[i].CompensatedTime + Compensation > millis())){
      switch(i){
        case 0:
          Pitch = 1300;
          break;
        case 1:
          Roll = 1700;
          break;
        case 2:
          Pitch = 1700;
          break;
        case 3:
          Roll = 1300;
          break;
        default:
          break;
      }
    }else if(Sensor[i].CompensatedTime != 0){
      switch(i){
        case 0:
        case 2:
          PitchOut = 0;
          Sensor[i].Active = false;
          Sensor[i].CompensatedTime = 0;
          break;
        case 1:
        case 3:
          RollOut = 0;
          Sensor[i].Active = false;
          Sensor[i].CompensatedTime = 0;
          break;
        default:
          break;
      }
    }
  }
 }
void FHT() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  mavlink_msg_HT_pack(255, 0, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 1, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
Serial.write(buf, len);
}
void RCOverride(mavlink_message_t *msg, uint16_t len, uint8_t *buf, uint16_t PitchOut, uint16_t RollOut) {
 mavlink_msg_rc_channels_override_pack(255, 0 , msg, 1, 0, RollOut, PitchOut, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, msg);
  Serial.write(buf, len);
}
