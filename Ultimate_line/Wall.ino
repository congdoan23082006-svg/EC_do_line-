

int macdinh=0;int hientai;int dem=0;
int macdinh_1=0;int hientai_1;int dem_1=0;
int macdinh_2=0;int hientai_2;int dem_2=0;

void Wall(){
  lectura();
   distance = ultrasonic.read();
  //-------------------------------------
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
//--------------------------
hientai = Dis();
 if(hientai != macdinh)
 {
  if(hientai == 1)
    {
      dem+=1;
    }
  macdinh = hientai;
 }
 //--------------------------
hientai_1 = Dis_1();
 if(hientai_1 != macdinh_1)
 {
  if(hientai_1 == 1)
    {
      dem_1+=1;
    }
  macdinh_1 = hientai_1;
 }
 //--------------------------
 hientai_2 = Dis_2();
 if(hientai_2 != macdinh_2)
 {
  if(hientai_2 == 1)
    {
      dem_2+=1;
    }
  macdinh_2 = hientai_2;
 }
//=============================
        if(hientai==1 && dem==1) {Left_25();  Forward();  }
        else if(hientai_1==1 && dem_1 ==1 ) {Right_55();  Forward(); }
        else if(hientai_1==1 && dem_1 ==2 ) {Left_80();  Forward(); }

}

//=============================================================
int Dis(){
 // if(distance > 13 && distance < 17) return 1;
}
int Dis_1(){
 // if(distance > 8 && distance < 10) return 1;
}
int Dis_2(){
//  if(distance > 10 && distance < 13) return 1;
}

int Black_all(){
      if(digital[1]&&digital[2]&&digital[3]&&digital[4]&&digital[5]&&digital[6]&&digital[7]&&digital[8]&&digital[9]&&digital[10]&&digital[11]&&digital[12]&digital[13]&&digital[14]) return 1;
}

void Control(){
      motor1.drive(200); 
      motor2.drive(100);
  // motor1.brake(); 
  // motor2.brake();
  //motor1.standby()
}
void Right_90(){
      right(motor1, motor2, 200);
     delay(250);
     brake(motor1, motor2);
   
}
void Right_55(){
      right(motor1, motor2, 150);
     delay(150);
     brake(motor1, motor2);
    
}
void Right_45(){
      right(motor1, motor2, 150);
     delay(150);
     brake(motor1, motor2);
 
}
//----------------------------------------
void Left_90(){
      left(motor1, motor2, 200);
     delay(250);
     brake(motor1, motor2);
 
}
void Left_25(){
      left(motor1, motor2, 200);
     delay(150);
     brake(motor1, motor2);
    
}
void Left_80(){
      left(motor1, motor2, 200);
     delay(200);
     brake(motor1, motor2);
}
//----------------------------------------
void Forward(){
    forward(motor1,motor2, 90);
    delay(500);
    brake(motor1, motor2);
}
void Back_1(){
    back(motor1, motor2, -90);
    //delay(1000);
    //brake(motor1, motor2);
}

