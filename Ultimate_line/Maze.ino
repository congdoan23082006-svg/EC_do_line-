int macdinh_BF = 0;int hientai_BF;int dem_BF=0;

int macdinh_R = 0;int hientai_R=0;int dem_R=0;
int macdinh_L = 0;int hientai_L;int dem_L=0;
int macdinh_B = 0;int hientai_B;int dem_B=0;

void maze(){
  lectura();
  //-----------------------------------------------------------------
  hientai_R = (digital[7] | digital[8]) & (digital[1] | digital[2]);//right_T();
 // Serial.print("hientai_R= "); Serial.print(hientai_R);
 if(hientai_R != macdinh_R)
 {
  if(hientai_R == 1)
    {
      dem_R+=1;
    }
  macdinh_R = hientai_R;
 }//  Serial.print("      "); Serial.println(dem_R);
 
 //--------------------------------------------------------------------
   hientai_L = (digital[7] | digital[8]) & (digital[13] | digital[14]); 
   Serial.print("hientai_L= "); Serial.print(hientai_L);
 if(hientai_L != macdinh_L)
 {
  if(hientai_L == 1)
    {
      dem_L+=1;
    }
  macdinh_L = hientai_L;
 }Serial.print("      "); Serial.println(dem_L);
 //------------------------------------------------------------------
    hientai_B = digital[4] & digital[5] & digital[6] & digital[7] & digital[8] & digital[9] & digital[10];
   // Serial.print("hientai_B= "); Serial.print(hientai_B);
 if(hientai_B != macdinh_B)
 {
  if(hientai_B == 1)
    {
      dem_B+=1;
    }
  macdinh_B = hientai_B;
 }//Serial.print("      "); Serial.println(dem_B);
 //====================================================================

  //if(hientai_R==1 && dem_R==0)  Forward1();
  
  if(hientai_R==1 && dem_R==1)    Right1_90();

  //else if(hientai_R==1 && dem_R==2) Forward1();

  else if(hientai_B==1 && dem_B==1) Left1_90();

  else if(hientai_L==1 && dem_L==0) Left1_90();

  else if(hientai_L==1 && dem_L==1) Forward1();

  else PID();


}

//----------------------------------------------------
  int forward_T(){  if(digital[1] && digital[2] && digital[13] && digital[14]) return 1;}
  int left_T(){     if(digital[7] && digital[8] && digital[13] && digital[14]) return 1;}
  int right_T(){    if(digital[7] && digital[8] && digital[1]  && digital[2])  return 1;}
  //--------------------------------------------------

  int ngang_B(){  if(digital[5] && digital[6] && digital[7] && digital[8] && digital[9] && digital[10]) return 1; }
  int ngang_W(){ if(!digital[5] && !digital[6] && !digital[7] && !digital[8] && !digital[9] && !digital[10]) return 1; }
//-----------------------------------------------------------

void Left1_90(){
  Serial.println("   Left1_90   ");
      left(motor1, motor2, 210);
     delay(250);
     brake(motor1, motor2);
    // delay(1000);
  }
void Right1_90(){
  Serial.println("   Right1_90   ");
     right(motor1, motor2, 210);
     delay(250);
     brake(motor1, motor2);
  }
void Forward1(){
  Serial.println("   Forward1   ");
      forward(motor1,motor2,95);
      delay(100);
      brake(motor1, motor2);
  }
