void fondos(){
  for(int i=1; i<=14; i++){
  lectura_fondo[i]=map(mux.read(i),0,4095, 0, 1023);

  Serial.print(lectura_fondo[i]);
  Serial.print("\t");
}
Serial.println(" ");
}
//--------------------------------
void lineas(){
  for(int i=1; i<=14; i++){
  lectura_linea[i]=map(mux.read(i),0,4095, 0, 1023);

  Serial.print(lectura_linea[i]);
  Serial.print("\t");
}
Serial.println(" ");
}
//---------------------------------
void promedio(){
  for(int i=1;i<=14;i++){
  umbral[i]= (lectura_fondo[i]+lectura_linea[i]+200) / 2;
//umbral[i]=650
  Serial.print(umbral[i]);
  Serial.print("\t");
}
Serial.println(" ");
}

//-------------------------------

int lectura(void){
  
for(int i=1; i<=14 ;i++){
  sensores[i] = map(mux.read(i),0,4095, 0, 1023);

  if(linea==0){if(sensores[i]<=umbral[i]){digital[i]=0;} else{digital[i]=1;}}

  //if(linea==1){if(sensores[i]<=umbral[i]){digital[i]=1;} else{digital[i]=0;}}
  //if(sensores[i]>=800)  {digital[i]=1;}    else{digital[i]=0;}

 // Serial.print(digital[i]);
//  Serial.print(sensores[i]);
 // Serial.print("\t");

} //Serial.println();

  if(barra==1){sumap=(1300*digital[14]+1200*digital[13]+1100*digital[12]+1000*digital[11]+900*digital[10]+800*digital[9]+700*digital[8]+600*digital[7]+500*digital[6]+400*digital[5]+300*digital[4]+200*digital[3]+100*digital[2]+0*digital[1]);}

  suma=(digital[1]+digital[2]+digital[3]+digital[4]+digital[5]+digital[6]+digital[7]+digital[8]+digital[9]+digital[10]+digital[11]+digital[12]+digital[13]+digital[14]);

    if (suma != 0 || sumap!=0 ) {

        pos=(sumap/suma);
        
    }
    else if(suma==0) {
        pos = 650;
    }

    if(poslast<=100 && pos==-1){
      pos=0;
    }
    if(poslast>=1200 && pos==-1){
      pos=1300;
    }

  poslast=pos; //Serial.print(pos);  Serial.print("\t");
  return pos;

}
