
void frenos(){

  if(pos <= 250){
    motores(veladelante, -velatras);
    delay(50);
  }
  
  if(pos >= 1050){
   while(pos<=250 || pos>=1050){
     motores(-255,245);
     lectura();
   }
   vel=100;
  }

}

void motores(int izq, int der){
 
// Serial.print("\t");
//   Serial.print(izq);
//   Serial.print("\t");
//   Serial.println(der);

//if(izq < 30 ){ motor1.brake();    motor2.drive(200);  }
//if(der < 30 ){ motor2.brake();    motor1.drive(200);  }


    motor1.drive(izq); // Quay Motor trái 
    // motor1.brake(); // Dừng Motor 1
 
    motor2.drive(der); // Quay Motor phải 
    // motor2.brake();

    // Điều khiển cả hai động cơ để tiến và sau đó dừng lại.
    //  forward(motor1,motor2,155);
    //  delay(1000);
    //  brake(motor1, motor2);
    //  delay(2000);

    // Điều khiển cả hai động cơ để lùi và sau đó dừng lại.
    // back(motor1, motor2, -255);
    // delay(1000);
    // brake(motor1, motor2);
    // delay(2000);

    // Điều khiển cả hai động cơ để quay trái và sau đó dừng lại.
    // left(motor1, motor2, 200);
    // delay(2000);
    // brake(motor1, motor2);
    // delay(1000);

    // // Điều khiển cả hai động cơ để quay phải và sau đó dừng lại.
    // right(motor1, motor2, 200);
    // delay(2000);
    // brake(motor1, motor2);
    // delay(2000);
}