#include <DueTimer.h>

unsigned long valor = 0;
float corrente_lida = 0;
float fator_conv_ad = (3.3/1024.0);
float fator_conv_corr = 66.0;

void setup() {
  Timer1.attachInterrupt(InterruptHandler).start(100);
  // Configura a funcaoo InterruptHandler() como a funcao 
  // para ser chamada a cada interrupcaoo do Timer1

}

void loop() {
  // put your main code here, to run repeatedly:

}

void InterruptHandler(){
  valor = analogRead(A0);
  corrente_lida = (fator_conv_ad*valor); // Utilizar o fator de conversao do A/D
  corrente_lida = corrente_lida - 1.65; //Retirar o Offset DC de 3.3/2 V
  corrente_lida = corrente_lida*1000; // Transformar a corrente lida em mA
  corrente_lida = corrente_lida/66.0; // Utilizar o fator de conversao do sensor
}
