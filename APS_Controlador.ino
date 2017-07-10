/* Marllon Frizzo & Carlos Alexandre Peron */

#ifndef _DEF_PRINCIPAIS_H
#define _DEF_PRINCIPAIS_H

#define F_CPU 16000000UL  //define a frequencia do microcontrolador - 16MHz

#include <avr/io.h>       //definições do componente especificado
#include <util/delay.h>   //biblioteca para o uso das rotinas de _delay_ms e _delay_us()
#include <avr/interrupt.h>
#include <avr/pgmspace.h>   //para o uso do PROGMEM, gravação de dados na memória flash

//Definições de macros para o trabalho com bits

#define set_bit(y,bit)  (y|=(1<<bit)) //coloca em 1 o bit x da variável Y
#define clr_bit(y,bit)  (y&=~(1<<bit))  //coloca em 0 o bit x da variável Y
#define cpl_bit(y,bit)  (y^=(1<<bit)) //troca o estado lógico do bit x da variável Y
#define tst_bit(y,bit)  (y&(1<<bit))  //retorna 0 ou 1 conforme leitura do bit

#endif

#ifndef _USART_H
#define _USART_H

#define BAUD   9600    //taxa de 9600 bps
#define MYUBRR  F_CPU/16/BAUD-1

#define tam_vetor 5 //número de digitos individuais para a conversão por ident_num()   
#define conv_ascii  48  //48 se ident_num() deve retornar um número no formato ASCII (0 para formato normal)

#define TOP 39999

void USART_Inic(unsigned int ubbr0);
void USART_Transmite(unsigned char dado);
unsigned char USART_Recebe();
void escreve_USART(char *c);
void escreve_USART_Flash(const char *c);

void ident_num(unsigned int valor, unsigned char *disp);

#endif

void USART_Inic(unsigned int ubrr0)
{
  UBRR0H = (unsigned char)(ubrr0>>8); //Ajusta a taxa de transmissão
  UBRR0L = (unsigned char)ubrr0;

  UCSR0A = 0;//desabilitar velocidade dupla (no Arduino é habilitado por padrão)
  UCSR0B = (1<<RXEN0)|(1<<TXEN0); //Habilita a transmissão e a recepção
  UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);/*modo assíncrono, 8 bits de dados, 1 bit de parada, sem paridade*/
}
//---------------------------------------------------------------------------
void USART_Transmite(unsigned char dado)
{
  while (!( UCSR0A & (1<<UDRE0)) ); //espera o dado ser enviado
  UDR0 = dado;          //envia o dado
}
//---------------------------------------------------------------------------
unsigned char USART_Recebe()
{
  while (!(UCSR0A & (1<<RXC0)));  //espera o dado ser recebido
  return UDR0;        //retorna o dado recebido
}
//---------------------------------------------------------------------------
void escreve_USART(char *c)   //escreve String (RAM)
{
   for (; *c!=0;c++) USART_Transmite(*c);
}
//---------------------------------------------------------------------------
void escreve_USART_Flash(const char *c) //escreve String (Flash)
{
   for (;pgm_read_byte(&(*c))!=0;c++) USART_Transmite(pgm_read_byte(&(*c)));
}
//--------------------------------------------------------------------------- 
//Conversão de um número em seus digitos individuais
//---------------------------------------------------------------------------
void ident_num(unsigned int valor, unsigned char *disp)
{   
  unsigned char n;

  for(n=0; n<tam_vetor; n++)
    disp[n] = 0 + conv_ascii; //limpa vetor para armazenagem dos digitos 

  do
  {
    *disp = (valor%10) + conv_ascii; //pega o resto da divisao por 10 
    valor /=10;            //pega o inteiro da divisão por 10
    disp++;

  }while (valor!=0);
}

/* Programa Principal */

unsigned int temp;
unsigned char digitos[tam_vetor];

int main()
{
  // Pinos OC1A e 0C1B (PB1 e PB2) como saída
  // Servo no pino PB2

  unsigned char dado_recebido;
  
  //DDRB  = 0b00000110;
  //PORTB = 0b11111001;  
  DDRB  = 0b000100;
  PORTB = 0b111011;
  ICR1 = TOP;

  // Posiciona o servo em 0 graus
  OCR1B = 1092;
  
  TCCR1A = (1 << WGM11); //00000010
  TCCR1B = (1 << WGM13) | (1<<WGM12) | (1 << CS11); //00011010
          
  set_bit(TCCR1A,COM1A1);
  set_bit(TCCR1A,COM1B1);
    
  USART_Inic(MYUBRR);
  
  //configura ADC
  //ADMUX  = 0b11000000;  //Tensão interna de ref (1.1V), canal 0 
  //ADMUX =  (1<<REFS1)|(1<<REFS0);
  //ADCSRA = 0b10000111;  //habilita o AD, prescaler = 128
  ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
 
  while(1)
  {
    do
    {
      //_delay_ms(500);
      dado_recebido = USART_Recebe();
      //_delay_ms(100);
      if (dado_recebido == 'A') {
        //Sensor de temperatura LM35
        ADMUX = (1<<REFS1)|(1<<REFS0);
        //_delay_ms(100);
        
        temp = (unsigned int)le_adc(0);
        temp = temp + (temp*19)/256;
        ident_num(temp,digitos);
        USART_Transmite(digitos[2]);
        USART_Transmite(digitos[1]);
        USART_Transmite(176);//simbolo 'o'
        USART_Transmite('C');
        //_delay_ms(100);
      } else if (dado_recebido == 'B') {
        ADMUX = (0<<REFS1)|(1<<REFS0);
        //_delay_ms(100); 
        
        //Luminosidade
        ident_num((unsigned int)le_adc(1),digitos);
        USART_Transmite(digitos[3]);
        USART_Transmite(digitos[2]);
        USART_Transmite(digitos[1]);
        USART_Transmite(digitos[0]);
        //_delay_ms(100);
        
      } else if (dado_recebido == 'C') {
        ADMUX = (0<<REFS1)|(1<<REFS0);
        //_delay_ms(100);
        
        // Leitura do Potenciômetro
        unsigned int potenciometro = (unsigned int)le_adc(2);

        if (potenciometro <= 204) {
          OCR1B = 1092;
          USART_Transmite('0');
          //_delay_ms(100);
        } else if (potenciometro <= 408) {
          OCR1B = 1900;
          USART_Transmite('1');
          //_delay_ms(100);
        } else if (potenciometro <= 612) {
          OCR1B = 2650;
          USART_Transmite('2');
          //_delay_ms(100);
        } else if (potenciometro <= 816) {
          OCR1B = 3800;
          USART_Transmite('3');
          //_delay_ms(100);
        } else if (potenciometro <= 1023) {
          OCR1B = 5138;
          USART_Transmite('4');
          //_delay_ms(100);
        } 
      }  
    }while(1);  
  }
}

signed int le_adc(unsigned char canal)
{
  ADMUX &= 0xF0; //Limpar o canal lido anteriormente
  ADMUX |= canal; //Define o novo canal a ser lido
  //ADCSRA |= (1<<ADSC);
  set_bit(ADCSRA, ADSC);                //inicia a conversão
  //while(ADCSRA & (1<<ADSC));
  while(tst_bit(ADCSRA,ADSC));          //espera a conversão ser finalizada
  //return (ADC + (ADC*19)/256);             //fator k de divisão = 1
  return ADC;  
}