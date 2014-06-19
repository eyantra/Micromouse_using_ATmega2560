#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.h"

struct block{
    int gTime;
    int str;
    int dir;
};
struct block_list{
    int posx;
    int posy;
    struct block unit;
    struct block_list * next;
};
int cell_distance = 284;
int presentx;
int presenty;
int presentdir;
int haschanged;
int A[8][8];
struct block B[7][7];
int front,left,right;
int previous_gTime,present_gTime;
struct block_list * head;
struct block_list * head2;
#define FCPU 11059200ul
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp, sharp1, sharp2, distance, adc_reading;
unsigned int value, value1, value2;
float BATT_Voltage, BATT_V;
/********************code for bot motion**************/
unsigned long int ShaftCountLeft = 0;
unsigned long int ShaftCountRight = 0;
void init_ports1(void){
		DDRA = DDRA | 0x0F;
		PORTA = PORTA & 0xF0;

		DDRL = DDRL |  0x18;
		PORTL = PORTL | 0x18;
		
        DDRE = 0x0F;
        PORTE = 0x00;
}
void left_position_encoder_interrupt_init(void){
        cli();
        EICRB = EICRB | 0x02;
        EIMSK = EIMSK | 0x10;
        sei();
}        
ISR(INT4_vect){
        ShaftCountLeft++;
}
void right_position_encoder_interrupt_init(void){
        cli();
        EICRB = EICRB | 0x08;
        EIMSK = EIMSK | 0x20;
        sei();
}        
ISR(INT5_vect){
        ShaftCountRight++;
}      
void stop(){
        PORTA = 0x00;
}
void forward(){
        PORTA = 0x06;
}
void angle_rotate(unsigned int Degrees){
        float ReqdShaftCount = 0;
        unsigned int ReqdShaftCountInt = 0;

        ReqdShaftCount = (float) Degrees/4.090;
        ReqdShaftCountInt = (unsigned int) ReqdShaftCount;

        ShaftCountLeft = 0;
        ShaftCountRight = 0;

        while(1){
                if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt)){
                        break;
                }

        }
        PORTA = 0x00;
}
void linear_distance_mm(unsigned int DistanceInMM){

        float ReqdShaftCount = 0;
        unsigned long int ReqdShaftCountInt = 0;

        ReqdShaftCount = DistanceInMM/5.338;
        ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

        ShaftCountLeft = 0;
        while(1){
                if((ShaftCountLeft > ReqdShaftCountInt) || (ShaftCountRight > ReqdShaftCountInt)) {
                        break;
                }
        }

        stop();

}
void init_devices1(){
        cli();
        init_ports1();
        left_position_encoder_interrupt_init();
        right_position_encoder_interrupt_init();
        sei();

}
void delay_motor(){
        unsigned int i;
        for(i=0;i<1;i++)
        {
                _delay_ms(2);
        }

}
/******************code for lcd print of sensors**********************/
void LCD_port_config() {
    DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
    PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0
}
void motion_pin_config(void){
    DDRA = DDRA | 0x0F;
    PORTA = PORTA & 0xF0;

    DDRL = DDRL |  0x18;
    PORTL = PORTL | 0x18;
}
void timer5_init(){
    TCCR5B = 0x00; //just for safety
    TCNT5H = 0xFF;
    TCNT5L = 0x01;
    OCR5AH = 0x00;
    OCR5AL = 0xFF;
    OCR5BH = 0x00;
    OCR5BL = 0xFF;
    OCR5CH = 0x00;
    OCR5CL = 0xFF;
    TCCR5A = 0xA9;
    TCCR5B = 0x0B;
}
void velocity( unsigned char left_motor, unsigned char right_motor){
    if(left_motor >254){
        left_motor = 255;
        }

    if(right_motor >254){
        right_motor = 255;
        }

    OCR5AL = (unsigned char) left_motor;
    OCR5BL = (unsigned char) right_motor;
}
void adc_port_config() {
    DDRF = 0x00;
    PORTF = 0x00;
    DDRK = 0x00;
    PORTK = 0x00;
}
void port_init() {
    LCD_port_config();
    adc_port_config();
}
void adc_init() {
    ADCSRA = 0X00;
    ADCSRB = 0X00;
    ADMUX = 0X20;
    ACSR = 0X80;
    ADCSRA = 0X86;
}
unsigned char ADC_Conversion(unsigned char Ch) {
    unsigned char a;
    if(Ch>7)
        ADCSRB = 0X08;
    Ch = Ch & 0x07;
    ADMUX = 0x20 | Ch;
    ADCSRA = ADCSRA | 0x40;
    while((ADCSRA & 0x10) == 0);
    a = ADCH;
    ADCSRB = 0x00;
    return a;
}
void print_sensor(char row, char column, unsigned char channel) {
    ADC_Value = ADC_Conversion(channel);
    lcd_print(row, column, ADC_Value, 3);
}
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading) {
    float distance;
    unsigned int distanceInt;
    distance = (int) (10.00 * (2799.6 * (1.00 / (pow (adc_reading,1.1546)))));
    distanceInt = (int) distance;
    if(distanceInt > 800) {
        distanceInt = 800;
    }
    return distanceInt;
}
void init_devices() {
    port_init();
    adc_init();
    cli();
    motion_pin_config();
    timer5_init();
    sei();
}
/*********************************************************************/
/********************code for bot motion**************/
void align(){
        unsigned char prox_left2 = ADC_Conversion(4);
        unsigned char prox_front2 = ADC_Conversion(6);
        unsigned char prox_right2 = ADC_Conversion(8);
		if(prox_right2 - prox_left2 < 0x14 ){
			if(prox_front2 > 0xF2){
				init_devices1();
				forward();
				linear_distance_mm(cell_distance/2);
				stop();
				PORTA = 0x0A;
				angle_rotate(0);
				delay_motor();
				stop();
			}
			else if(prox_front2 < 0xF2 && prox_front2 > 0xE1){
				init_devices1();
				forward();
				linear_distance_mm(24);
				stop();
				PORTA = 0x0A;
				angle_rotate(0);
				delay_motor();
				stop();
			}
			else return;
		}
		else if(prox_right2 - prox_left2 > 0x19 && prox_right2 - prox_left2 < 0x4B){
				PORTA = 0x0A;
				angle_rotate(3);
				stop();
				delay_motor();
				PORTA = 0x06;
				linear_distance_mm(0);
				stop();
				
				if(prox_front2 > 0xF2){
				init_devices1();
				forward();
				linear_distance_mm(cell_distance/2);
				stop();
				PORTA = 0x0A;
				angle_rotate(0);
				delay_motor();
				stop();
				}
				else if(prox_front2 < 0xF2 && prox_front2 > 0xE1){
					init_devices1();
					forward();
					linear_distance_mm(24);
					stop();
					PORTA = 0x0A;
					angle_rotate(0);
					delay_motor();
					stop();
				}
				else return;
		}
		else if(prox_left2 - prox_right2 > 0x19 && prox_left2 - prox_right2 < 0x4B){
				PORTA = 0x05;
				angle_rotate(3);
				stop();
				delay_motor();
				PORTA = 0x06;
				linear_distance_mm(0);
				stop();
				
				if(prox_front2 > 0xF2){
				init_devices1();
				forward();
				linear_distance_mm(cell_distance/2);
				stop();
				PORTA = 0x0A;
				angle_rotate(0);
				delay_motor();
				stop();
				}
				else if(prox_front2 < 0xF2 && prox_front2 > 0xE1){
					init_devices1();
					forward();
					linear_distance_mm(24);
					stop();
					PORTA = 0x0A;
					angle_rotate(0);
					delay_motor();
					stop();
				}
				else return;
		}
		else if(prox_right2 - prox_left2 > 0x4B && prox_right2 - prox_left2 < 0xB4){
				PORTA = 0x0A;
				angle_rotate(11);
				stop();
				delay_motor();
				PORTA = 0x06;
				linear_distance_mm(0);
				stop();
				
				if(prox_front2 > 0xF2){
				init_devices1();
				forward();
				linear_distance_mm(cell_distance/2);
				stop();
				PORTA = 0x0A;
				angle_rotate(0);
				delay_motor();
				stop();
				}
				else if(prox_front2 < 0xF2 && prox_front2 > 0xE1){
					init_devices1();
					forward();
					linear_distance_mm(24);
					stop();
					PORTA = 0x0A;
					angle_rotate(0);
					delay_motor();
					stop();
				}
				else return;
		}
		else if(prox_left2 - prox_right2 > 0x4B && prox_left2 - prox_right2 < 0xB4){
				PORTA = 0x05;
				angle_rotate(11);
				stop();
				delay_motor();
				PORTA = 0x06;
				linear_distance_mm(0);
				stop();
				
				if(prox_front2 > 0xF2){
				init_devices1();
				forward();
				linear_distance_mm(cell_distance/2);
				stop();
				PORTA = 0x0A;
				angle_rotate(0);
				delay_motor();
				stop();
				}
				else if(prox_front2 < 0xF2 && prox_front2 > 0xE1){
					init_devices1();
					forward();
					linear_distance_mm(24);
					stop();
					PORTA = 0x0A;
					angle_rotate(0);
					delay_motor();
					stop();
				}
				else return;
		}		
}
void move_one_unit(){
	init_devices1();
	//init_devices();
	forward();
	//velocity(20,20);
	linear_distance_mm(cell_distance/2);
	stop();
	PORTA = 0x0A;
	//velocity(20,20);
	angle_rotate(0);
	delay_motor();
	stop();
	align();
}
void turn(int turn_angle){
	init_devices1();
	if(turn_angle == 0){
		PORTA = 0x06;
		//velocity(30,30);
		linear_distance_mm(0);
		stop();
		PORTA = 0x0A;
		//velocity(30,30);
		angle_rotate(0);
		stop();
		delay_motor();
	}
	else if(turn_angle == 90){
		init_devices();
		lcd_set_4bit();
		lcd_init();
		unsigned char prox_front3 = ADC_Conversion(6);
		if(prox_front3 < 0xC8){ 
			PORTA = 0x06;
			//velocity(30,30);
			linear_distance_mm(0);
			stop();
		}
		else{
		PORTA = 0x06;
		//velocity(30,30);
		linear_distance_mm(17);
		stop();		
		}
		
		PORTA = 0x0A;
		//velocity(30,30);
		angle_rotate(80);
		stop();
		delay_motor();
		forward();
		linear_distance_mm(8);
		stop();
		PORTA = 0x0A;
		//velocity(30,30);
		angle_rotate(0);
		stop();
	}
	else if(turn_angle == 180){
		PORTA = 0x06;
		//velocity(30,30);
		linear_distance_mm(0);
		stop();
		PORTA = 0x0A;
		//velocity(30,30);
		angle_rotate(170);
		stop();
		delay_motor();
		/*forward();
		linear_distance_mm(8);
		stop();
		PORTA = 0x0A;
		angle_rotate(0);
		stop();*/
	}
	else if(turn_angle == 270){
		PORTA = 0x06;
		//velocity(30,30);
		linear_distance_mm(0);
		stop();
		PORTA = 0x05;
		//velocity(30,30);
		angle_rotate(80);
		stop();
		delay_motor();
		forward();
		linear_distance_mm(15);
		stop();
		PORTA = 0x0A;
		angle_rotate(0);
		stop();
	}
}
void move_fast(int no_units){
   
    /////////////
    if(B[presentx][presenty].dir==0){
        presentx=presentx-no_units;
        return;
    }
    if(B[presentx][presenty].dir==1){
        presenty=presenty-no_units;
        return;
    }
    if(B[presentx][presenty].dir==2){
        presentx=presentx+no_units;
        return;
    }
    if(B[presentx][presenty].dir==3){
        presenty=presenty+no_units;
        return;
    }
}
void addToList(int posx,int posy,int dir,int str,int gTime){
        ////printf("%d %d %d \n",posx,posy,dir);
        struct block_list * temp = (struct block_list*) malloc(sizeof(struct block_list));
        B[posx][posy].gTime=gTime;
        B[posx][posy].str=str;
        B[posx][posy].dir=dir;
        temp->unit.gTime = gTime;
        temp->unit.str = str;
        temp->unit.dir = dir;
        temp->posx = posx;
        temp->posy = posy;
        if(head2 == NULL){
            temp->next=NULL;
            head2 = temp;
        }
        else{
            temp->next = head2;
            head2 = temp;
        }
            /*int i,j;
                for(i=7;i>=0;i--){
                    for(j=0;j<8;j++){
                        if((B[j][i]).gTime<10)
                        //printf(" ");
                        //printf("%d,",(B[j][i]).gTime);
                       
                    }//printf("\n");
                }//printf("\n");
                usleep(500);*/
}
int reachable(int dir,int px,int py){
    if(dir==0){
        if(A[px][py]==2 || A[px][py]==3){
            return 0;
        }
        else return 1;
    }
    if(dir==1){
        if(A[px][py]==1 || A[px][py]==3){
            return 0;
        }
        else return 1;
    }
    if(dir==2){
        if(A[px+1][py]==2 || A[px+1][py]==3){
            return 0;
        }
        else return 1;
    }
    if(dir==3){
        if(A[px][py+1]==1 || A[px][py+1]==3){
            return 0;
        }
        else return 1;
    }
   
}
int nextIter(){
    while(head != NULL){
        int px = head->posx;
        int py = head->posy;
            if(reachable(0,px,py)){////printf("0 reachable %d %d \n",px,py);
                int str_temp;
                int dir_temp = 2;
                int gTime_temp;
                if(B[px][py].dir == 2) {
                    str_temp = B[px][py].str+1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu   
                    gTime_temp=1+B[px][py].gTime;
                }
                else{
                    str_temp = 1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=3+B[px][py].gTime;
                    if(px==3){
                        if(py==3){
                            gTime_temp=1+B[px][py].gTime;
                        }
                    }
                }
                if(B[px-1][py].gTime>gTime_temp)
                addToList(px-1,py,dir_temp,str_temp,gTime_temp );
            }
            if(reachable(1,px,py)){////printf("1 reachable %d %d \n",px,py);
                int str_temp;
                int dir_temp = 3;
                int gTime_temp;
                if(B[px][py].dir == 3) {
                    str_temp = B[px][py].str+1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=1+B[px][py].gTime;
                }
                else{
                    str_temp = 1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=3+B[px][py].gTime;
                    if(px==3){
                        if(py==3){
                            gTime_temp=1+B[px][py].gTime;
                        }
                    }
                }
                if(B[px][py-1].gTime>gTime_temp)
                addToList(px,py-1,dir_temp,str_temp,gTime_temp );
            }
            if(reachable(2,px,py)){////printf("2 reachable %d %d \n",px,py);
                int str_temp;
                int dir_temp = 0;
                int gTime_temp;
                if(B[px][py].dir == 0) {
                    str_temp = B[px][py].str+1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=1+B[px][py].gTime;
                }
                else{
                    str_temp = 1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=3+B[px][py].gTime;
                    if(px==3){
                        if(py==3){
                            gTime_temp=1+B[px][py].gTime;
                        }
                    }
                }
                if(B[px+1][py].gTime>gTime_temp)
                addToList(px+1,py,dir_temp,str_temp,gTime_temp );
            }
            if(reachable(3,px,py)){////printf("3 reachable %d %d \n",px,py);
                int str_temp;
                int dir_temp = 1;
                int gTime_temp;
                if(B[px][py].dir == 1) {
                    str_temp = B[px][py].str+1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=1+B[px][py].gTime;
                }
                else{
                    str_temp = 1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=3+B[px][py].gTime;
                    if(px==3){
                        if(py==3){
                            gTime_temp=1+B[px][py].gTime;
                        }
                    }
                }
                if(B[px][py+1].gTime>gTime_temp)
                addToList(px,py+1,dir_temp,str_temp,gTime_temp );
            }
        head=head->next;
    }
    return;
}
int nextIter1(){
    while(head != NULL){
        int px = head->posx;
        int py = head->posy;
            if(reachable(0,px,py)){////printf("0 reachable %d %d \n",px,py);
                int str_temp;
                int dir_temp = 2;
                int gTime_temp;
                if(B[px][py].dir == 2) {
                    str_temp = B[px][py].str+1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu   
                    gTime_temp=1+B[px][py].gTime;
                }
                else{
                    str_temp = 1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=3+B[px][py].gTime;
                    if(px==0&&py==0){
                            gTime_temp=1+B[px][py].gTime;
                    }
                }
                if(B[px-1][py].gTime>gTime_temp)
                addToList(px-1,py,dir_temp,str_temp,gTime_temp );
            }
            if(reachable(1,px,py)){////printf("1 reachable %d %d \n",px,py);
                int str_temp;
                int dir_temp = 3;
                int gTime_temp;
                if(B[px][py].dir == 3) {
                    str_temp = B[px][py].str+1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=1+B[px][py].gTime;
                }
                else{
                    str_temp = 1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=3+B[px][py].gTime;
                    if(px==0&&py==0){
                            gTime_temp=1+B[px][py].gTime;
                    }
                }
                if(B[px][py-1].gTime>gTime_temp)
                addToList(px,py-1,dir_temp,str_temp,gTime_temp );
            }
            if(reachable(2,px,py)){////printf("2 reachable %d %d \n",px,py);
                int str_temp;
                int dir_temp = 0;
                int gTime_temp;
                if(B[px][py].dir == 0) {
                    str_temp = B[px][py].str+1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=1+B[px][py].gTime;
                }
                else{
                    str_temp = 1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=3+B[px][py].gTime;
                    if(px==0&&py==0){
                            gTime_temp=1+B[px][py].gTime;
                    }
                }
                if(B[px+1][py].gTime>gTime_temp)
                addToList(px+1,py,dir_temp,str_temp,gTime_temp );
            }
            if(reachable(3,px,py)){////printf("3 reachable %d %d \n",px,py);
                int str_temp;
                int dir_temp = 1;
                int gTime_temp;
                if(B[px][py].dir == 1) {
                    str_temp = B[px][py].str+1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=1+B[px][py].gTime;
                }
                else{
                    str_temp = 1;
                    ////////////////////////////////////////////////gtime to be caluculated maa bulli chestadu
                    gTime_temp=3+B[px][py].gTime;
                    if(px==0&&py==0){
                            gTime_temp=1+B[px][py].gTime;
                    }
                }
                if(B[px][py+1].gTime>gTime_temp)
                addToList(px,py+1,dir_temp,str_temp,gTime_temp );
            }
        head=head->next;
    }
    return;
}
void varadha(){
    int i,j;
    for(i=0;i<7;i++){
        for(j=0;j<7;j++){
            (B[i][j]).gTime=99;
            (B[i][j]).str=0;
            (B[i][j]).dir=5;
        }
    }
    head = NULL;
    head2 = NULL;
    addToList(3,3,0,0,0);
    head = head2;
    head2 = NULL;
    while(1){
        int check =  nextIter();
        if(head2==NULL){
            break;
        }
        else{
            head = head2;
            head2=NULL;
        }
    }
    return;
}
void varadha1(){
    int i,j;
    for(i=0;i<7;i++){
        for(j=0;j<7;j++){
            (B[i][j]).gTime=99;
            (B[i][j]).str=0;
            (B[i][j]).dir=5;
        }
    }
    head = NULL;
    head2 = NULL;
    addToList(0,0,0,0,0);
    head = head2;
    head2 = NULL;
    while(1){
        int check =  nextIter1();
        if(head2==NULL){
            break;
        }
        else{
            head = head2;
            head2=NULL;
        }
    }
    return;
}
int goal(){
        if(presentx == 3){
            if(presenty == 3) return 1;
        }
        return 0;
}
int goal1(){
        if(presentx == 0 && presenty == 0){
            return 1;
        }
        return 0;
}
void get_sensor_values(){
    init_devices();
    lcd_set_4bit();
    lcd_init();
    sharp = ADC_Conversion(9);
    value = Sharp_GP2D12_estimation(sharp);
	sharp1 = ADC_Conversion(11);
    value1 = Sharp_GP2D12_estimation(sharp1);
	sharp2 = ADC_Conversion(13);
    value2 = Sharp_GP2D12_estimation(sharp2);
    unsigned char prox_left = ADC_Conversion(4);
    unsigned char prox_front = ADC_Conversion(6);
    unsigned char prox_right = ADC_Conversion(8);
	
	if(prox_left < 0xE6)
		left = 1;
	else if(value < 270)
		left = 1;
	else left = 0;
	
	if(prox_front < 0xE6)
		front = 1;
	else if(value1 < 200)
		front = 1;
		else front = 0;
		
	if(prox_right < 0xE6)
		right = 1;
	else if(value2 < 270)
		right = 1;
		else right = 0;
	lcd_print(1,6,prox_left,3);
	lcd_print(1,10,prox_front,3);
	lcd_print(1,14,prox_right,3);
	lcd_print(2,6,value,3);
	lcd_print(2,10,value1,3);
	lcd_print(2,14,value2,3);
	delay_motor();
		
}
void update_walls(){
    ////printf("curr pos is %d, %d, %d, ----enter new walls if any\n",presentx,presenty,presentdir);
    front=0;
    left=0;
    right=0;
    get_sensor_values();
    //scanf("%d %d %d",&front,&left,&right);
   
    if(presentdir==0){
        if(front==1 && left==1){
            A[presentx][presenty]=3;
        }
        else if(front ==1 && left ==0){
            A[presentx][presenty]=2;
        }
        else if(front ==0 && left ==1){
            A[presentx][presenty]=1;
        }
        else{
            A[presentx][presenty]=0;
        }
       
        if(right==1){
            if(A[presentx][presenty+1]==2 || A[presentx][presenty+1]==3){
                A[presentx][presenty+1]=3;
            }
            else{
                A[presentx][presenty+1]=1;
            }
        }
        else{
       
        }
           
    }
    if(presentdir==1){
        if(front==1 && right==1){
            A[presentx][presenty]=3;
        }
        else if(front ==1 && right ==0){
            A[presentx][presenty]=1;
        }
        else if(front ==0 && right ==1){
            A[presentx][presenty]=2;
        }
        else{
            A[presentx][presenty]=0;
        }
       
        if(left==1){
            if(A[presentx+1][presenty]==1 || A[presentx+1][presenty]==3){
                A[presentx+1][presenty]=3;
            }
            else{
                A[presentx+1][presenty]=2;
            }
        }
        else{
       
        }
           
    }
    if(presentdir==2){
        if(front==1){
            if(A[presentx+1][presenty] ==1 || A[presentx+1][presenty] ==3){
                A[presentx+1][presenty] =3;
            }
            else{
                A[presentx+1][presenty]=2;
            }
        }
        if(right==1){
            if(A[presentx][presenty]==2 || A[presentx][presenty]==3){
                A[presentx][presenty] =3;
            }
            else{
                A[presentx][presenty]=1;
            }
        }
        if(left==1){
            if(A[presentx][presenty+1] ==2 || A[presentx][presenty+1] ==3){
                A[presentx][presenty+1] =3;
            }
            else{
                A[presentx][presenty+1]=1;
            }
        }
           
    }
    if(presentdir==3){
        if(front==1){
            if(A[presentx][presenty+1] ==2 || A[presentx][presenty+1] ==3){
                A[presentx][presenty+1] =3;
            }
            else{
                A[presentx][presenty+1]=1;
            }
        }
        if(right==1){
            if(A[presentx+1][presenty]==1 || A[presentx+1][presenty]==3){
                A[presentx+1][presenty] =3;
            }
            else{
                A[presentx+1][presenty]=2;
            }
        }
        if(left==1){
            if(A[presentx][presenty] ==1 || A[presentx][presenty] ==3){
                A[presentx][presenty] =3;
            }
            else{
                A[presentx][presenty]=2;
            }
        }
           
    }
    init_devices();
    lcd_set_4bit();
    lcd_init();
	lcd_print(2,6,A[presentx][presenty],1);
	lcd_print(2,10,A[presentx+1][presenty],1);
	lcd_print(2,14,A[presentx][presenty+1],1);
	delay_motor();
}
int orient(int o_dir){
    /*int turn_angle=90*(o_dir-presentdir);
    turn(turn_angle);
    presentdir=o_dir;*/
    init_devices();
    lcd_set_4bit();
    lcd_init();
	lcd_print(1,6,presentx,1);
	lcd_print(1,10,presenty,1);
	lcd_print(2,6,o_dir,2);
	lcd_print(2,10,presentdir,2);
	
	int turn_angle;
	if(o_dir-presentdir == 0){
		turn_angle = 0;}
	else if(o_dir-presentdir == 1){
		turn_angle = 270;}
	else if(o_dir-presentdir == -1){
		turn_angle = 90;}
	else if(o_dir-presentdir == 2){
		turn_angle = 180;}
	else if(o_dir-presentdir == -2){
		turn_angle = 180;}
	else if(o_dir-presentdir == 3){
		turn_angle = 90;}
	else if(o_dir-presentdir == -3){
		turn_angle = 270;}
	lcd_print(2,14,turn_angle,3);
	delay_motor();
	turn(turn_angle);
	presentdir=o_dir;
    if(turn_angle==0)return 0;
    else return 1;
}
void findnext(){
    /*init_devices();
    lcd_set_4bit();
    lcd_init();*/
	delay_motor();
    if(B[presentx][presenty].dir==0){
        presentx--;
        move_one_unit();
        //printf("%d,%d\n",presentx,presenty);
		//lcd_print(1,2,presentx,1);
		//lcd_print(1,6,presenty,1);
        return;
    }
    if(B[presentx][presenty].dir==1){
		presenty--;
        move_one_unit();
		//lcd_print(1,2,presentx,1);
		//lcd_print(1,6,presenty,1);
        //printf("%d,%d\n",presentx,presenty);
        return;
    }
    if(B[presentx][presenty].dir==2){
        presentx++;
        move_one_unit();
		//lcd_print(1,2,presentx,1);
		//lcd_print(1,6,presenty,1);
        //printf("%d,%d\n",presentx,presenty);
        return;
    }
    if(B[presentx][presenty].dir==3){
        presenty++;
        move_one_unit();
		//lcd_print(1,2,presentx,1);
		//lcd_print(1,6,presenty,1);
        //printf("%d,%d\n",presentx,presenty);
        return;
    }
}
void continuous_motion(){

    while(1){
        //printf("%d, %d, %d\n",presentx,presenty,B[presentx][presenty].str);
        orient(B[presentx][presenty].dir);
        move_fast(B[presentx][presenty].str);
        int goal_check = goal();
        if(goal_check==1)break;
    }
    //printf("%d, %d, %d\n",presentx,presenty,B[presentx][presenty].str);
}
int main(){
	int i,j;
    for(i=0;i<=7;i++){
        for(j=0;j<=7;j++){
            A[i][j]=0;
        }
    }
    for(i=0;i<7;i++){
        for(j=0;j<7;j++){
            (B[i][j]).gTime=999;
            (B[i][j]).str=0;
            (B[i][j]).dir=0;
        }
    }
    for(i=0;i<=7;i++){
        A[0][i]=2;
        A[i][0]=1;
        A[i][7]=1;
        A[7][i]=2;
    }
    A[0][0]=3;
    A[0][7]=3;
    A[7][0]=2;
    ///////////////
	//A[1][0] = 3;
    /*A[2][0]=3;
    A[1][1]=3;
    A[3][3]=3;
    A[4][3]=3;
    A[5][3]=2;
    A[3][4]=3;
    A[3][5]=1;
    A[4][5]=1;
    A[3][3] = 3;
    A[3][4] = 3;
    A[4][3] = 1;
    A[5][2] = 2;
    A[1][5] = 2;
    A[5][5] = 2;
    A[5][4] = 1;
    A[2][6] = 2;*/
   
    ///////////////
    presentx=0;
    presenty=0;
    presentdir=3;
    haschanged=1;
    previous_gTime=99;
    while(1){
        while(1){
				update_walls();
                varadha();
                /*int i,j;
                //printf("after varadha gtime and dir\n\n");
                for(i=8;i>=0;i--){
                    for(j=0;j<=8;j++){
                        //printf("%d ",A[j][i]);
                    }//printf("\n");
                }
                for(i=7;i>=0;i--){
                    for(j=0;j<8;j++){
                        if((B[j][i]).gTime<10)
                        //printf(" ");
                        //printf("%d,",(B[j][i]).gTime);
                       
                    }//printf("\n");
                }//printf("\n");
                for(i=7;i>=0;i--){
                    for(j=0;j<8;j++){
                        if((B[j][i]).gTime<10)
                        //printf(" ");
                        //printf("%d,",(B[j][i]).dir);
                       
                    }//printf("\n");
                }//printf("\n");*/
            orient(B[presentx][presenty].dir);
            findnext();
			delay_motor();
            int goal_0 = goal();
            if(goal_0 == 1) break;
        }
        //printf("out1\n");
        while(1){
				update_walls();
                varadha1();
                /*int i,j;
                //printf("after varadha1 gtime and dir\n\n");
                for(i=8;i>=0;i--){
                    for(j=0;j<=8;j++){
                        //printf("%d ",A[j][i]);
                    }//printf("\n");
                }
                for(i=7;i>=0;i--){
                    for(j=0;j<8;j++){
                        if((B[j][i]).gTime<10)
                        //printf(" ");
                        //printf("%d,",(B[j][i]).gTime);
                       
                    }//printf("\n");
                }//printf("\n");
                for(i=7;i>=0;i--){
                    for(j=0;j<8;j++){
                        if((B[j][i]).gTime<10)
                        //printf(" ");
                        //printf("%d,",(B[j][i]).dir);
                       
                    }//printf("\n");
                }//printf("\n");*/
                haschanged=1;
            orient(B[presentx][presenty].dir);
            findnext();
			delay_motor();
            update_walls();
			delay_motor();
            int goal_1 = goal1();
            if(goal_1 == 1) break;
        }
        varadha();
        present_gTime=B[0][0].gTime;
        if(previous_gTime==present_gTime)break;
        else previous_gTime=present_gTime;
    }
    continuous_motion();
    return 0;
   
}
