#include "motors.h"

/*
 * motors.c
 *
 *  Created on: Apr 8, 2025
 *      Author: Kristopher Lima
 */

volatile int leftReset = 0;
volatile int rightReset = 0;
volatile uint16_t modeLeft = 0;
volatile uint16_t modeRight = 0;

//fever is 1 , multi is 2, base is 0
extern volatile uint8_t restock;


//May need to be positioned inside of main.c since its overriding the weak definition of this function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	//left motor
	if (GPIO_Pin == GPIO_PIN_13) {
		runMotor(0);
		restock = 0;

		//This is where it resets back to initial position
		if(leftReset == 7){
			restock = 2;
			HAL_Delay(2000);
			runMotor(3);
			leftReset = 0;

		}
		++leftReset;

	}

	//right motor
	if (GPIO_Pin == GPIO_PIN_12) {
			runMotor(1);
			restock = 0;

			//This is where it resets back to initial position
			if(rightReset == 7){
				restock = 1;
				HAL_Delay(2000);
				runMotor(4);
				rightReset = 0;
			}
			++rightReset;
		}
}

void runMotor(uint8_t left_0_right_1 ){

	//2 m seconds event happening here
			if(left_0_right_1 == 0){

				for(int i = 0 ; i < 256 ;++i){
					HAL_Delay(2);


						if (modeLeft == 0){
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
						}
						else if(modeLeft == 1){
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
						}
						else if(modeLeft == 2){
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
						}
						else{//mode = 3
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
						}


						modeLeft = (modeLeft + 1)%4;

				}

			}

			else if(left_0_right_1 == 1){
				for(int i = 0 ; i < 256 ;++i){

					HAL_Delay(2);


						if (modeRight == 0){
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, 1);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, 0);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 0);
							HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 0);
						}
						else if(modeRight == 1){
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, 0);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, 1);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 0);
							HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 0);
						}
						else if(modeRight == 2){
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, 0);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, 0);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 1);
							HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 0);
						}
						else{//mode = 3
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, 0);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, 0);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 0);
							HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 1);
						}

						//step_count++;
						modeRight = (modeRight + 1)%4;


				}
			}

			else if(left_0_right_1 == 3){
				for(int i = 0 ; i < 1792 ;++i){

					HAL_Delay(2);


					if (modeLeft == 0){
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
					}
					else if(modeLeft == 1){
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
					}
					else if(modeLeft == 2){
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
					}
					else{//mode = 3
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
					}

						//step_count++;
					modeLeft = (modeLeft + 1)%4;


				}
			}

			else if(left_0_right_1 == 4){
				for(int i = 0 ; i < 1792 ;++i){

					HAL_Delay(2);


						if (modeRight == 0){
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, 0);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, 0);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 0);
							HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 1);
						}
						else if(modeRight == 1){
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, 0);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, 0);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 1);
							HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 0);
						}
						else if(modeRight == 2){
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, 0);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, 1);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 0);
							HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 0);
						}
						else{//mode = 3
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, 1);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, 0);
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 0);
							HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 0);
						}

						//step_count++;
						modeRight = (modeRight + 1)%4;


				}
			}
}




