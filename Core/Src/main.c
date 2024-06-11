/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define true 1
#define false 0

// Servo Registers
#define SMS_STS_ID				5 // For setting ID
#define SCSCL_GOAL_POSITION_L	42
#define SMS_STS_LOCK			55

// Servo Commands
#define INST_WRITE	0x03

#define HIP_OFFSET		30 // mm - the distance from yaw to pitch axes in hip joint
#define THIGH_LENGTH	100 // mm
#define CALF_LENGTH		120 // mm
#define PI	3.14159265f

// Calibration mode sets all the servos to zero point, calibrate for legs straight out
#define CALIBRATION_MODE	false

// Leg enum for arrays etc
enum { FRONT_LEFT, MID_LEFT, REAR_LEFT, REAR_RIGHT, MID_RIGHT, FRONT_RIGHT };

enum { RED, GREEN, BLUE }; // Eye colours

enum { WALK_TYPE, LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y }; // Walk type and joystick axes, as received from controller

// Different walking types are sent as bitfield, first byte of command message
#define WIGGLE_BIT		0b00000001 // Bit 0 for wiggle vs walk mode
#define HIGH_STEP_BIT	0b00000010 // Bit 1 for normal or high step
#define HIGH_BODY_BIT	0b00000100 // Bit 2 for normal or high body
#define QUICK_STEP_BIT	0b00001000 // Bit 3 for normal or quick (shorter) steps
#define RIPPLE_BIT		0b00010000 // Bit 4 to select ripple gait instead of tripod

typedef struct
{
	float x;
	float y;
	float z;
} Point;
Point footNeutralPoint[6];

typedef struct
{
	float x;
	float y;
	float z;
	float yaw;
} HipLocation;
HipLocation hipLocation[6];

// A couple of legs are assembled with 2nd and 3rd servos mirrored for better cable routing
float directions[19] = { 0, 1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1, -1 };

float calibrations[19] = { 0, -4, 1, 9, -4, 2, 14, -1, -3, 8, -2, -4, 12, -3, 0, 11, 1, 0, 6 };

typedef struct
{
	float minAngle;
	float maxAngle;
} ServoLimits;
ServoLimits servo[19]; // 19 because servo IDs start from 1; 0 will be unused

uint8_t rxData[10];  //  creating a buffer of 10 bytes
uint8_t charReceived = 0;

uint8_t controller[5] = { 0, 128, 128, 128, 128 }; // Initialise with default walk mode and middle joystick axes
uint8_t controllerUpdate[6] = { 0, 0, 0, 0, 0, 0 }; // Cache and verify checksum before updating values above
int8_t cIndex = -1;

void SetEyeColour(char colour)
{
	HAL_GPIO_WritePin(GPIOC, EYES_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EYES_G_Pin|EYES_B_Pin, GPIO_PIN_RESET);

	if (colour == RED)
		HAL_GPIO_WritePin(GPIOC, EYES_R_Pin, GPIO_PIN_SET);
	else if (colour == GREEN)
		HAL_GPIO_WritePin(GPIOA, EYES_G_Pin, GPIO_PIN_SET);
	else if (colour == BLUE)
		HAL_GPIO_WritePin(GPIOA, EYES_B_Pin, GPIO_PIN_SET);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (rxData[0] == 'c' && cIndex < 0) // Start of controller message
	{
		cIndex = 0;
	}
	else if (cIndex >= 0)
	{
		controllerUpdate[cIndex] = rxData[0];
		if (cIndex == 5) // Just wrote checksum to array
		{
			if ((uint8_t)(controllerUpdate[0] + controllerUpdate[1] + controllerUpdate[2] + controllerUpdate[3] + controllerUpdate[4]) == controllerUpdate[5])
				for (int n=0; n<5; n++)
					controller[n] = controllerUpdate[n];
			cIndex = -1;
		}
		else
			cIndex++;
	}
	else
		charReceived = rxData[0];

	HAL_UART_Receive_IT(huart, rxData, 1);
}



float DegreesToRadians(float angle)
{
	return angle*PI/180.0f;
}

float RadiansToDegrees(float angle)
{
	return angle*180.0f/PI;
}

void SetServoLimits(uint8_t id, float minAngle, float maxAngle)
{
	servo[id].minAngle = minAngle;
	servo[id].maxAngle = maxAngle;
}

void SetHipLocation(uint8_t leg, float x, float y, float z, float yaw)
{
	hipLocation[leg].x = x;
	hipLocation[leg].y = y;
	hipLocation[leg].z = z;
	hipLocation[leg].yaw = yaw;

	float length = 150; // mm from hip yaw joint
	if (leg == FRONT_LEFT || leg == REAR_RIGHT) yaw += 10; // Space front and rear legs out by 10 degrees
	if (leg == REAR_LEFT || leg == FRONT_RIGHT) yaw -= 10;
	footNeutralPoint[leg].x = x + length*sinf(DegreesToRadians(yaw));
	//footNeutralPoint[leg].y = -100; // TODO: Parameterise? #define?
	footNeutralPoint[leg].z = z + length*cosf(DegreesToRadians(yaw));
}

char WriteToServo(uint8_t id, uint8_t address, uint8_t* data, uint8_t length)
{
	uint8_t header[6];
	header[0] = 0xFF;
	header[1] = 0xFF;
	header[2] = id;
	header[3] = 3 + length;
	header[4] = INST_WRITE;
	header[5] = address;
	HAL_UART_Transmit(&huart2, header, sizeof(header), HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart2, data, length, HAL_MAX_DELAY);

	uint8_t checksum = header[2] + header[3] + header[4] + header[5];
	for (int n=0; n<length; n++)
		checksum += data[n];
	checksum = ~checksum;
	HAL_UART_Transmit(&huart2, &checksum, 1, HAL_MAX_DELAY);

	// Check reply / ack.. for some reason we have an extra byte at the start, not sure why, maybe tail end of TX in half duplex?
	uint8_t reply[10];
	if (HAL_UART_Receive(&huart2, reply, 7, 100) == HAL_OK)
	{
		// Bytes are 2x 0xFF header, ID, message length (2), result (0 for success), checksum
		checksum = ~(reply[3] + reply[4] + reply[5]);
		return !(reply[1] != 0xFF
				|| reply[2] != 0xFF
				|| reply[3] != id
				|| reply[4] != 2
				|| reply[5] != 0 // success status
				|| reply[6] != checksum); // true for success, false for fail
	}
	else
		return false;
}

char MoveServo(uint8_t id, float angle, uint16_t speed)
{
	if (angle < servo[id].minAngle) angle = servo[id].minAngle;
	if (angle > servo[id].maxAngle) angle = servo[id].maxAngle;
	angle += calibrations[id];

	uint16_t position = (uint16_t)(2048.0f + angle / 90.0f * 1024.0f * directions[id]); // Servos do 90 degrees per 1024 resolution counts
	// Speed is steps per second, about 3000 maximum for LS servo, 6000 for HS servo, use 0 for maximum

	uint8_t data[6];
	data[0] = position&0xFF;
	data[1] = position>>8;
	data[2] = 0; // These two bytes are for "time" but don't seem to do anything
	data[3] = 0;
	data[4] = speed&0xFF;
	data[5] = speed>>8;
	return WriteToServo(id, SCSCL_GOAL_POSITION_L, data, sizeof(data)); // true for success, false for fail
}

char SetServoID(uint8_t oldID, uint8_t newID)
{
	// unlock eprom writes 0 to SMS_STS_LOCK, lock writes 1 to it
	uint8_t lock = 0;
	uint8_t unlock = 1;
	WriteToServo(oldID, SMS_STS_LOCK, &lock, 1);
	WriteToServo(oldID, SMS_STS_ID, &newID, 1);
	return WriteToServo(newID, SMS_STS_LOCK, &unlock, 1);
}

void MoveLeg(uint8_t leg, Point target) // leg is enum 0-5, target is absolute coords relative to centre of body
{
	// Transform requested coordinate into leg's reference frame.. Translation first
	target.x -= hipLocation[leg].x;
	target.z -= hipLocation[leg].z;
	// Then rotation [ cos(θ) −sin(θ) | sin(θ) cos(θ) ]
	float angle = DegreesToRadians(hipLocation[leg].yaw);
	float x = target.x * cosf(angle) - target.z * sinf(angle);
	float z = target.x * sinf(angle) + target.z * cosf(angle);
	float y = target.y;

	// Inverse Kinematics to get joint angles
	float yaw = atanf(x/z);
	float shadowLength = sqrtf(x*x+z*z) - HIP_OFFSET;
	float linearDistanceToFoot = sqrtf(shadowLength*shadowLength + y*y);
	float angleOfElevationToFoot = atanf(y/shadowLength);

	// Law of cosines: angle = acosf( ( a^2 + b^2 - c^2) / 2ab )
	// where a and b are adjacent sides to angle, c is opposite
	float insideAngle = acosf((THIGH_LENGTH*THIGH_LENGTH + linearDistanceToFoot*linearDistanceToFoot - CALF_LENGTH*CALF_LENGTH) / (2*THIGH_LENGTH*linearDistanceToFoot));
	float hipAngle = angleOfElevationToFoot + insideAngle;
	float kneeAngle = PI - acosf((THIGH_LENGTH*THIGH_LENGTH + CALF_LENGTH*CALF_LENGTH - linearDistanceToFoot*linearDistanceToFoot) / (2*THIGH_LENGTH*CALF_LENGTH));

	// Send angles to servos
	MoveServo(leg*3+1, RadiansToDegrees(yaw), 0);
	MoveServo(leg*3+2, -RadiansToDegrees(hipAngle), 0); // Positive angles for hip bend down?
	MoveServo(leg*3+3, -RadiansToDegrees(kneeAngle), 0); // Positive angles for knee bend up?

}

float ScaleJoystickValues(uint8_t axis)
{
	float scaled = ((float)(controller[axis])-128.0f)*0.78f; // converts 0-255 to -100 to 100
	if (scaled < 10.0f && scaled > -10.0f) scaled = 0; // 10% deadband
	return scaled;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //HAL_UART_RegisterCallback(&huart1, HAL_UART_RX_COMPLETE_CB_ID, HAL_UART_RxCpltCallback);
  HAL_UART_Receive_IT(&huart1, rxData, 1);


  for (int n=0; n<6; n++)
  {
	  SetServoLimits(1+3*n, -35, 35);
	  SetServoLimits(2+3*n, -35, 35);
	  SetServoLimits(3+3*n, -120, 20);
  }

  SetHipLocation(FRONT_LEFT, -60, 0, 58, -60);
  SetHipLocation(MID_LEFT, -75, 0, 0, -90);
  SetHipLocation(REAR_LEFT, -60, 0, -58, -120);
  SetHipLocation(REAR_RIGHT, 60, 0, -58, 120);
  SetHipLocation(MID_RIGHT, 75, 0, 0, 90);
  SetHipLocation(FRONT_RIGHT, 60, 0, 58, 60);

  int servoAngle[19] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  int servoID = 1;

  char txBuffer[30];
  char* failMsg = "FAILED!\r\n";


  uint32_t lastTick = HAL_GetTick();
  float strideTimer = 0;
  float strideLengthX = 0;
  float strideLengthZ = 0;
  float rotationRate = 0;
  float bodyPitch = 0;
  float rideHeight = 80;

  float targetStrideLengthX = 0;
  float targetStrideLengthZ = 0;
  float targetRotationRate = 0;
  float targetBodyPitch = 0;

  float wigglePitch = 0;
  float wiggleRoll = 0;
  float wiggleYaw = 0;
  float wiggleHeight = rideHeight;

  HAL_GPIO_WritePin(GPIOC, SERVO_EN_Pin, GPIO_PIN_SET); // Enable servos
  SetEyeColour(GREEN);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (charReceived)
	  {
		  if (charReceived == 'r') SetEyeColour(RED);
		  if (charReceived == 'g') SetEyeColour(GREEN);
		  if (charReceived == 'b') SetEyeColour(BLUE);

		  if (CALIBRATION_MODE)
		  {
			  if (charReceived == 'a' || charReceived == 's')
			  {
				  if (charReceived == 'a') servoAngle[servoID] -= 1;
				  if (charReceived == 's') servoAngle[servoID] += 1;

				  //MoveServo(servoID, servoAngle, 0);

				  sprintf(txBuffer, "ID %d, Angle = %d\r\n", servoID, servoAngle[servoID]);
				  HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);

				  if (!MoveServo(servoID, servoAngle[servoID], 0))
					  HAL_UART_Transmit(&huart1, (uint8_t*)failMsg, strlen(failMsg), HAL_MAX_DELAY);
			  }
			  else if (charReceived == '-' || charReceived == '=')
			  {
				  if (charReceived == '-' && servoID > 1) servoID--;
				  if (charReceived == '=' && servoID < 18) servoID++;
				  sprintf(txBuffer, "Selected servo %d\r\n", servoID);
				  HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
			  }
		  }

		  charReceived = 0;
	  }

	  if (controller[WALK_TYPE] & WIGGLE_BIT)
	  {
		  wigglePitch = ScaleJoystickValues(LEFT_Y)*0.2f; // ±20deg
		  wiggleRoll = ScaleJoystickValues(LEFT_X)*0.2f;
		  wiggleYaw = ScaleJoystickValues(RIGHT_X)*0.2f;
		  wiggleHeight = 100.0f+ScaleJoystickValues(RIGHT_Y)*0.5f;
	  }
	  else if (!CALIBRATION_MODE)
	  {
		  targetStrideLengthX = ScaleJoystickValues(LEFT_X);
		  targetStrideLengthZ = ScaleJoystickValues(LEFT_Y);
		  targetRotationRate = ScaleJoystickValues(RIGHT_X)*0.7f; // This one was a bit strong, needed to scale down somewhat
		  targetBodyPitch = ScaleJoystickValues(RIGHT_Y) *0.2f; // ±20 degrees
	  }

	  uint32_t now = HAL_GetTick();
	  int pollingRate = 20; // Milliseconds per update
	  if (CALIBRATION_MODE) pollingRate = 200; // 5Hz
	  if (now > lastTick+pollingRate)
	  {
		  lastTick += pollingRate;

		  //if (now > lastTick + pollingRate) // ERROR: Main loop not keeping up!
			//  SetEyeColour(RED);

		  if (CALIBRATION_MODE)
		  {
			  for (int id=1; id<=18; id++)
				  if (!MoveServo(id, servoAngle[id], 0))
					  HAL_UART_Transmit(&huart1, (uint8_t*)failMsg, strlen(failMsg), HAL_MAX_DELAY);
		  }
		  else if (controller[WALK_TYPE] & WIGGLE_BIT)
		  {
			  for (int leg = FRONT_LEFT; leg <= FRONT_RIGHT; leg++)
			  {
				  Point target;
				  float tempX = footNeutralPoint[leg].x;
				  float tempY = -wiggleHeight;
				  float tempZ = footNeutralPoint[leg].z;

				  float pitch = DegreesToRadians(wigglePitch);
				  float roll = DegreesToRadians(wiggleRoll);
				  float yaw = DegreesToRadians(wiggleYaw);

				  // Pitch, affects Y and Z
				  float tempY2 = tempY * cosf(pitch) - tempZ * sinf(pitch);
				  float tempZ2 = tempY * sinf(pitch) + tempZ * cosf(pitch);

				  // Roll, affects X and Y
				  float tempX2 = tempX * cosf(roll) - tempY2 * sinf(roll);
				  target.y = tempX * sinf(roll) + tempY2 * cosf(roll);

				  // Yaw, affects X and Z
				  target.x = tempX2 * cosf(yaw) - tempZ2 * sinf(yaw);
				  target.z = tempX2 * sinf(yaw) + tempZ2 * cosf(yaw);

				  MoveLeg(leg, target);
			  }
		  }
		  else
		  {
			  // Ramps for stride length
			  float rampRate = 2.0f; // Interplays with polling rate of course
			  if (targetStrideLengthZ > strideLengthZ) strideLengthZ += rampRate;
			  if (targetStrideLengthZ < strideLengthZ) strideLengthZ -= rampRate;
			  if (targetStrideLengthX > strideLengthX) strideLengthX += rampRate;
			  if (targetStrideLengthX < strideLengthX) strideLengthX -= rampRate;
			  if (targetRotationRate < rotationRate) rotationRate -= rampRate;
			  if (targetRotationRate > rotationRate) rotationRate += rampRate;
			  if (targetBodyPitch < bodyPitch) bodyPitch -= rampRate;
			  if (targetBodyPitch > bodyPitch) bodyPitch += rampRate;


			  float legLiftHeight = sqrtf(strideLengthX*strideLengthX+strideLengthZ*strideLengthZ+rotationRate*rotationRate);
			  if (legLiftHeight > 50) legLiftHeight = 50;

			  if (controller[WALK_TYPE] & HIGH_STEP_BIT) legLiftHeight *= 2.0f;

			  // Approximation of total stride length, which we'll use to limit angles
			  float totalStrideLength = sqrtf(strideLengthX*strideLengthX+strideLengthZ*strideLengthZ) + fabs(rotationRate); // Rotation needed a bit extra
			  float scalingFactor = 1;
			  if (totalStrideLength > 100.0f)
				  scalingFactor = 100.0f / totalStrideLength;

			  float targetHeight = 60; // Squat when not moving
			  if (controller[WALK_TYPE] & HIGH_BODY_BIT)
				  targetHeight = 150;
			  else if (legLiftHeight > 1) targetHeight = 100;
			  if (targetHeight > rideHeight) rideHeight += 2;
			  if (targetHeight < rideHeight) rideHeight -= 2;

			  float cadence = 2.0f - 0.01f*totalStrideLength*scalingFactor; // So, 2.0s for shortest steps, 1s for long steps

			  if (controller[WALK_TYPE] & QUICK_STEP_BIT)
			  {
				  scalingFactor *= 0.5f; // Shorter steps
				  cadence *= 0.5f; // Faster steps
			  }

			  strideTimer += (float)pollingRate/1000.0f/cadence;
			  if (strideTimer > 1) strideTimer -= 1;

			  for (int leg = FRONT_LEFT; leg <= FRONT_RIGHT; leg++)
			  {

				  Point target;
				  float rotation;
				  if (controller[WALK_TYPE] & RIPPLE_BIT)
				  {
					  // Ripple gait: Each leg 1/6th out of phase with others
					  // Sequence is rear left, front right, mid left, rear right, front left, mid right
					  // Each leg on ground 2/3rds of the time, quick semicircle return after
					  float t = strideTimer;
					  if (leg == REAR_LEFT) t += 0.833f; // Most advanced in gait sequence
					  if (leg == FRONT_RIGHT) t += 0.667;
					  if (leg == MID_LEFT) t += 0.5f;
					  if (leg == REAR_RIGHT) t += 0.333f;
					  if (leg == FRONT_LEFT) t += 0.167f;
					  // MID_RIGHT is last in sequence, no change

					  if (t > 1.0f) t -= 1.0f;

					  if (t < 0.6666f) // Horizontal drag part of gait
					  {
						  t *= 0.75f; // Convert to 0-0.5 range so it works with the normal gait algorithm

						  target.x = footNeutralPoint[leg].x + strideLengthX * scalingFactor * (0.5f - t*2.0f);
						  target.y = -rideHeight;
						  target.z = footNeutralPoint[leg].z + strideLengthZ * scalingFactor * (0.5f - t*2.0f);
						  rotation = DegreesToRadians((-0.5f+t*2.0f)*rotationRate)*0.25f;
						  // The 0.25 gets it into similar units as strides, e.g 100 rotation rate -> ~100mm steps
					  }
					  else // Return arc
					  {
						  // Convert t from 0.666-1.0 range into 0.5-1.0 so it works with normal gait algorithm
						  t = 0.5f + (t-0.6666f)*1.5f;

						  float angle = PI * (t-0.5f)*2.0f; // strideTimer of 0.5-1.0 converted to 0-PI radians
						  target.x = footNeutralPoint[leg].x - strideLengthX * scalingFactor * 0.5f * cosf(angle);
						  target.y = -rideHeight + legLiftHeight * sinf(angle);
						  target.z = footNeutralPoint[leg].z - strideLengthZ * scalingFactor * 0.5f * cosf(angle);
						  rotation = DegreesToRadians((0.5f - (t-0.5f)*2.0f)*rotationRate * scalingFactor)*0.25f;
					  }
				  }
				  else
				  {
					  // Tripod gait: Each foot down for 50% of the time, alternating flat drag then semicircle return
					  float t = strideTimer;
					  if (leg == MID_LEFT || leg == REAR_RIGHT || leg == FRONT_RIGHT)
						  t = strideTimer + 0.5f; // Half the legs are opposite phase
					  if (t > 1.0f) t -= 1.0f;

					  if (t < 0.5f) // Horizontal drag part of gait
					  {
						  target.x = footNeutralPoint[leg].x + strideLengthX * scalingFactor * (0.5f - t*2.0f);
						  target.y = -rideHeight;
						  target.z = footNeutralPoint[leg].z + strideLengthZ * scalingFactor * (0.5f - t*2.0f);
						  rotation = DegreesToRadians((-0.5f+t*2.0f)*rotationRate)*0.25f;
						  // The 0.25 gets it into similar units as strides, e.g 100 rotation rate -> ~100mm steps
					  }
					  else // Return arc
					  {
						  float angle = PI * (t-0.5f)*2.0f; // strideTimer of 0.5-1.0 converted to 0-PI radians
						  target.x = footNeutralPoint[leg].x - strideLengthX * scalingFactor * 0.5f * cosf(angle);
						  target.y = -rideHeight + legLiftHeight * sinf(angle);
						  target.z = footNeutralPoint[leg].z - strideLengthZ * scalingFactor * 0.5f * cosf(angle);
						  rotation = DegreesToRadians((0.5f - (t-0.5f)*2.0f)*rotationRate * scalingFactor)*0.25f;
					  }
				  }

				  // Add rotation offsets about body centre, in sync with steps, matrix is [ cos(θ) −sin(θ) | sin(θ) cos(θ) ]
				  // Want similar units - degrees per second is about right?
				  float tempX = target.x;
				  float tempZ = target.z;
				  target.x = tempX * cosf(rotation) - tempZ * sinf(rotation);
				  target.z = tempX * sinf(rotation) + tempZ * cosf(rotation);

				  // Add body pitching
				  float tempY = target.y;
				  tempZ = target.z;
				  float pitch = DegreesToRadians(bodyPitch);
				  target.y = tempY * cosf(pitch) - tempZ * sinf(pitch);
				  target.z = tempY * sinf(pitch) + tempZ * cosf(pitch);

				  MoveLeg(leg, target);
			  }

		  }

		  // Read pack voltage and transmit back to controller
		  if (!CALIBRATION_MODE)
		  {
			  HAL_ADC_Start(&hadc1);
			  if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
			  {
				  int adcValue = HAL_ADC_GetValue(&hadc1);

				  //if (adcValue > 1000 && adcValue < 1500) SetEyeColour(RED); // Testing!! Should pass if working

				  // Batt voltage measured via 10K:1K divider, i.e divide by 11
				  // 12 bit ADC = 0-4095 for 0-3.3V, 0.806 mV per ADC, 8.86 mV from battery per ADC
				  // 1422 ADC is 12.6V = 100%, 1016 ADC is 9V = 0%

				  uint8_t SoC = (adcValue-1016)/4;
				  if (SoC > 100) SoC = 100;
				  HAL_UART_Transmit(&huart1, &SoC, 1, HAL_MAX_DELAY);
			  }
		  }
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EYES_R_Pin|SERVO_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EYES_G_Pin|EYES_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EYES_R_Pin SERVO_EN_Pin */
  GPIO_InitStruct.Pin = EYES_R_Pin|SERVO_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EYES_G_Pin EYES_B_Pin */
  GPIO_InitStruct.Pin = EYES_G_Pin|EYES_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
