/******************************************************************************
* File Name:   publisher_task.c
*
* Description: This file contains the task that sets up the user button GPIO 
*              for the publisher and publishes MQTT messages on the topic
*              'MQTT_PUB_TOPIC' to control a device that is actuated by the
*              subscriber task. The file also contains the ISR that notifies
*              the publisher task about the new device state to be published.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "timers.h"

#include <inttypes.h>
#include <math.h>

/* Task header files */
#include "publisher_task.h"
#include "mqtt_task.h"
#include "subscriber_task.h"

/* Configuration file for MQTT client */
#include "mqtt_client_config.h"

/* Middleware libraries */
#include "cy_mqtt_api.h"
#include "cy_retarget_io.h"

/******************************************************************************
* Macros
******************************************************************************/
/* Interrupt priority for User Button Input. */
#define USER_BTN_INTR_PRIORITY          (5)

/* The maximum number of times each PUBLISH in this example will be retried. */
#define PUBLISH_RETRY_LIMIT             (10)

/* A PUBLISH message is retried if no response is received within this 
 * time (in milliseconds).
 */
#define PUBLISH_RETRY_MS                (1000)

/* Queue length of a message queue that is used to communicate with the 
 * publisher task.
 */
#define PUBLISHER_TASK_QUEUE_LENGTH     (3u)

#define TIMER_ID	1

/******************************************************************************
* Global Variables
*******************************************************************************/
/* FreeRTOS task handle for this task. */
TaskHandle_t publisher_task_handle;

/* Handle of the queue holding the commands for the publisher task */
QueueHandle_t publisher_task_q;

/* Structure to store publish message information. */
cy_mqtt_publish_info_t publish_info =
{
    .qos = (cy_mqtt_qos_t) MQTT_MESSAGES_QOS,
    .topic = MQTT_PUB_TOPIC,
    .topic_len = (sizeof(MQTT_PUB_TOPIC) - 1),
    .retain = false,
    .dup = false
};

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void publisher_init(void);
static void publisher_deinit(void);
static void isr_button_press(void *callback_arg, cyhal_gpio_event_t event);
void print_heap_usage(char *msg);

static void vTimerCallback( TimerHandle_t pxTimer );
static TimerHandle_t xTimer = NULL;

/******************************************************************************
 * Function Name: publisher_task
 ******************************************************************************
 * Summary:
 *  Task that sets up the user button GPIO for the publisher and publishes 
 *  MQTT messages to the broker. The user button init and deinit operations,
 *  and the MQTT publish operation is performed based on commands sent by other
 *  tasks and callbacks over a message queue.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void publisher_task(void *pvParameters)
{
    /* Status variable */
    cy_rslt_t result;

    publisher_data_t publisher_q_data;

    /* Command to the MQTT client task */
    mqtt_task_cmd_t mqtt_task_cmd;

    /* To avoid compiler warnings */
    (void) pvParameters;

    /* Initialize and set-up the user button GPIO. */
    publisher_init();

    struct readings DHT_reading = {0, 0};

    /* Create a message queue to communicate with other tasks and callbacks. */
    publisher_task_q = xQueueCreate(PUBLISHER_TASK_QUEUE_LENGTH, sizeof(publisher_data_t));

    while (true)
    {
        /* Wait for commands from other tasks and callbacks. */
        if (pdTRUE == xQueueReceive(publisher_task_q, &publisher_q_data, portMAX_DELAY))
        {
            switch(publisher_q_data.cmd)
            {
                case PUBLISHER_INIT:
                {
                    /* Initialize and set-up the user button GPIO. */
                    publisher_init();
                    break;
                }

                case PUBLISHER_DEINIT:
                {
                    /* Deinit the user button GPIO and corresponding interrupt. */
                    publisher_deinit();
                    break;
                }

                case PUBLISH_MQTT_MSG:
                {
                	publish_info.topic = MQTT_PUB_TOPIC;
					publish_info.topic_len = (sizeof(MQTT_PUB_TOPIC) - 1);
                    /* Publish the data received over the message queue. */
                    publish_info.payload = publisher_q_data.data;
                    publish_info.payload_len = strlen(publish_info.payload);

                    printf("  Publisher: Publishing '%s' on the topic '%s'\n\n",
                           (char *) publish_info.payload, publish_info.topic);

                    result = cy_mqtt_publish(mqtt_connection, &publish_info);

                    if (result != CY_RSLT_SUCCESS)
                    {
                        printf("  Publisher: MQTT Publish failed with error 0x%0X.\n\n", (int)result);

                        /* Communicate the publish failure with the the MQTT 
                         * client task.
                         */
                        mqtt_task_cmd = HANDLE_MQTT_PUBLISH_FAILURE;
                        xQueueSend(mqtt_task_q, &mqtt_task_cmd, portMAX_DELAY);
                    }

                    print_heap_usage("publisher_task: After publishing an MQTT message");
                    break;
                }

                case PUBLISH_MQTT_SENSOR:
				{
					DHT_reading.result_code = DHT_Read(&DHT_reading.humidity, &DHT_reading.temperature);

					if(DHT_reading.result_code == SUCCESS) {
						printf("DHT succes \n\r");

						int humidity = DHT_reading.humidity;
						char buffer[3];

						itoa(humidity, buffer, 10);

						publish_info.topic = MQTT_PUB_HUMID;
						publish_info.topic_len = (sizeof(MQTT_PUB_HUMID) - 1);

						publish_info.payload = buffer;
						publish_info.payload_len = strlen(buffer);

						printf("  Publisher: Publishing '%s' on the topic '%s'\n\n",
							   (char *) publish_info.payload, publish_info.topic);

						result = cy_mqtt_publish(mqtt_connection, &publish_info);

						if (result != CY_RSLT_SUCCESS)
						{
							printf("  Publisher: MQTT Publish failed with error 0x%0X.\n\n", (int)result);

							/* Communicate the publish failure with the the MQTT
							 * client task.
							 */
							mqtt_task_cmd = HANDLE_MQTT_PUBLISH_FAILURE;
							xQueueSend(mqtt_task_q, &mqtt_task_cmd, portMAX_DELAY);
						}

						int temperature = DHT_reading.temperature;
						char tempBuffer[3];

						itoa(temperature, tempBuffer, 10);

						publish_info.topic = MQTT_PUB_TEMP;
						publish_info.topic_len = (sizeof(MQTT_PUB_TEMP) - 1);

						publish_info.payload = tempBuffer;
						publish_info.payload_len = strlen(tempBuffer);

						printf("  Publisher: Publishing '%s' on the topic '%s'\n\n",
							   (char *) publish_info.payload, publish_info.topic);

						result = cy_mqtt_publish(mqtt_connection, &publish_info);

						if (result != CY_RSLT_SUCCESS)
						{
							printf("  Publisher: MQTT Publish failed with error 0x%0X.\n\n", (int)result);

							/* Communicate the publish failure with the the MQTT
							 * client task.
							 */
							mqtt_task_cmd = HANDLE_MQTT_PUBLISH_FAILURE;
							xQueueSend(mqtt_task_q, &mqtt_task_cmd, portMAX_DELAY);
						}

					}
					else {
						/* Exit critical section */
						taskEXIT_CRITICAL();

						printf("DHT failed \n\r");
					}
					print_heap_usage("publisher_task: After publishing an MQTT message");
					break;

				}
            }
            printf("entering sleep mode\n");
            cyhal_syspm_sleep();
        }
    }
}

/******************************************************************************
 * Function Name: publisher_init
 ******************************************************************************
 * Summary:
 *  Function that initializes and sets-up the user button GPIO pin along with  
 *  its interrupt.
 * 
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void publisher_init(void)
{
    /* Initialize the user button GPIO and register interrupt on falling edge. */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_register_callback(CYBSP_USER_BTN, isr_button_press, NULL);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                            USER_BTN_INTR_PRIORITY, true);
    
    cyhal_gpio_init(DATA_PIN, CYHAL_GPIO_DIR_BIDIRECTIONAL, CYHAL_GPIO_DRIVE_PULLUP, 1);

    printf("Press the user button (SW2) to publish \"%s\"/\"%s\" on the topic '%s'...\n\n", 
           MQTT_DEVICE_ON_MESSAGE, MQTT_DEVICE_OFF_MESSAGE, publish_info.topic);

    // timer voor meting en uit slaap halen, elke 3 minuten

    xTimer = xTimerCreate( (const char *) "Timer",
        							pdMS_TO_TICKS(30000UL),
        							pdTRUE,
        							(void *) TIMER_ID,
        							vTimerCallback);

    xTimerStart( xTimer, 0 );
}

/******************************************************************************
 * Function Name: publisher_deinit
 ******************************************************************************
 * Summary:
 *  Cleanup function for the publisher task that disables the user button  
 *  interrupt and deinits the user button GPIO pin.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void publisher_deinit(void)
{
    /* Deregister the ISR and disable the interrupt on the user button. */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, NULL, NULL);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                            USER_BTN_INTR_PRIORITY, false);
    cyhal_gpio_free(CYBSP_USER_BTN);

}

static void vTimerCallback( TimerHandle_t pxTimer )
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	publisher_data_t publisher_q_data;

	/* Assign the publish command to be sent to the publisher task. */
	//publisher_q_data.cmd = PUBLISH_MQTT_MSG;
	publisher_q_data.cmd = PUBLISH_MQTT_SENSOR;

	/* Assign the publish message payload so that the device state toggles. */
	publisher_q_data.data = "xxx";

	/* Send the command and data to publisher task over the queue */
	xQueueSendFromISR(publisher_task_q, &publisher_q_data, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/******************************************************************************
 * Function Name: isr_button_press
 ******************************************************************************
 * Summary:
 *  GPIO interrupt service routine. This function detects button
 *  presses and sends the publish command along with the data to be published 
 *  to the publisher task over a message queue. Based on the current device 
 *  state, the publish data is set so that the device state gets toggled.
 *
 * Parameters:
 *  void *callback_arg : pointer to variable passed to the ISR (unused)
 *  cyhal_gpio_event_t event : GPIO event type (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void isr_button_press(void *callback_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    publisher_data_t publisher_q_data;

    /* To avoid compiler warnings */
    (void) callback_arg;
    (void) event;

    /* Assign the publish command to be sent to the publisher task. */
    //publisher_q_data.cmd = PUBLISH_MQTT_MSG;
    publisher_q_data.cmd = PUBLISH_MQTT_MSG;

    /* Assign the publish message payload so that the device state toggles. */
    publisher_q_data.data = "sensor test";

    /* Send the command and data to publisher task over the queue */
    xQueueSendFromISR(publisher_task_q, &publisher_q_data, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
* Function Name: Fraction_Convert
****************************************************************************//**
*
* Converts a 8 bit binary number into a fractional decimal value
*
* \param num
* 8 bit binary value
*
* \return
* Fractional decimal value of the binary number
*
*******************************************************************************/
float Fraction_Convert(uint8_t num)
{
    float fraction = 0;
    int unit = 0;
    for( int i = 0; i<8; i++)
    {
        unit = num & 1;
        num = num>>1;
        fraction = fraction + unit * pow(2, -(1+i));
    }
    return fraction;
}

/*******************************************************************************
* Function Name: DHT_Start
****************************************************************************//**
*
* Initiates the communication with the sensor.
* Function sends 18ms low signal after 1s delay.
*
*******************************************************************************/

void DHT_Start(void)
{
	//cyhal_gpio_configure(DATA_PIN, CYHAL_GPIO_DIR_BIDIRECTIONAL, CYHAL_GPIO_DRIVE_STRONG);
    cyhal_gpio_write(DATA_PIN, 1);
    cyhal_system_delay_ms(1000);
    cyhal_gpio_write(DATA_PIN, 0);
    cyhal_system_delay_ms(18);
    cyhal_gpio_write(DATA_PIN, 1);
    //cyhal_gpio_configure(DATA_PIN, CYHAL_GPIO_DIR_BIDIRECTIONAL, CYHAL_GPIO_DRIVE_PULLUP);

//    if (result != CY_RSLT_SUCCESS)
//        {
//            printf("result error \n\r");
//        }
}

/*******************************************************************************
* Function Name: DHT_Read
****************************************************************************//**
*
* Reads the temperature and humidity values if the sensor read is successful
*
* \param humidity
* Pointer to the float variable which stores humidity value
*
* \param temperature
* Pointer to the float variable which stores temperature value
*
* \return
* Status of sensor read
*
*******************************************************************************/

uint8 DHT_Read(float* humidity, float* temperature)
{
    uint8_t delay_time = 0, ack_time = 0;
    uint8_t temp = 0, index = 0, bit_count = 7;
    uint8_t byteval[5] = {0,0,0,0,0};   /* Array to store the 5 byte values received from the sensor */


    DHT_Start();
    /****************************************************************************
    *   Response to the start condition is a 54us low signal and 80us high signal.
    ****************************************************************************/

    taskENTER_CRITICAL();
    while( cyhal_gpio_read(DATA_PIN) == 1)
    {
        cyhal_system_delay_us(1);
        ack_time++;
        if(ack_time > timeout_duration)
        {
        	printf("timeout 1 \n\r");
            /* Connection timed out */
            return DHT_CONNECTION_ERROR;
        }
    }

    while( cyhal_gpio_read(DATA_PIN) == 0)
    {
    	cyhal_system_delay_us(1);
        ack_time++;
        if(ack_time > timeout_duration)
        {
        	printf("timeout 2 \n\r");
            /* Connection timed out */
            return DHT_CONNECTION_ERROR;
        }
    }

    /* Value of ack_time stored so that it can be used to calculate bit value */
    /* Value of delay_time corresponds to a little more than 54 us */
    delay_time = ack_time;
    ack_time = 0;
    while( cyhal_gpio_read(DATA_PIN) == 1)
    {
        /* Spin until sensor pulls the data line high */
    	cyhal_system_delay_us(1);
        ack_time++;
        if(ack_time > timeout_duration)
        {
            /* Connection timed out */
        	printf("timeout 3 \n\r");
            return DHT_CONNECTION_ERROR;
        }
    }

    for (int i = 0; i < 40; i++)
	{
        ack_time = 0;
        while( cyhal_gpio_read(DATA_PIN) == 0)
        {
            /* Spin until sensor pulls the data line high */
        	cyhal_system_delay_us(1);
            ack_time++;
            if(ack_time > timeout_duration)
            {
                /* Connection timed out */
            	printf("timeout 4 \n\r");
                return DHT_CONNECTION_ERROR;
            }
        }

        ack_time = 0;

        while( cyhal_gpio_read(DATA_PIN) == 1)
        {
        	cyhal_system_delay_us(1);
            ack_time++;
            if(ack_time > timeout_duration)
            {
                /* Connection timed out */
            	printf("timeout 5 \n\r");
                return DHT_CONNECTION_ERROR;
            }
        }

        /****************************************************************************
        *  The sensor MCU outputs a low signal for 54us and high signal for 24us if
        *  bit value is 0 and a low signal for 54us and high signal for 70us if bit
        *  value is 1.
        ****************************************************************************/

        /* If ack_time value is more than delay_time/2 i.e., ack_time > (approx) 27 us then bit value is 1 */
        if ((ack_time) > (delay_time/2))
            byteval[index] |= (1 << bit_count);
        if (bit_count == 0)   /* Next Byte */
   	    {
   		    bit_count = 7;    /* Reset bit_count to 7 */
   		    index++;          /* Increment index so that next byte is chosen */
   	    }
   	    else bit_count--;
    }
    taskEXIT_CRITICAL();

    /* Checksum is calculated by adding all 4 bytes */
    temp = (byteval[0] + byteval[1] + byteval[2] + byteval[3]);
    if((temp == byteval[4]) && (byteval[4] != 0))
    {
        /* Pass the temperature and humidity data only when checksum is matched and it is not equal to 0 */
        *humidity = (int)byteval[0] + Fraction_Convert(byteval[1]);
        *temperature = (int)byteval[2] + Fraction_Convert(byteval[3]);

        return SUCCESS;
    }
    else
    {
    	/* ********************************************************************
    	 * Enter critical section which is exited in the task function.
    	 * If the application is not inside critical section and if
    	 * taskEXIT_CRITICAL() is called the system goes into hard fault.
    	 * To ensure that this does not happen the function is called.
    	 * ********************************************************************/
    	taskENTER_CRITICAL();

    	/* Incorrect value */
        return DHT_INCORRECT_VALUE;
    }
}

/* [] END OF FILE */
