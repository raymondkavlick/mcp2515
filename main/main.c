//Curtis Instruments @ 2018

#include "main.h"
#include "mcp2515.h"

void task_1ms(void* arg)
{
    uint32_t io_num;
    for(;;)
    {
//        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
//        {
//            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
//        }
    	//printf("\nin1ms\n");
    	//xTaskSuspend();
    }
}

void can_rx(void* arg)
{
    uint32_t io_num;
    for(;;)
    {
    	extern bool rx_has_data();
    	rx_has_data();

//        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
//        {
//            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
//        }
    	//printf("\nin1ms\n");
    	//xTaskSuspend();
    }
}

void app_main(void)
{
	printf("\r\n*******************************************\r\n");
	printf("\r\nTemplate for Curtis Instruments 1305 Canary\r\n");
	printf("\r\n*******************************************\r\n");

	mcp_init(BAUD_125);
	uint8_t test_data[8] = { 0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77};
	mcp_send_can(0x7FF,test_data,8);

    //xTaskCreate(task_1ms, "1ms tic", 2048, NULL, 10, NULL);
    xTaskCreate(can_rx, "canrx handler", 2048, NULL, 10, NULL);
    while(1)
    {
    	mcp_send_can(2,test_data,8);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
		printf("tic.\r\n");
		//vTaskSuspend(xHandle);
    }

}
