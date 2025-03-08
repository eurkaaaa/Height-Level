#include "tof_init.h"
#include "deck.h"
#include "debug.h"
#include "param.h"
#include "FreeRTOS.h"
#include "task.h"
#include "zranger2.h"
#include "range.h"
#include "app.h"

typedef struct tof_packet {
    uint8_t id;
    int16_t distance[64];
} tof_packet;

typedef void (*ToFdataCallback)(const tof_packet* pk);

static const float expPointA = 2.5f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 4.0f;
static const float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;
static VL53L5CX_Configuration vl53l5dev_f;
static VL53L5CX_ResultsData vl53l5_res_f;
static tof_packet pk;
static ToFdataCallback tofHandlerCallback = NULL;
void tof_get_data(void *argument);
void ToFRegisterMessageHandler(ToFdataCallback callback);
static void tofCallback();

void tof_get_data(void* argument){
	pk.id = 0;
    pinMode(DECK_GPIO_IO1, OUTPUT);
    digitalWrite(DECK_GPIO_IO1, HIGH);
	vTaskDelay(100);
	initialize_sensors_I2C(&vl53l5dev_f,1);//初始化VL53L5CX传感器
	vl53l5cx_start_ranging(&vl53l5dev_f);//配置开始测距
	while(1){
	  get_sensor_data(&vl53l5dev_f, &vl53l5_res_f);//获取传感器数据
	  memcpy(pk.distance, vl53l5_res_f.distance_mm, sizeof(pk.distance));
	  tofHandlerCallback(&pk);
	  vTaskDelay(500);
	}
}

void ToFRegisterMessageHandler(ToFdataCallback callback) {
    tofHandlerCallback = callback;
}

// static void tofCallback()
// {
//     DEBUG_PRINT("print tof data \n");
//     DEBUG_PRINT("%d \n", pk.distance[16]);
// 	// for(int i=0;i<64;i++)
//     // {
//     //     DEBUG_PRINT("%d \t", pk.distance[i]);
//     //     if(i % 8 == 0)
//     //     {
//     //         DEBUG_PRINT("\n");
//     //     }
//     // }
// 	vTaskDelay(5);
// }

int computeZ(VL53L5CX_ResultsData vl53l5_res_f)
{
    int count = 0;
    double sum = 0.0;
    for(int i=0;i<16;i++)
    {
        if(vl53l5_res_f.target_status[i] == 5 || vl53l5_res_f.target_status[i] == 9)
        {
            count++;
            sum += vl53l5_res_f.distance_mm[i];
        }
    }
    return sum/count;
}

void appMain()
{
    DEBUG_PRINT("start \n");
    vTaskDelay(3000);
	pk.id = 0;
    pinMode(DECK_GPIO_IO1, OUTPUT);
    digitalWrite(DECK_GPIO_IO1, HIGH);
	vTaskDelay(100);
	// initialize_sensors_I2C(&vl53l5dev_f,1);//初始化VL53L5CX传感器
    if(initialize_sensors_I2C(&vl53l5dev_f,1) == true)
    {
        DEBUG_PRINT("init sucess! \n");
    }
    else
        DEBUG_PRINT("init failed \n");
	vl53l5cx_start_ranging(&vl53l5dev_f);//配置开始测距
    while(1){
        get_sensor_data(&vl53l5dev_f, &vl53l5_res_f);//获取传感器数据
        //memcpy(pk.distance, vl53l5_res_f.distance_mm, sizeof(pk.distance));
        // DEBUG_PRINT("print tof data \n");
        // for(int i=0;i<16;i++)
        // {
        //     DEBUG_PRINT("%d \t", vl53l5_res_f.distance_mm[i]);
        //     if((i+1)  % 4 == 0)
        //     {
        //         DEBUG_PRINT("\n");
        //     }
        // }
        // vTaskDelay(1000);
        // DEBUG_PRINT("print tof statu \n");
        // for(int i=0;i<16;i++)
        // {
        //     DEBUG_PRINT("%d \t", vl53l5_res_f.target_status[i]);
        //     if((i+1) % 4 == 0)
        //     {
        //         DEBUG_PRINT("\n");
        //     }
        // }
        int my_z = computeZ(vl53l5_res_f);
        // DEBUG_PRINT("compute %d \n", my_z);
        float distance = (float)my_z * 0.001f; // Scale from [mm] to [m]
        float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
        rangeEnqueueDownRangeInEstimator(distance,stdDev,xTaskGetTickCount());
        vTaskDelay(67);
      }
}