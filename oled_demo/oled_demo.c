#include <stdio.h>
#include <unistd.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_pwm.h"
#include "iot_i2c.h"
#include "iot_errno.h"

#include "ssd1306.h"


#include "hi_io.h"

#define OLED_I2C_BAUDRATE 400*1000

void TestDrawChinese1(void)
{
    const uint32_t W = 16, H = 16;
    uint8_t fonts[][32] = {
        {
            /*-- ID:0,字符:"丁",ASCII编码:B6A1,对应字:宽x高=16x16,画布:宽W=16 高H=16,共32字节*/
            0x00,0x04,0x3F,0xFE,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,
            0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x02,0xC0,0x01,0xC0,0x00,0x80,
        },{
           /*-- ID:1,字符:"焕",ASCII编码:BBC0,对应字:宽x高=16x16,画布:宽W=16 高H=16,共32字节*/
            0x10,0x40,0x18,0x64,0x18,0x7E,0x1A,0xCC,0x5F,0x8A,0x7D,0xFF,0x79,0xB6,0x59,0xB6,
            0x19,0xB6,0x19,0xB6,0x1F,0xFF,0x14,0x30,0x26,0x68,0x24,0xCC,0x41,0x87,0x02,0x02
        }
    };

    ssd1306_Fill(Black);
    for (size_t i = 0; i < sizeof(fonts)/sizeof(fonts[0]); i++) {
        ssd1306_DrawRegion(i * W, 0, W, H, fonts[i], sizeof(fonts[0]), W);
    }
    ssd1306_UpdateScreen();
    sleep(1);
}





void Ssd1306TestTask(void* arg)
{
    (void) arg;
    IoTGpioInit(HI_IO_NAME_GPIO_13);
    IoTGpioInit(HI_IO_NAME_GPIO_14);

    hi_io_set_func(HI_IO_NAME_GPIO_13, HI_IO_FUNC_GPIO_13_I2C0_SDA);
    hi_io_set_func(HI_IO_NAME_GPIO_14, HI_IO_FUNC_GPIO_14_I2C0_SCL);
    
    IoTI2cInit(0, OLED_I2C_BAUDRATE);

    //WatchDogDisable();

    usleep(20*1000);
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    //ssd1306_DrawString("Hello HarmonyOS!", Font_7x10, White);

    uint32_t start = HAL_GetTick();
    ssd1306_UpdateScreen();
    uint32_t end = HAL_GetTick();
    printf("ssd1306_UpdateScreen time cost: %d ms.\r\n", end - start);

    TestDrawChinese1();

    ssd1306_SetCursor(0, 18);
    ssd1306_DrawString("37120242204", Font_11x18, White);
    ssd1306_UpdateScreen();
    ssd1306_SetCursor(0, 36);
    ssd1306_DrawString("084", Font_11x18, White);
    ssd1306_UpdateScreen();
    //TestGetTick();
    while (1) {

    }
}


void Ssd1306TestDemo(void)
{
    osThreadAttr_t attr;

    attr.name = "Ssd1306Task";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = osPriorityNormal;

    if (osThreadNew(Ssd1306TestTask, NULL, &attr) == NULL) {
        printf("[Ssd1306TestDemo] Falied to create Ssd1306TestTask!\n");
    }
}
APP_FEATURE_INIT(Ssd1306TestDemo);
