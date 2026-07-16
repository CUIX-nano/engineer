#include "cmsis_os.h"

extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void interboard_communication_mission(void  *argument);
    extern void engineer_chassis_mission(void *argument);
    extern void VOFA_Thread(void *argument);
    extern void magazine_mission(void *argument);
    extern void referee_system_mission(void *argument);
    extern void referee_task(void *arg);


    void start_mission_planer_task(void const *argument)
    {
        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(interboard_communication_mission,"interboard_communication_mission", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(engineer_chassis_mission,"engineer_chassis_mission", 512, nullptr,configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(VOFA_Thread,"VOFA_Thread", 512, nullptr,configMAX_PRIORITIES - 2, nullptr);
        xTaskCreate(magazine_mission,"magazine_mission", 512, nullptr,configMAX_PRIORITIES - 2, nullptr);
        xTaskCreate(referee_system_mission,"referee_system_mission", 512, nullptr,configMAX_PRIORITIES - 2, nullptr);
        xTaskCreate(referee_task,"referee_task", 512, nullptr,configMAX_PRIORITIES - 2, nullptr);
        vTaskDelete(nullptr);
    }
}