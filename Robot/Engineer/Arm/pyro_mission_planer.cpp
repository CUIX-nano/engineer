#include "cmsis_os.h"


extern "C" {
    extern void pyro_init_thread(void *argument);
    extern void engineer_arm_mission(void* args);
    extern void interboard_communication_mission(void* args);
    extern  void self_control_mission(void* args);
    extern void engineer_arm_planning_mission(void* args);
    extern void VOFA_Thread(void *argument);

    void start_mission_planer_task(void const *argument)
    {
        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelay(10);
        xTaskCreate(engineer_arm_mission, "engineer_arm_mission", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(interboard_communication_mission, "interboard_communication_mission", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(self_control_mission, "self_control_mission", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(engineer_arm_planning_mission, "engineer_arm_planning_mission", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(VOFA_Thread, "VOFA_Thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}