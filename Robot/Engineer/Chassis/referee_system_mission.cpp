#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_databoard.h"
#include "pyro_uart_drv.h"
#include "pyro_bsp_uart.h"          // 新增
#include <string.h>
#include "queue.h"
#include "CRC8_CRC16.h"
#include <string>
#include <sstream>
#include <memory>
#include "UI_group.h"
#include "referee.h"


pyro::uart_drv_t* referee_system_uart;

uint8_t referee_system_tx_buf[256];

class hexagon_UI:public UI_group
{
    public:
        explicit hexagon_UI(uint16_t center_x, uint16_t center_y, uint16_t l,UI_obj::color_e color,pyro::uart_drv_t* uart):UI_group(1,uart)
        {
            this->center_x = center_x;
            this->center_y = center_y;
            this->l = l;
            this->H = l*sqrt(3)/2;
            this->_color = color;
            points[0] = std::make_pair(center_x-l/2,center_y-H);
            points[1] = std::make_pair(center_x+l/2,center_y-H);
            points[2] = std::make_pair(center_x+l,center_y);
            points[3] = std::make_pair(center_x+l/2,center_y+H);
            points[4] = std::make_pair(center_x-l/2,center_y+H);
            points[5] = std::make_pair(center_x-l,center_y);

            // for(int i = 0;i<6 ;i++)
            // {
            //     std::string tmp_str = "L11";
            //     tmp_str[1]+=i;
            //     UI_obj* tmp = new UI_Line(tmp_str,points[i].first,points[i].second,points[(i+1)%6].first,points[(i+1)%6].second,5,0,color);
            //     // tmp->start();
            //     this->add_UI_obj(tmp);
            // }
            // UI_obj* rect = new UI_Rectangle("Aim",940,520,980,560,2,0,color);
            // this->add_UI_obj(rect);
            UI_obj* LineH = new UI_Line("LLH",center_x-l,center_y,center_x+l,center_y,5,0,color);
            this->add_UI_obj(LineH);
            UI_obj* LineV = new UI_Line("LLV",center_x,center_y-H,center_x,center_y+H,5,0,color);
            this->add_UI_obj(LineV);

        }
        void start()
        {
            for(auto& ui : this->_UI_objs)
                ui->start();
        }
        void update()
        {
            for(auto& ui : this->_UI_objs)
                ui->update();
        }
    private:
        uint16_t center_x,center_y;
        uint16_t l,H;
        UI_obj::color_e _color;
        std::pair<float,float> points[6];
};

class MotionSelectCombo:public UI_group
{
    public:
        explicit MotionSelectCombo(uint16_t center_x, uint16_t center_y, uint16_t width,uint16_t height,UI_obj::color_e color,pyro::uart_drv_t* uart):UI_group(2,uart)
        {
            this->center_x = center_x;
            this->center_y = center_y;
            this->width = width;
            this->height = height;
            this->_color = color;

            UI_obj* rect = new UI_Rectangle("Mot",center_x-width/2,center_y-height/2,center_x+width/2,center_y+height/2,4,1,color);
            this->add_UI_obj(rect);

            num = new UI_Integer("MoN",center_x-12,center_y+25,4,1,50,1,UI_obj::white);
            this->add_UI_obj(num);
        }
        void start()
        {
            for(auto& ui : this->_UI_objs)
                ui->start();
        }
        void update()
        {
            for(auto& ui : this->_UI_objs)
                ui->update();
        }

        void update(int which)
        {
            which%=6;
            which +=which == 0?6:0;
            num->update(center_x-12,center_y+25,4,which,50,1,UI_obj::white);
            update();
        }
    private:
        uint16_t center_x,center_y;
        uint16_t width,height;
        UI_obj::color_e _color;
        UI_Integer* num;
};

class EnergyUnitStorageP1:public UI_group
{
        public:
        explicit EnergyUnitStorageP1(uint16_t center_x, uint16_t center_y, uint16_t big_radius,uint16_t small_radius,UI_obj::color_e color,pyro::uart_drv_t* uart):UI_group(3,uart)
        {
            this->center_x = center_x;
            this->center_y = center_y;
            this->big_radius = big_radius;
            this->small_radius = small_radius;
            this->_color = color;
            
            cir1 = new UI_Circle("EB1",center_x+big_radius,center_y,20,small_radius,2,color);
            this->add_UI_obj(cir1);

            cir2 = new UI_Circle("EB2",center_x,center_y+big_radius,20,small_radius,2,color);
            this->add_UI_obj(cir2);

            cir3 = new UI_Circle("EB3",center_x-big_radius,center_y,20,small_radius,2,color);
            this->add_UI_obj(cir3);

            

            cir4 = new UI_Circle("EB4",center_x,center_y-big_radius,20,small_radius,2,color);
            this->add_UI_obj(cir4);

            select = new UI_Line("BBC",center_x,center_y,center_x+big_radius/3,center_y,10,2,color);
            this->add_UI_obj(select);
        }
        void start()
        {
            for(auto& ui : this->_UI_objs)
                ui->start();
        }
        void update()
        {
            for(auto& ui : this->_UI_objs)
                ui->update();
        }
        void update(uint8_t have,uint8_t which)
        {
            if(have&1)
            {
                cir1->update(center_x+big_radius,center_y,20,small_radius,2,_color);
            }
            else
            {
                cir1->update(center_x+big_radius,center_y,3,2,2,_color);
            }

            if(have&2)
            {
                cir2->update(center_x,center_y+big_radius,20,small_radius,2,_color);
            }
            else
            {
                cir2->update(center_x,center_y+big_radius,3,2,2,_color);
            }

            if(have&4)
            {
                cir3->update(center_x-big_radius,center_y,20,small_radius,2,_color);
            }
            else
            {
                cir3->update(center_x-big_radius,center_y,3,2,2,_color);
            }

            if(have&8)
            {
                cir4->update(center_x,center_y-big_radius,20 ,small_radius,2,_color);
            }
            else
            {
                cir4->update(center_x,center_y-big_radius,3,2,2,_color);
            }
            which%=4;
            which+=which==0?4:0;

            switch(which)
            {
                case 1:
                select->update(center_x,center_y,center_x+big_radius/2,center_y,20,2,_color);
                break;
                case 2:
                select->update(center_x,center_y,center_x,center_y+big_radius/2,20,2,_color);
                break;
                case 3:
                select->update(center_x,center_y,center_x-big_radius/2,center_y,20,2,_color);
                break;
                case 4:
                select->update(center_x,center_y,center_x,center_y-big_radius/2,20,2,_color);
                break;
            }
            update();
        }
    private:
        uint16_t center_x,center_y;
        uint16_t big_radius,small_radius;
        UI_obj::color_e _color;
        std::pair<uint16_t,uint16_t> centers[4];
        UI_Circle* cir1,*cir2,*cir3,*cir4;
        UI_Line* select;
};

class EnergyUnitStorageP2:public UI_group
{
        public:
        explicit EnergyUnitStorageP2(uint16_t center_x, uint16_t center_y, uint16_t big_radius,uint16_t small_radius,UI_obj::color_e color,pyro::uart_drv_t* uart):UI_group(4,uart)
        {
            this->center_x = center_x;
            this->center_y = center_y;
            this->big_radius = big_radius;
            this->small_radius = small_radius;
            this->_color = color;
            
            UI_obj* cir1 = new UI_Circle("EC1",center_x+big_radius,center_y,3,small_radius,2,color);
            this->add_UI_obj(cir1);

            UI_obj* cir2 = new UI_Circle("EC2",center_x-big_radius,center_y,3,small_radius,2,color);
            this->add_UI_obj(cir2);

            UI_obj* cir3 = new UI_Circle("EC3",center_x,center_y+big_radius,3,small_radius,2,color);
            this->add_UI_obj(cir3);

            UI_obj* cir4 = new UI_Circle("EC4",center_x,center_y-big_radius,3,small_radius,2,color);
            this->add_UI_obj(cir4);

            UI_obj* big_cir = new UI_Circle("BCC",center_x,center_y,3,big_radius,2,color);
            this->add_UI_obj(big_cir);
        }
        void start()
        {
            for(auto& ui : this->_UI_objs)
                ui->start();
        }
        void update()
        {
            for(auto& ui : this->_UI_objs)
                ui->update();
        }
    private:
        uint16_t center_x,center_y;
        uint16_t big_radius,small_radius;
        UI_obj::color_e _color;
        std::pair<uint16_t,uint16_t> centers[4];
};

bool referee_system_callback(uint8_t *buf, uint16_t len,BaseType_t xHigherPriorityTaskWoken)
{
    if( len != 0 )
    {
        memcpy(referee_system_tx_buf,buf,len);
    }
    return false;
}

hexagon_UI *hexagon;
MotionSelectCombo* motion_select;
EnergyUnitStorageP1 * p1;
EnergyUnitStorageP2 * p2;
uint16_t fresh_flag = 0;
uint8_t seq = 0;
uint8_t switch_flag = 0;

extern uint8_t which_mine;
extern uint8_t which_motion;
extern "C" void referee_system_mission(void const *argument)
{
    // 修改 UART 获取方式
    referee_system_uart = &pyro::bsp_uart::get_uart1();
    referee_system_uart->add_rx_event_callback(referee_system_callback, 1);
    hexagon = new hexagon_UI(960,630,140,UI_obj::color_e::blue_or_red,referee_system_uart);
    hexagon->set_task_cycle(10);
    hexagon->set_update_cycle(40);
    motion_select = new MotionSelectCombo(300,600,80,80,UI_obj::blue_or_red,referee_system_uart);
    motion_select->set_task_cycle(10);
    motion_select->set_update_cycle(40);
    p1 = new EnergyUnitStorageP1(1600,600,100,20,UI_obj::white,referee_system_uart);
    p1->set_task_cycle(10);
    p1->set_update_cycle(40);
    p2 =new EnergyUnitStorageP2(1600,600,100,50,UI_obj::black,referee_system_uart);
    p2->set_task_cycle(10);
    p2->set_update_cycle(40);
    hexagon->init();
    motion_select->init();
    p1->init();
    p2->init();

    hexagon->set_sender_id(robot_id_t::RED_ENGINEER);
    motion_select->set_sender_id(robot_id_t::RED_ENGINEER);
    p1->set_sender_id(robot_id_t::RED_ENGINEER);
    p2->set_sender_id(robot_id_t::RED_ENGINEER);
    // hexagon->start();
    for(;;)
    {

        if(referee_data.robot_status.robot_id == robot_id_t::RED_ENGINEER || referee_data.robot_status.robot_id == robot_id_t::BLUE_ENGINEER)
        {
            hexagon->set_sender_id(referee_data.robot_status.robot_id);
            motion_select->set_sender_id(referee_data.robot_status.robot_id);
            p1->set_sender_id(referee_data.robot_status.robot_id);
            p2->set_sender_id(referee_data.robot_status.robot_id);
        }

        {
            if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET)
            {
                switch_flag |= 1;
            }
            else
            {
                switch_flag &= 0xfe;
            }

            if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == GPIO_PIN_RESET)
            {
                switch_flag |= 8;
            }
            else
            {
                switch_flag &= 0xf7;
            }

            if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9) == GPIO_PIN_RESET)
            {
                switch_flag |= 4;
            }
            else
            {
                switch_flag &= 0xfb;
            }

            if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13) == GPIO_PIN_RESET)
            {
                switch_flag |= 2;
            }
            else
            {
                switch_flag &= 0xfd;
            }
        }

        if(fresh_flag>50)
        {
            motion_select->start();
            hexagon->start();
            p1->start();
            p2->start();
            fresh_flag=0;
        }
        else
        {
            motion_select->update(which_motion);
            hexagon->update();
            p1->update(switch_flag,which_mine);
            p2->update();
            fresh_flag++;
        }
        hexagon->render(seq++);
        vTaskDelay(20);
        motion_select->render(seq++);
        vTaskDelay(20);
        p1->render(seq++);
        vTaskDelay(20);
        p2->render(seq++);
        vTaskDelay(20);
    }
}