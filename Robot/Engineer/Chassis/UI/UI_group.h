#ifndef UI_GROUP_H
#define UI_GROUP_H

#include "UI.h"

#include <vector>
#include "memory.h"

#include "pyro_uart_drv.h"
#include "referee.h"


class UI_group
{
    public:
        UI_group(uint32_t index,pyro::uart_drv_t *uart);
        ~UI_group();
        void init();
        void add_UI_obj(UI_obj* obj);
        void render(uint8_t seq);
        void set_task_cycle(uint8_t cycle);
        void set_update_cycle(uint8_t update_rate);
        void set_sender_id(uint32_t sender_id);

    private:
        uint32_t _index;
        pyro::uart_drv_t *_uart;
        
        uint8_t _UI_num;
        frame_header_struct_t _header;
        uint16_t _cmd_id;
        uint16_t _sub_id;
        uint32_t _sender_id;
        uint32_t _receiver_id;
        uint8_t _data[128];
        uint16_t _crc_16;
        // std::vector<uint8_t> _buf;
        uint8_t _buf[256];

        uint8_t _task_cycle;
        uint8_t _update_cycle;

        uint8_t _update_count;
    protected:
        std::vector<UI_obj*> _UI_objs;
};


#endif