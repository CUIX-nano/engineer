#include "UI_group.h"
#include <stdlib.h>
extern "C"
{
#include "CRC8_CRC16.h"
}
UI_group::UI_group(uint32_t index,pyro::uart_drv_t *uart)
{
    this->_index = index;    
    this->_header.SOF = 0xA5;
    this->_cmd_id = 0x301;
    this->_uart = uart;
    this->_UI_objs.clear();
    this->_UI_objs.clear();
}

UI_group::~UI_group()
{
    this->_UI_objs.clear();
}

void UI_group::init()
{
    this->_update_count = rand()%(this->_update_cycle/this->_task_cycle);
}

void UI_group::add_UI_obj(UI_obj* obj)
{
    this->_UI_objs.push_back(obj);
}

void UI_group::render(uint8_t seq)
{
    // this->_update_count = (this->_update_count+1)%(this->_update_cycle/this->_task_cycle);

    // if(this->_update_count!=0)
    // {
    //     return;
    // }


    switch(_UI_objs.size())
    {
        case 1:_sub_id = 0x0101;break;
        case 2:_sub_id = 0x0102;break;
        case 5:_sub_id = 0x0103;break;
        case 7:_sub_id = 0x0104;break;
        default:return;
    }
    // if(_UI_num!=_UI_objs.size())
    // {
        _UI_num = _UI_objs.size();
    //     this->_data.clear();
    //     this->_data.resize(_UI_num*sizeof(UI_obj::UI_struct)+6);
    //     // this->_buf.resize(5+2+_data.size()+2);
    // }
    uint16_t length = _UI_num*sizeof(UI_obj::UI_struct)+6;
    _header.data_length = length;
    _header.seq = seq;
    append_CRC8_check_sum((unsigned char*)&_header,sizeof(frame_header_struct_t));
    _data[1] = (_sub_id>>8)&0xff;
    _data[0] = _sub_id&0xff;
    _data[3] = (_sender_id>>8)&0xff;
    _data[2] = _sender_id&0xff;
    _data[5] = (_receiver_id>>8)&0xff;
    _data[4] = _receiver_id&0xff;
    for(int i = 0;i<_UI_num;i++)
    {
        memcpy(&_data[6+i*UI_obj::UI_bin_buf_size],_UI_objs[i]->get_UI_bin_buf(),UI_obj::UI_bin_buf_size);
    }
    memcpy(&_buf[0],&_header,sizeof(frame_header_struct_t));
    memcpy(&_buf[sizeof(frame_header_struct_t)],&_cmd_id,sizeof(uint16_t));
    memcpy(&_buf[sizeof(frame_header_struct_t)+sizeof(uint16_t)],_data,length);
    uint16_t total_size = 5+2+length+2;
    append_CRC16_check_sum((uint8_t*)&_buf[0],total_size);

    _uart->write(_buf,total_size,100);
}

void UI_group::set_task_cycle(uint8_t cycle)
{
    this->_task_cycle = cycle;
}

void UI_group::set_update_cycle(uint8_t update_rate)
{
    this->_update_cycle = update_rate;
}

void UI_group::set_sender_id(uint32_t sender_id)
{
    this->_sender_id = sender_id;
    this->_sender_id = sender_id;
    switch (this->_sender_id)
    {
        case robot_id_t::RED_HERO:
        this->_receiver_id = robot_player_id_t::RED_HERO_PLAYER;
        break;
        case robot_id_t::RED_ENGINEER:
        this->_receiver_id = robot_player_id_t::RED_ENGINEER_PLAYER;
        break;
        case robot_id_t::RED_STANDARD_1:
        this->_receiver_id = robot_player_id_t::RED_STANDARD_1_PLAYER;
        break;
        case robot_id_t::RED_STANDARD_2:
        this->_receiver_id = robot_player_id_t::RED_STANDARD_2_PLAYER;
        break;
        case robot_id_t::RED_STANDARD_3:
        this->_receiver_id = robot_player_id_t::RED_STANDARD_3_PLAYER;
        break;
        case robot_id_t::RED_AERIAL:
        this->_receiver_id = robot_player_id_t::RED_AERIAL_PLAYER;
        break;
        case robot_id_t::BLUE_HERO:
        this->_receiver_id = robot_player_id_t::BLUE_HERO_PLAYER;
        break;
        case robot_id_t::BLUE_ENGINEER:
        this->_receiver_id = robot_player_id_t::BLUE_ENGINEER_PLAYER;
        break;
        case robot_id_t::BLUE_STANDARD_1:
        this->_receiver_id = robot_player_id_t::BLUE_STANDARD_1_PLAYER;
        break;
        case robot_id_t::BLUE_STANDARD_2:
        this->_receiver_id = robot_player_id_t::BLUE_STANDARD_2_PLAYER;
        break;
        case robot_id_t::BLUE_STANDARD_3:
        this->_receiver_id = robot_player_id_t::BLUE_STANDARD_3_PLAYER;
        break;
        case robot_id_t::BLUE_AERIAL:
        this->_receiver_id = robot_player_id_t::BLUE_AERIAL_PLAYER;
        break;
        default:
        break;
    }
}