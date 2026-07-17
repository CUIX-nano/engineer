#include "pyro_databoard.h"
#include "pyro_mutex.h"
#include <string.h>

namespace pyro
{
    topic::topic(const char *name, data_type_t type)
    : _mtx()
    {
        this->_name = new char[strlen(name)+1];
        strcpy(this->_name, name);
        this->_type = type;
        this->_timestamp = xTaskGetTickCount();
        _valid = false;
    }

    topic::~topic()
    {
        delete[] _name;
        // mutex_t内部析构自动销毁信号量，无需手动vSemaphoreDelete
    }

    bool topic::operator==(const topic &other)
    {
        return (strcmp(this->_name, other._name) == 0);
    }

    topic::data_status_t topic::write(genenral_data_t& data)
    {
        // RAII作用域锁，离开作用域自动释放，不会漏Give
        scoped_mutex_t lock(_mtx);
        if (!lock.is_locked())
        {
            return DATA_ERROR;
        }

        switch(_type)
        {
            case UNSIGNED_INT:
                _data.data_ui = data.data_ui;
                break;
            case SIGNED_INT:
                _data.data_si = data.data_si;
                break;
            case FLOAT:
                _data.data_f = data.data_f;
                break;
        }
        _timestamp = xTaskGetTickCount();
        _valid = true;
        return DATA_OK;
    }

    topic::data_status_t topic::read(genenral_data_t* data)
    {
        scoped_mutex_t lock(_mtx);
        if (!lock.is_locked())
        {
            return DATA_ERROR;
        }

        if(!_valid)
        {
            return DATA_INVALID;
        }
        switch(_type)
        {
            case UNSIGNED_INT:
                data->data_ui = _data.data_ui;
                break;
            case SIGNED_INT:
                data->data_si = _data.data_si;
                break;
            case FLOAT:
                data->data_f = _data.data_f;
                break;
        }
        return DATA_OK;
    };

    const char* topic::get_name()
    {
        return _name;
    }

    TickType_t topic::get_timestamp()
    {
        scoped_mutex_t lock(_mtx);
        if (!lock.is_locked())
        {
            return 0;
        }
        return _timestamp;
    }

    databoard::databoard()
    {
        for(int i = 0; i <= DATABOARD_CHANNEL_COUNT; i++)
        {
            _topics[i] = nullptr;
        }
        // 全局通道锁使用封装mutex_t
    }

    databoard::~databoard()
    {
        scoped_mutex_t lock(_board_mtx);
        if (!lock.is_locked())
        {
            // 析构阶段无法阻塞，资源泄漏风险，工程可加断言
            configASSERT(false);
        }
        for(int i = 0; i <= DATABOARD_CHANNEL_COUNT; i++)
        {
            if(_topics[i] != nullptr)
            {
                delete _topics[i];
            }
        }
    }

    uint32_t databoard::create_topic(const char *name, data_type_t type)
    {
        topic *new_topic = nullptr;
        uint32_t id = 0xFFFFFFFF;
        scoped_mutex_t lock(_board_mtx);
        if (!lock.is_locked())
        {
            return id;
        }

        // 查重同名topic
        for(int i = 1; i <= DATABOARD_CHANNEL_COUNT; i++)
        {
            if(_topics[i] != nullptr)
            {
                if(strcmp(_topics[i]->get_name(), name) == 0)
                {
                    return i;
                }
            }
        }
        new_topic = new topic(name, type);
        // 寻找空通道
        for (int i = 1; i <= DATABOARD_CHANNEL_COUNT; i++)
        {
            if(_topics[i] == nullptr)
            {
                _topics[i] = new_topic;
                id = i;
                return id;
            }
        }
        // 无空闲通道，销毁新建topic
        delete new_topic;
        return id;
    }

    uint32_t databoard::get_topic_id(const char *name)
    {
        scoped_mutex_t lock(_board_mtx);
        if (!lock.is_locked())
        {
            return 0xFFFFFFFF;
        }
        for(int i = 1; i <= DATABOARD_CHANNEL_COUNT; i++)
        {
            if(_topics[i] != nullptr)
            {
                if(strcmp(_topics[i]->get_name(), name) == 0)
                {
                    return i;
                }
            }
        }
        return 0xFFFFFFFF;
    }

    bool databoard::delete_topic(const char *name)
    {
        scoped_mutex_t lock(_board_mtx);
        if (!lock.is_locked())
        {
            return false;
        }
        for(int i = 1; i <= DATABOARD_CHANNEL_COUNT; i++)
        {
            if(_topics[i] != nullptr)
            {
                if(strcmp(_topics[i]->get_name(), name) == 0)
                {
                    delete _topics[i];
                    _topics[i] = nullptr;
                    return true;
                }
            }
        }
        return false;
    }

    topic::data_status_t databoard::read(uint32_t id, genenral_data_t* data,TickType_t& timestamp)
    {
        if(id > DATABOARD_CHANNEL_COUNT || id <= 0)
        {
            return topic::DATA_ERROR;
        }
        scoped_mutex_t lock(_board_mtx);
        if (!lock.is_locked())
        {
            return topic::DATA_ERROR;
        }
        if(_topics[id] != nullptr)
        {
            topic::data_status_t status = _topics[id]->read(data);
            if(status == topic::DATA_OK)
            {
                timestamp = _topics[id]->get_timestamp();
                return status;
            }
            else
            {
                return status;
            }
        }
        return topic::DATA_ERROR;
    }

    topic::data_status_t databoard::write_topic(uint32_t id, genenral_data_t data)
    {
        if(id > DATABOARD_CHANNEL_COUNT || id <= 0)
        {
            return topic::DATA_ERROR;
        }
        scoped_mutex_t lock(_board_mtx);
        if (!lock.is_locked())
        {
            return topic::DATA_ERROR;
        }
        if(_topics[id] != nullptr)
        {
            return _topics[id]->write(data);
        }
        return topic::DATA_ERROR;
    }
};
