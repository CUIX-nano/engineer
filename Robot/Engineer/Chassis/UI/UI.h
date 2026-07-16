#ifndef UI_H
#define UI_H

#include <cstdint>
#include <array>
#include <cstring>
#include <string>
#include "memory.h"
#include <vector>

class UI_obj
{
    public:
        const static uint8_t UI_bin_buf_size = 15;
        typedef enum 
        {
            None = 0,
            UI_Create = 1,
            UI_Modify = 2,
            UI_Delete = 3,
        }
        operation_type_e;

        typedef enum
        {
            Line=0,
            Rectangle,
            Circle,
            Ellipse,
            Curve,
            Float,
            Integer,
            Character
        }
        graphic_type_e;

        typedef enum
        {
            blue_or_red=0,
            yellow,
            green,
            orange,
            purple,
            pink,
            cyan,
            black,
            white
        }
        color_e;
        
        typedef struct __attribute__((packed))
        {
            std::array<uint8_t, 3> UI_Index;
            operation_type_e operate_type:3;
            graphic_type_e graphic_type:3;
            uint8_t graphic_layer:4;
            color_e graphic_color:4;
            uint16_t details_a:9;
            uint16_t details_b:9;
            uint16_t width:10;
            uint16_t start_x:11;
            uint16_t start_y:11;
            uint16_t details_c:10;
            uint16_t details_d:11;
            uint16_t details_e:11;
        }
        UI_struct;

        UI_obj(std::string str);

        void start(void);
        void update(void);
        uint8_t* get_UI_bin_buf(void);
    protected:
        std::string _str;
        UI_struct _UI_data;
        std::array<uint8_t, 15> UI_bin_buf;
};

class UI_Line:public UI_obj
{
    public:
        UI_Line(std::string str,uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t line_width,uint8_t layer, color_e color);
        void update(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t line_width,uint8_t layer, color_e color);
};

class UI_Rectangle:public UI_obj
{
    public:
        UI_Rectangle(std::string str,uint16_t start_x, uint16_t start_y, uint16_t width, uint16_t height, uint16_t line_width,uint8_t layer, color_e color);
        void update(uint16_t start_x, uint16_t start_y, uint16_t width, uint16_t height, uint16_t line_width,uint8_t layer, color_e color);
};

class UI_Circle:public UI_obj
{
    public:
        UI_Circle(std::string str,uint16_t center_x, uint16_t center_y, uint16_t line_width,uint16_t radius, uint8_t layer, color_e color);
        void update(uint16_t center_x, uint16_t center_y,uint16_t line_width, uint16_t radius, uint8_t layer, color_e color);
};

class UI_Ellipse:public UI_obj
{
    public:
        UI_Ellipse(std::string str,uint16_t center_x, uint16_t center_y, uint16_t line_width,uint16_t radius_a, uint16_t radius_b, uint8_t layer, color_e color);
        void update(uint16_t center_x, uint16_t center_y,uint16_t line_width, uint16_t radius_a, uint16_t radius_b, uint8_t layer, color_e color);
};

class UI_Curve:public UI_obj
{
    public:
        UI_Curve(std::string str,uint16_t center_x, uint16_t center_y, uint16_t line_width,uint16_t start_angle, uint16_t end_angle, uint16_t half_x, uint16_t half_y, uint8_t layer, color_e color);
        void update(uint16_t center_x, uint16_t center_y,uint16_t line_width, uint16_t start_angle, uint16_t end_angle, uint16_t half_x, uint16_t half_y, uint8_t layer, color_e color);
};

class UI_Float:public UI_obj
{
    public:
        UI_Float(std::string str,uint16_t x, uint16_t y,uint16_t line_width, float value, uint16_t character_size,uint8_t layer, color_e color);
        void update(uint16_t x, uint16_t y,uint16_t line_width, float value,uint16_t character_size ,uint8_t layer, color_e color);
};

class UI_Integer:public UI_obj
{
    public:
        UI_Integer(std::string str,uint16_t x, uint16_t y, uint16_t line_width,int32_t value, uint16_t character_size,uint8_t layer, color_e color);
        void update(uint16_t x, uint16_t y, uint16_t line_width,int32_t value,uint16_t character_size ,uint8_t layer, color_e color);
};

class UI_Character:public UI_obj
{
    public:
        UI_Character(std::string str,uint16_t x, uint16_t y, std::string content, uint16_t character_size,uint8_t layer, color_e color);
        void update(uint16_t x, uint16_t y, std::string content,uint16_t character_size ,uint8_t layer, color_e color);
    private:
        std::string _content;
};

#endif

