#include "UI.h"

UI_obj::UI_obj(std::string str):_str(str)
{
    memset(&_UI_data,0,sizeof(UI_struct));
    for(int i = 0 ; i <3 &&i<str.size();i++)
    {
        _UI_data.UI_Index[i] = _str[i];
    }
}

void UI_obj::start(void)
{
    _UI_data.operate_type = UI_Create;
}

void UI_obj::update(void)
{
    _UI_data.operate_type = UI_Modify;
}

uint8_t* UI_obj::get_UI_bin_buf(void)
{
    memcpy(UI_bin_buf.data(),&_UI_data,sizeof(UI_struct));
    return UI_bin_buf.data();
}

UI_Line::UI_Line(std::string str,uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t line_width,uint8_t layer, color_e color):UI_obj(str)
{
    _UI_data.graphic_type = Line;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = start_x;
    _UI_data.start_y = start_y;
    _UI_data.width = line_width;
    _UI_data.details_d = end_x;
    _UI_data.details_e = end_y;
}

void UI_Line::update(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t line_width,uint8_t layer, color_e color)
{
    _UI_data.graphic_type = Line;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = start_x;
    _UI_data.start_y = start_y;
    _UI_data.width = line_width;
    _UI_data.details_d = end_x;
    _UI_data.details_e = end_y;
}

UI_Rectangle::UI_Rectangle(std::string str,uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t line_width,uint8_t layer, color_e color):UI_obj(str)
{
    _UI_data.graphic_type = Rectangle;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = start_x;
    _UI_data.start_y = start_y;
    _UI_data.width = line_width;
    _UI_data.details_d = end_x;
    _UI_data.details_e = end_y;
}

void UI_Rectangle::update(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t line_width,uint8_t layer, color_e color)
{
    _UI_data.graphic_type = Rectangle;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = start_x;
    _UI_data.start_y = start_y;
    _UI_data.width = line_width;
    _UI_data.details_d = end_x;
    _UI_data.details_e = end_y;
}

UI_Circle::UI_Circle(std::string str,uint16_t center_x, uint16_t center_y,uint16_t line_width, uint16_t radius, uint8_t layer, color_e color):UI_obj(str)
{
    _UI_data.graphic_type = Circle;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = center_x;
    _UI_data.start_y = center_y;
    _UI_data.width = line_width;
    _UI_data.details_c = radius;
}

void UI_Circle::update(uint16_t center_x, uint16_t center_y,uint16_t line_width, uint16_t radius, uint8_t layer, color_e color)
{
    _UI_data.graphic_type = Circle;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = center_x;
    _UI_data.start_y = center_y;
    _UI_data.width = line_width;
    _UI_data.details_c = radius;
}

UI_Ellipse::UI_Ellipse(std::string str,uint16_t center_x, uint16_t center_y,uint16_t line_width, uint16_t radius_a, uint16_t radius_b, uint8_t layer, color_e color):UI_obj(str)
{
    _UI_data.graphic_type = Ellipse;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = center_x;
    _UI_data.start_y = center_y;
    _UI_data.width = line_width;
    _UI_data.details_a = radius_a;
    _UI_data.details_b = radius_b;
}

void UI_Ellipse::update(uint16_t center_x, uint16_t center_y,uint16_t line_width, uint16_t radius_a, uint16_t radius_b, uint8_t layer, color_e color)
{
    _UI_data.graphic_type = Ellipse;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = center_x;
    _UI_data.start_y = center_y;
    _UI_data.width = line_width;
    _UI_data.details_a = radius_a;
    _UI_data.details_b = radius_b;
}

UI_Curve::UI_Curve(std::string str,uint16_t center_x, uint16_t center_y, uint16_t line_width,uint16_t start_angle, uint16_t end_angle, uint16_t half_x, uint16_t half_y, uint8_t layer, color_e color):UI_obj(str)
{
    _UI_data.graphic_type = Curve;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = center_x;
    _UI_data.start_y = center_y;
    _UI_data.width = line_width;
    _UI_data.details_a = start_angle;
    _UI_data.details_b = end_angle;
    _UI_data.details_c = half_x;
    _UI_data.details_d = half_y;
}

void UI_Curve::update(uint16_t center_x, uint16_t center_y,uint16_t line_width, uint16_t start_angle, uint16_t end_angle, uint16_t half_x, uint16_t half_y, uint8_t layer, color_e color)
{
    _UI_data.graphic_type = Curve;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = center_x;
    _UI_data.start_y = center_y;
    _UI_data.width = line_width;
    _UI_data.details_a = start_angle;
    _UI_data.details_b = end_angle;
    _UI_data.details_c = half_x;
    _UI_data.details_d = half_y;
}

UI_Float::UI_Float(std::string str,uint16_t x, uint16_t y, uint16_t line_width,float value, uint16_t character_size,uint8_t layer, color_e color):UI_obj(str)
{
    _UI_data.graphic_type = Float;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = x;
    _UI_data.start_y = y;
    _UI_data.width = line_width;
    value*=1000;
    _UI_data.details_a = character_size;
    memcpy(((uint8_t*)&(_UI_data))+11,&value,sizeof(float));
}

void UI_Float::update(uint16_t x, uint16_t y,uint16_t line_width, float value,uint16_t character_size ,uint8_t layer, color_e color)
{
    _UI_data.graphic_type = Float;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = x;
    _UI_data.start_y = y;
    _UI_data.width = line_width;
    value*=1000;
    _UI_data.details_a = character_size;
    memcpy(((uint8_t*)&(_UI_data))+11,&value,sizeof(float));
}

UI_Integer::UI_Integer(std::string str,uint16_t x, uint16_t y,uint16_t line_width, int32_t value, uint16_t character_size,uint8_t layer, color_e color):UI_obj(str)
{
    _UI_data.graphic_type = Integer;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = x;
    _UI_data.start_y = y;
    _UI_data.width = line_width;
    _UI_data.details_a = character_size;
    memcpy(((uint8_t*)&(_UI_data))+11,&value,sizeof(int32_t));
}

void UI_Integer::update(uint16_t x, uint16_t y, uint16_t line_width,int32_t value,uint16_t character_size ,uint8_t layer, color_e color)
{
    _UI_data.graphic_type = Integer;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = x;
    _UI_data.start_y = y;
    _UI_data.width = line_width;
    _UI_data.details_a = character_size;
    memcpy(((uint8_t*)&(_UI_data))+11,&value,sizeof(int32_t));
}

UI_Character::UI_Character(std::string str,uint16_t x, uint16_t y, std::string content, uint16_t character_size,uint8_t layer, color_e color):UI_obj(str)
{
    _UI_data.graphic_type = Character;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = x;
    _UI_data.start_y = y;
    _UI_data.details_b = content.length();
    _UI_data.details_a = character_size;
    _content = content;
}

void UI_Character::update(uint16_t x, uint16_t y, std::string content,uint16_t character_size ,uint8_t layer, color_e color)
{
    _UI_data.graphic_type = Character;
    _UI_data.graphic_layer = layer;
    _UI_data.graphic_color = color;
    _UI_data.start_x = x;
    _UI_data.start_y = y;
    _UI_data.details_b = content.length();
    _UI_data.details_a = character_size;
    _content = content;
}