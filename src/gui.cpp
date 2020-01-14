#include "gui.hpp"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <array>
#include <string>
#include <sstream>
#include <iomanip>
//#include "odometry.hpp"
#include "op.hpp"
#include "motors.hpp"
#include "sensors.hpp"
#include "utilities.hpp"
#include <vector>
//#include "enver_logo.h"

LV_IMG_DECLARE(logo);

static lv_obj_t *g_btn_region; //tab view region of the screen
static lv_obj_t *g_sb_region; //status bar region of the screen
static lv_obj_t *g_sb_label;  // sb text label
static lv_style_t s_white;
static lv_style_t s_black;
static lv_style_t s_white_num;
static lv_style_t s_red;
static lv_style_t s_sm_text;


static lv_style_t s_btn_rel;
static lv_style_t s_btn_pr;

static lv_style_t s_field_cont;
static lv_style_t s_field;
static lv_style_t s_robot;
static lv_style_t s_line;


lv_theme_t *th;
void theme(){
th = lv_theme_alien_init(10, NULL);
}

void style_white(){
  lv_style_copy(&s_white, &lv_style_plain);
  s_white.text.font = &lv_font_dejavu_20;
  s_white.text.letter_space = 2;
  s_white.text.line_space = 2;
  s_white.text.color = LV_COLOR_WHITE;
}

void style_num(){
  lv_style_copy(&s_white_num, &lv_style_pretty);
  s_white_num.text.font = &lv_font_dejavu_20;
  s_white_num.text.letter_space = 5;
  s_white_num.text.line_space = 2;
  s_white_num.text.color = LV_COLOR_WHITE;
}

void style_black(){
  lv_style_copy(&s_black, &lv_style_plain);
  s_black.text.font = &lv_font_dejavu_20;
  s_black.text.letter_space = 2;
  s_black.text.line_space = 1;
  s_black.text.color = LV_COLOR_BLACK;
}

void style_red(){
  lv_style_copy(&s_red, &lv_style_btn_tgl_pr);
  s_red.text.font = &lv_font_dejavu_20;
  s_red.text.letter_space = 5;
  s_red.text.line_space = 2;
  s_red.text.color = LV_COLOR_RED;
}

void style_btn_rel(){
  lv_style_copy(&s_btn_rel, &lv_style_btn_tgl_pr);
  s_btn_rel.text.font = &lv_font_dejavu_20;
  s_btn_rel.text.letter_space = 5;
  s_btn_rel.text.line_space = 2;
  s_btn_rel.text.color = LV_COLOR_RED;
  s_btn_rel.body.main_color = LV_COLOR_MAKE(0x30, 0x30, 0x30);
  s_btn_rel.body.grad_color = LV_COLOR_BLACK;
  s_btn_rel.body.border.color = LV_COLOR_SILVER;
}

void style_sm_text(){
  lv_style_copy(&s_sm_text, &lv_style_plain);
  s_sm_text.text.font = &lv_font_dejavu_10;
  s_sm_text.text.letter_space = 1;
  s_sm_text.text.line_space = 1;
  s_sm_text.text.color = LV_COLOR_WHITE;
}


void field_cont_style(){
lv_style_copy(&s_field_cont ,&lv_style_plain);
s_field_cont.body.main_color = LV_COLOR_BLACK;
s_field_cont.body.grad_color = LV_COLOR_BLACK;
s_field_cont.body.border.width = 0;
s_field_cont.body.radius = 0;
}

void field_style(){
  lv_style_copy(&s_field ,&lv_style_plain);
  s_field_cont.body.main_color = LV_COLOR_WHITE;
  s_field_cont.body.grad_color = LV_COLOR_WHITE;
  s_field_cont.body.border.width = 0;
  s_field_cont.body.radius = 0;
}

void robot_style(){
  s_robot.body.radius = LV_RADIUS_CIRCLE;
  s_robot.body.main_color = LV_COLOR_RED;
  s_robot.body.grad_color = LV_COLOR_RED;
  s_robot.body.border.color = LV_COLOR_WHITE;
  s_robot.body.border.width = 2;
  s_robot.body.border.opa = LV_OPA_100;
}

void line_style(){
  s_line.line.width = 3;
  s_line.line.opa = LV_OPA_100;
  s_line.line.color = LV_COLOR_GREEN;
}



int auton_sel = 0;

static lv_res_t btnm_action(lv_obj_t * btnm, const char *txt) {

  int btnm_num = atoi(txt);

  switch (btnm_num) {
  case 1:
    lv_label_set_text(g_sb_label, "Red Right Auton");
    auton_sel = 1;
    break;
  case 2:
    lv_label_set_text(g_sb_label, "Red Left Auton");
    auton_sel = 2;
    break;
  case 3:
    lv_label_set_text(g_sb_label, "Blue Right Auton");
    auton_sel = 3;
break;
  case 4:
    lv_label_set_text(g_sb_label, "Blue Left Auton");
    auton_sel = 4;
break;
  case 5:
    lv_label_set_text(g_sb_label, "Skills Auton1");
    auton_sel = 5;
break;
  case 6:
    lv_label_set_text(g_sb_label, "Skills Auton2");
    auton_sel = 6;
break;
  }

  lv_obj_align(g_sb_label, NULL, LV_ALIGN_CENTER, 0, 0); // must be after set_text

  return LV_RES_OK; /*Return OK because the button matrix is not deleted*/
}

static lv_res_t btn_return_f(lv_obj_t * btn) {
  gui();
   return LV_RES_OK; /*Return OK if the button is not deleted*/
}

static lv_res_t btn_reset_f(lv_obj_t * btn) {
  //Position reset
   pos.set_x(0.0);
   pos.set_y(0.0);
   pos.set_alpha(0.0);

//Velcoity reset
   velo.set_vel_x(0.0);
   velo.set_vel_y(0.0);
   velo.set_vel_a(0.0);

   return LV_RES_OK; /*Return OK if the button is not deleted*/
}

lv_obj_t * robot = nullptr;
lv_obj_t * line = nullptr;
std::vector<lv_point_t> linePoints = {{0, 0}, {0, 0}};
double fieldDim = 0.0;
int lineWidth = 0;
int lineLength = 0;

void gui_odom(){
  lv_obj_t * container;
  container = lv_cont_create(lv_scr_act(), NULL);
  lv_obj_set_size(container, lv_obj_get_width(lv_scr_act()), lv_obj_get_height(lv_scr_act()));
  lv_obj_align(container, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style(container, &s_field_cont);


  lv_obj_t * field = lv_obj_create(container, NULL);
  lv_coord_t size = std::min(lv_obj_get_width(container), lv_obj_get_height(container));
  fieldDim = size;
  lv_obj_set_size(field, size, size);
  lv_obj_align(field, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);
  lv_obj_set_style(field, &s_field);

  robot = lv_led_create(field, NULL);
  lv_led_on(robot);
  lv_obj_set_size(robot, lv_obj_get_width(field) / 15, lv_obj_get_height(field) / 15);
  lv_obj_set_style(robot, &s_robot);

  line = lv_line_create(field, NULL);
  lv_obj_set_pos(line, 0, 0);
  lv_obj_set_style(line, &s_line);

  lineWidth = 3;
  lineLength = fieldDim / 6;

}

void odom_update(void* ign){

  while(true){
  lv_line_set_points(line, linePoints.data(), linePoints.size());

  double c_x = pos.get_x();
  double c_y = pos.get_y();
  double c_theta = radians_to_degrees(pos.get_alpha());

  lv_obj_set_pos(robot, c_x, c_y);

linePoints[0] = {(int16_t)((c_x * fieldDim)), (int16_t)((c_y * fieldDim) - (lineWidth/2))};
double newY = lineLength * cos(c_theta);
double newX = lineLength * sin(c_theta);
linePoints[1] = {(int16_t)(newX + linePoints[0].x), (int16_t)(-newY + linePoints[0].y)};

lv_line_set_points(line, linePoints.data(), linePoints.size());
lv_obj_invalidate(line);


  lv_obj_t * x_label2 = lv_label_create(lv_scr_act(), NULL);
  lv_obj_align(x_label2, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 10);
  lv_obj_set_style(x_label2, &s_white_num);

  lv_obj_t * y_label2 = lv_label_create(lv_scr_act(), NULL);
  lv_obj_align(y_label2, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 35);
  lv_obj_set_style(y_label2, &s_white_num);

  lv_obj_t * a_label2 = lv_label_create(lv_scr_act(), NULL);
  lv_obj_align(a_label2, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 60);
  lv_obj_set_style(a_label2, &s_white_num);

  std::ostringstream x2;
  x2 << std::setprecision(4) << pos.get_x();
  auto s12 = x2.str();
  lv_label_set_text(x_label2, s12.c_str());

  std::ostringstream y2;
  y2 << std::setprecision(4) << pos.get_y();
  auto s22 = y2.str();
  lv_label_set_text(y_label2, s22.c_str());

  std::ostringstream a2;
  a2 << std::setprecision(4) << radians_to_degrees(pos.get_alpha());
  auto s32 = a2.str();
  lv_label_set_text(a_label2, s32.c_str());

  pros::delay(30);
  }
  pros::delay(1);
}


void gui_debug() {
lv_obj_t * label_home = lv_label_create(lv_scr_act(), NULL);
lv_label_set_text(label_home, "RETURN");
lv_obj_set_style(label_home, &s_sm_text);
lv_obj_align(label_home, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -25, -75);

lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
lv_btn_set_style(btn1, LV_BTN_STYLE_REL, th->btn.rel);
lv_btn_set_style(btn1, LV_BTN_STYLE_PR, th->btn.pr);
lv_obj_set_size(btn1, 60, 60);
lv_obj_align(btn1, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -50);
//lv_obj_set_free_num(btn1, 1);   /*Set a unique number for the button*/
lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, btn_return_f);

lv_obj_t * label_reset = lv_label_create(lv_scr_act(), NULL);
lv_label_set_text(label_reset, "RESET");
lv_obj_set_style(label_reset, &s_sm_text);
lv_obj_align(label_reset, NULL, LV_ALIGN_IN_RIGHT_MID, -27, -25);

lv_obj_t * btn_reset = lv_btn_create(lv_scr_act(), NULL);
lv_btn_set_style(btn_reset, LV_BTN_STYLE_REL, th->btn.rel);
lv_btn_set_style(btn_reset, LV_BTN_STYLE_PR, th->btn.pr);
lv_obj_set_size(btn_reset, 60, 60);
lv_obj_align(btn_reset, NULL, LV_ALIGN_IN_RIGHT_MID, -10, -25);
//lv_obj_set_free_num(btn_reset, 1);   /*Set a unique number for the button*/
lv_btn_set_action(btn_reset, LV_BTN_ACTION_CLICK, btn_reset_f);

//lv_coord_t size_w = lv_obj_get_width(lv_scr_act())/6;
//lv_coord_t size_l = lv_obj_get_height(lv_scr_act())/6;
lv_obj_t * label = lv_label_create(lv_scr_act(), NULL);
//lv_obj_set_size(label, size_w, size_l);
lv_obj_set_style(label, &s_white);
lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10);
lv_label_set_text(label, "Global X:");

label = lv_label_create(lv_scr_act(), NULL);
lv_obj_set_style(label, &s_white);
lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 35);
lv_label_set_text(label, "Global Y:");

label = lv_label_create(lv_scr_act(), NULL);
lv_obj_set_style(label, &s_white);
lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 60);
lv_label_set_text(label, "Global A:");

label = lv_label_create(lv_scr_act(), NULL);
lv_obj_set_style(label, &s_white);
lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 110);
lv_label_set_text(label, "Vel X:");

label = lv_label_create(lv_scr_act(), NULL);
lv_obj_set_style(label, &s_white);
lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 135);
lv_label_set_text(label, "Vel Y:");


label = lv_label_create(lv_scr_act(), NULL);
lv_obj_set_style(label, &s_white);
lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 160);
lv_label_set_text(label, "Vel A:");



}

void debug_update(void*){
//char x[5];
double g = 5;
lv_obj_t * x_label = lv_label_create(lv_scr_act(), NULL);
lv_obj_align(x_label, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 10);
lv_obj_set_style(x_label, &s_white_num);

lv_obj_t * y_label = lv_label_create(lv_scr_act(), NULL);
lv_obj_align(y_label, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 35);
lv_obj_set_style(y_label, &s_white_num);

lv_obj_t * a_label = lv_label_create(lv_scr_act(), NULL);
lv_obj_align(a_label, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 60);
lv_obj_set_style(a_label, &s_white_num);

lv_obj_t * e360b_label = lv_label_create(lv_scr_act(), NULL);
lv_obj_align(e360b_label, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 85);
lv_obj_set_style(e360b_label, &s_white_num);

lv_obj_t * vX_label = lv_label_create(lv_scr_act(), NULL);
lv_obj_align(vX_label, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 110);
lv_obj_set_style(vX_label, &s_white_num);

lv_obj_t * vY_label = lv_label_create(lv_scr_act(), NULL);
lv_obj_align(vY_label, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 135);
lv_obj_set_style(vY_label, &s_white_num);

lv_obj_t * vA_label = lv_label_create(lv_scr_act(), NULL);
lv_obj_align(vA_label, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 160);
lv_obj_set_style(vA_label, &s_white_num);




while(true){
g++;
std::ostringstream x;
x << std::setprecision(4) << xyz.get_x();//pos.get_x();
auto s1 = x.str();
lv_label_set_text(x_label, s1.c_str());


std::ostringstream y;
y << std::setprecision(4) << pos.get_y();
auto s2 = y.str();
lv_label_set_text(y_label, s2.c_str());

std::ostringstream a;
a << std::setprecision(4) << radians_to_degrees(pos.get_alpha());
auto s3 = a.str();
lv_label_set_text(a_label, s3.c_str());

std::ostringstream e360b;
e360b << std::setprecision(4) << g;//  encoder360B.get_value();//encoder360B.get_value();
auto s4 = e360b.str();
lv_label_set_text(e360b_label, s4.c_str());

std::ostringstream vX;
vX << std::setprecision(4) << velo.get_vel_x();//encoder360B.get_value();
auto s5 = vX.str();
lv_label_set_text(vX_label, s5.c_str());

std::ostringstream vY;
vY << std::setprecision(4) << velo.get_vel_y();//encoder360B.get_value();
auto s6 = vY.str();
lv_label_set_text(vY_label, s6.c_str());

std::ostringstream vA;
vA << std::setprecision(4) << velo.get_vel_a();//encoder360B.get_value();
auto s7 = vA.str();
lv_label_set_text(vA_label, s7.c_str());







/*sprintf(x,"= %d", i);
lv_obj_t * label = lv_label_create(lv_scr_act(), NULL);
lv_label_set_text(label, x);
*/

pros::delay(20);
 }
}

void gui_btnm(void) {
  // Create a button descriptor string array w/ no repeat "\224"
  lv_obj_t * label_home = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(label_home, "RETURN");
  lv_obj_set_style(label_home, &s_sm_text);
  lv_obj_align(label_home, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -25, -75);

  lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
  lv_btn_set_style(btn1, LV_BTN_STYLE_REL, th->btn.rel);
  lv_btn_set_style(btn1, LV_BTN_STYLE_PR, th->btn.pr);
  lv_obj_set_size(btn1, 60, 60);
  lv_obj_align(btn1, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -50);
  lv_obj_set_free_num(btn1, 1);   /*Set a unique number for the button*/
  lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, btn_return_f);

  static const char * btnm_map[] = { "\2241", "\2242", "\2243", "\n",
                                     "\2244", "\2245", "\2246", "" };

  // Create a default button matrix* no repeat
  lv_obj_t *btnm = lv_btnm_create(g_btn_region, NULL);
  lv_obj_set_size(btnm, lv_obj_get_width(g_btn_region),
      lv_obj_get_height(g_btn_region) - 32);

  lv_btnm_set_map(btnm, btnm_map);
  lv_btnm_set_action(btnm, btnm_action);
}



static lv_res_t btn_click_action(lv_obj_t * btn) {

   uint8_t id = lv_obj_get_free_num(btn);
   static char buffer[32];
   auton_sel = id;

   snprintf(buffer, 32, "Selection is %d \n", id);
   lv_label_set_text(g_sb_label, buffer);
   lv_obj_align(g_sb_label, NULL, LV_ALIGN_CENTER, 0, 0); // must be after set_text

   return LV_RES_OK; /*Return OK if the button is not deleted*/
}

void gui_3btn(void) {
  lv_obj_t * label_home = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(label_home, "RETURN");
  lv_obj_set_style(label_home, &s_sm_text);
  lv_obj_align(label_home, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -25, -75);

  lv_obj_t * btn0 = lv_btn_create(lv_scr_act(), NULL);
  lv_btn_set_style(btn0, LV_BTN_STYLE_REL, th->btn.rel);
  lv_btn_set_style(btn0, LV_BTN_STYLE_PR, th->btn.pr);
  lv_obj_set_size(btn0, 60, 60);
  lv_obj_align(btn0, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -50);
  lv_obj_set_free_num(btn0, 1);   /*Set a unique number for the button*/
  lv_btn_set_action(btn0, LV_BTN_ACTION_CLICK, btn_return_f);


 /*Create a title label*/
 lv_obj_t * label = lv_label_create(g_btn_region, NULL);
 lv_label_set_text(label, "Selection Buttons");
 lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

 /*Create a normal button*/
 lv_obj_t * btn1 = lv_btn_create(g_btn_region, NULL);
 lv_btn_set_style(btn1,LV_BTN_STYLE_REL,&lv_style_btn_rel);
 lv_btn_set_style(btn1,LV_BTN_STYLE_PR,&lv_style_btn_pr);
 lv_obj_align(btn1, NULL, LV_ALIGN_IN_LEFT_MID, 30, 0);
 lv_obj_set_free_num(btn1, 1);   /*Set a unique number for the button*/
 lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, btn_click_action);

 /*Add a label to the button*/
 label = lv_label_create(btn1, NULL);
 lv_label_set_text(label, "Sel 1");

 /*Copy the button and set toggled state. (The release action is copied too)*/
 lv_obj_t * btn2 = lv_btn_create(g_btn_region, btn1);
 lv_obj_align(btn2, NULL, LV_ALIGN_CENTER, 0, 0);
 lv_obj_set_free_num(btn2, 2);               /*Set a unique number for the button*/
 lv_btn_set_action(btn2, LV_BTN_ACTION_CLICK, btn_click_action);

 /*Add a label to the toggled button*/
 label = lv_label_create(btn2, NULL);
 lv_label_set_text(label, "Sel 2");

 /*Copy the button and set inactive state.*/
 lv_obj_t * btn3 = lv_btn_create(g_btn_region, btn1);
 lv_obj_align(btn3, NULL, LV_ALIGN_IN_RIGHT_MID, -30, 0);
 lv_obj_set_free_num(btn3, 3);                  /*Set a unique number for the button*/
 lv_btn_set_action(btn3, LV_BTN_ACTION_CLICK, btn_click_action);

 /*Add a label to the inactive button*/
 label = lv_label_create(btn3, NULL);
 lv_label_set_text(label, "Sel 3");
}

static lv_res_t switch_action (lv_obj_t * sw) {
  uint8_t id = lv_obj_get_free_num(sw);
  static char buffer[32];

  snprintf(buffer, 32, "SW%d Toggled to %s\n",id,lv_sw_get_state(sw)?"On":"Off");
  lv_label_set_text(g_sb_label, buffer);
  lv_obj_align(g_sb_label, NULL, LV_ALIGN_CENTER, 0, 0); // must be after set_text

  return LV_RES_OK; /*Return OK if the button is not deleted*/
}

void set_switch_style (lv_obj_t * sw) {
  /*Create styles for the switch*/
  static lv_style_t bg_style;
  static lv_style_t indic_style;
  static lv_style_t knob_on_style;
  static lv_style_t knob_off_style;

  lv_style_copy(&bg_style, &lv_style_pretty);
  bg_style.body.radius = LV_RADIUS_CIRCLE;

  lv_style_copy(&indic_style, &lv_style_pretty_color);
  indic_style.body.radius = LV_RADIUS_CIRCLE;
  indic_style.body.main_color = LV_COLOR_HEX(0x9fc8ef);
  indic_style.body.grad_color = LV_COLOR_HEX(0x9fc8ef);
  indic_style.body.padding.hor = 0;
  indic_style.body.padding.ver = 0;

  lv_style_copy(&knob_off_style, &lv_style_pretty);
  knob_off_style.body.radius = LV_RADIUS_CIRCLE;
  knob_off_style.body.main_color = LV_COLOR_RED;
  knob_off_style.body.grad_color = LV_COLOR_MAROON; //misspelled should be MAROON
  knob_off_style.body.shadow.width = 4;
  knob_off_style.body.shadow.type = LV_SHADOW_BOTTOM;

  lv_style_copy(&knob_on_style, &lv_style_pretty_color);
  knob_on_style.body.radius = LV_RADIUS_CIRCLE;
  knob_on_style.body.main_color = LV_COLOR_LIME;
  knob_on_style.body.grad_color = LV_COLOR_GREEN;
  knob_on_style.body.shadow.width = 4;
  knob_on_style.body.shadow.type = LV_SHADOW_BOTTOM;

  lv_sw_set_style(sw, LV_SW_STYLE_BG, &bg_style);
  lv_sw_set_style(sw, LV_SW_STYLE_INDIC, &indic_style);
  lv_sw_set_style(sw, LV_SW_STYLE_KNOB_ON, &knob_on_style);
  lv_sw_set_style(sw, LV_SW_STYLE_KNOB_OFF, &knob_off_style);
}

void gui_switch(void) {

  lv_obj_t * label_home = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(label_home, "RETURN");
  lv_obj_set_style(label_home, &s_sm_text);
  lv_obj_align(label_home, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -25, -75);

  lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
  lv_btn_set_style(btn1, LV_BTN_STYLE_REL, th->btn.rel);
  lv_btn_set_style(btn1, LV_BTN_STYLE_PR, th->btn.pr);
  lv_obj_set_size(btn1, 60, 60);
  lv_obj_align(btn1, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -50);
  lv_obj_set_free_num(btn1, 1);   /*Set a unique number for the button*/
  lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, btn_return_f);

  /*Create a title label*/
  lv_obj_t * label = lv_label_create(g_btn_region, NULL);
  lv_label_set_text(label, "Flip Switches");
  lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

  lv_obj_t * sw1 = lv_sw_create(g_btn_region, NULL);
  lv_obj_set_free_num(sw1, 1);                  /*Set a unique number for the object*/
  set_switch_style(sw1);  // style is in separate function for cleaner code
  lv_obj_align(sw1, NULL, LV_ALIGN_IN_LEFT_MID, 50, 0);

  lv_obj_t * sw2 = lv_sw_create(g_btn_region, sw1); // copy sw1 to sw2
  lv_obj_set_free_num(sw2, 2);                  /*Set a unique number for the object*/
  lv_obj_align(sw2, NULL, LV_ALIGN_IN_RIGHT_MID, -50, 0);

  // both switches use the same call back function
  lv_sw_set_action(sw1, switch_action);
  lv_sw_set_action(sw2, switch_action);
}

lv_obj_t * gauge1;
lv_obj_t * gauge2;
lv_obj_t * gauge3;

void gui_gauges(void) {

  lv_obj_t * label_home = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(label_home, "RETURN");
  lv_obj_set_style(label_home, &s_sm_text);
  lv_obj_align(label_home, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -25, -75);

  lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
  lv_btn_set_style(btn1, LV_BTN_STYLE_REL, th->btn.rel);
  lv_btn_set_style(btn1, LV_BTN_STYLE_PR, th->btn.pr);
  lv_obj_set_size(btn1, 60, 60);
  lv_obj_align(btn1, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -50);
  lv_obj_set_free_num(btn1, 1);   /*Set a unique number for the button*/
  lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, btn_return_f);

  /*Create a style*/
  static lv_style_t style;
  lv_style_copy(&style, &lv_style_pretty_color);
  style.body.main_color = LV_COLOR_HEX3(0x666);     /*Line color at the beginning*/
  style.body.grad_color =  LV_COLOR_HEX3(0x666);    /*Line color at the end*/
  style.body.padding.hor = 10;                      /*Scale line length*/
  style.body.padding.inner = 8 ;                    /*Scale label padding*/
  style.body.border.color = LV_COLOR_HEX3(0x333);   /*Needle middle circle color*/
  style.line.width = 2;
  style.text.color = LV_COLOR_HEX3(0x333);
  style.line.color = LV_COLOR_RED;                  /*Line color after the critical value*/

  /*Describe the color for the needles*/

  /*Create a gauge*/
  lv_coord_t gauge_size =  lv_obj_get_width(lv_scr_act())/3-10;
  gauge1 = lv_gauge_create(lv_scr_act(), NULL);
  lv_gauge_set_style(gauge1, &style);
  lv_obj_set_size(gauge1, gauge_size, gauge_size);
  lv_obj_align(gauge1, NULL, LV_ALIGN_IN_TOP_LEFT, 5, 10);

  /*Create a gauge*/
  gauge2 = lv_gauge_create(lv_scr_act(), NULL);
  lv_gauge_set_style(gauge2, &style);
  lv_obj_set_size(gauge2, gauge_size, gauge_size);
  lv_obj_align(gauge2, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);

  /*Create a gauge*/
  gauge3 = lv_gauge_create(lv_scr_act(), NULL);
  lv_gauge_set_style(gauge3, &style);
  lv_obj_set_size(gauge3, gauge_size, gauge_size);
  lv_obj_align(gauge3, NULL, LV_ALIGN_IN_TOP_RIGHT, -5, 10);

}

void gauge_update(void* param) {
  /*Set the values*/
  int i=23;
  while (1) {
    lv_gauge_set_value(gauge1, 0, 1*i%100);
    lv_gauge_set_value(gauge1, 1, i%2?40:50);
    lv_gauge_set_value(gauge2, 0, 2*i%100);
    lv_gauge_set_value(gauge3, 0, 3*i%100);
    pros::Task::delay(2500);
    i++;
  }
}

uint8_t demo_id = 0;

static lv_res_t demo_click_action(lv_obj_t * btn) {
   demo_id = lv_obj_get_free_num(btn);

   g_btn_region = lv_obj_create(lv_scr_act(), NULL);
   lv_obj_set_size(g_btn_region, lv_obj_get_width(lv_scr_act()),
       lv_obj_get_height(lv_scr_act()) * 0.8);
   lv_obj_align(g_btn_region, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);
   lv_obj_set_style(g_btn_region, &lv_style_pretty_color);

   //
   g_sb_region = lv_obj_create(lv_scr_act(), NULL);
   lv_obj_set_size(g_sb_region, lv_obj_get_width(lv_scr_act()),
       lv_obj_get_height(lv_scr_act()) * 0.2);
   lv_obj_align(g_sb_region, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
   lv_obj_set_style(g_sb_region, &lv_style_pretty_color);

   g_sb_label = lv_label_create(g_sb_region, NULL);
   lv_obj_set_style(g_sb_label, &s_btn_rel);
   lv_obj_align(g_sb_label, NULL, LV_ALIGN_CENTER, 0, 0);

   if (demo_id==1) {
    gui_btnm();
  } else if (demo_id==2) {
    gui_3btn();
  } else if (demo_id==3) {
    gui_switch();
  } else if (demo_id == 4){
    gui_gauges();
  //  pros::Task telm_task (gauge_update, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "guage");
  }
  else if (demo_id == 5){
  gui_debug();
  pros::Task debug_task(debug_update, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "LVGL: debug gui");
  }

  else if(demo_id == 6){
  gui_odom();
  pros::Task odom_task(odom_update, nullptr , TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "LVGL: odom gui");
  }

   return LV_RES_OK; /*Return OK if the button is not deleted*/
}

static lv_style_t style_sb;
lv_color_t black = LV_COLOR_MAKE(0,0,0);
void page(){
lv_style_copy(&style_sb, &lv_style_plain);
style_sb.body.main_color = LV_COLOR_BLACK;
style_sb.body.grad_color =  LV_COLOR_BLACK;
style_sb.body.border.color = LV_COLOR_RED;
style_sb.body.border.width = 10;
style_sb.body.border.opa = LV_OPA_MAX;
style_sb.body.radius = 0;
style_sb.body.opa = LV_OPA_MAX;
}

static lv_style_t style_sb2;
void page2(){
lv_style_copy(&style_sb2, &lv_style_plain);
//style_sb.body.empty=0;
style_sb.body.border.color = LV_COLOR_RED;
style_sb.body.border.width = 2;
style_sb.body.border.opa = 255;
style_sb.body.radius = LV_RADIUS_CIRCLE;
}

void gui_init() {
 //Intialize styles
//   lv_theme_t *th = lv_theme_alien_init(10, NULL);
   theme();
   style_white();
   style_black();
   style_num();
   style_red();
   style_btn_rel();
   style_sm_text();
   page();
   field_cont_style();
   field_style();
   robot_style();
   line_style();

   lv_theme_set_current(th);
}

   void gui(){
   lv_obj_t * box1;
   box1 = lv_cont_create(lv_scr_act(), NULL);
   lv_cont_set_fit(box1, true, true);
   lv_obj_set_style(box1, &style_sb);
   lv_obj_align(box1, NULL, LV_ALIGN_CENTER, -180, -78);//-150, -75);
   lv_obj_set_size(box1, lv_obj_get_width(lv_scr_act())+5, lv_obj_get_height(lv_scr_act()));
// -25 , -25
  /* lv_obj_t * page_ = lv_page_create(lv_scr_act(), NULL);
   lv_obj_set_size(page_, lv_obj_get_width(lv_scr_act())-10, lv_obj_get_width(lv_scr_act())-10);
   lv_obj_align(page_, NULL, LV_ALIGN_CENTER, 0, 0);
   lv_page_set_style(page_, LV_PAGE_STYLE_BG, &style_sb2);
*/
  lv_obj_t * img1 = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img1, &logo);
  lv_obj_align(img1, NULL, LV_ALIGN_CENTER, 0, 0);

   lv_coord_t x = lv_obj_get_width(lv_scr_act());
   lv_coord_t y = 15;
  // Select page
  /*Create a title label*/
  lv_obj_t * label = lv_label_create(lv_scr_act(), NULL);
//  lv_obj_set_width(label, 300);
  lv_label_set_text(label, "4549 Robotics");
  lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, -25, 1);
  lv_obj_set_style(label, &s_red);
  lv_obj_set_width(label, x);
  lv_obj_set_height(label, y);


  lv_coord_t btn_width = (lv_obj_get_width(lv_scr_act())/4)-10;
  lv_coord_t btn_height =  btn_width/2;

  /*Create a normal button*/
  lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_size(btn1, btn_width, btn_height);
  lv_obj_align(btn1, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 15);
  //lv_obj_set_style(btn1, &s_btn_rel);
  lv_obj_set_free_num(btn1, 1);   /*Set a unique number for the button*/
  lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, demo_click_action);

  /*Add a label to the button*/
  label = lv_label_create(btn1, NULL);
  lv_label_set_text(label, "AUTO");

  /*Copy the button and set toggled state. (The release action is copied too)*/
  lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_size(btn2, btn_width, btn_height);
  lv_obj_align(btn2, NULL, LV_ALIGN_IN_LEFT_MID, 10, 0);
  lv_obj_set_free_num(btn2, 2);               /*Set a unique number for the button*/
  lv_btn_set_action(btn2, LV_BTN_ACTION_CLICK, demo_click_action);

  /*Add a label to the toggled button*/
  label = lv_label_create(btn2, NULL);
  lv_label_set_text(label, "YEET");

  /*Copy the button and set toggled state. (The release action is copied too)*/
  lv_obj_t * btn3 = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_size(btn3, btn_width, btn_height);
  lv_obj_align(btn3, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 15);
  lv_obj_set_free_num(btn3, 3);               /*Set a unique number for the button*/
  lv_btn_set_action(btn3, LV_BTN_ACTION_CLICK, demo_click_action);

  /*Add a label to the toggled button*/
  label = lv_label_create(btn3, NULL);
  lv_label_set_text(label, "NO");

  /*Copy the button and set toggled state. (The release action is copied too)*/
  lv_obj_t * btn4 = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_size(btn4, btn_width, btn_height);
  lv_obj_align(btn4, NULL, LV_ALIGN_IN_RIGHT_MID, -10, 0);
  lv_obj_set_free_num(btn4, 4);               /*Set a unique number for the button*/
  lv_btn_set_action(btn4, LV_BTN_ACTION_CLICK, demo_click_action);

  /*Add a label to the toggled button*/
  label = lv_label_create(btn4, NULL);
  lv_label_set_text(label, "CAP");

  lv_obj_t * btn5 = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_size(btn5, btn_width, btn_height);
  lv_obj_align(btn5, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -15);
  lv_obj_set_free_num(btn5, 5);               /*Set a unique number for the button*/
  lv_btn_set_action(btn5, LV_BTN_ACTION_CLICK, demo_click_action);

  label = lv_label_create(btn5, NULL);
  lv_label_set_text(label, "DEBUG");

  lv_obj_t * btn6 = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_size(btn6, btn_width, btn_height);
  lv_obj_align(btn6, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -15);
  lv_obj_set_free_num(btn6, 6);               /*Set a unique number for the button*/
  lv_btn_set_action(btn6, LV_BTN_ACTION_CLICK, demo_click_action);

  label = lv_label_create(btn6, NULL);
  lv_label_set_text(label, "ODOM");
}
