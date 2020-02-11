#include "main.h"
#include "lcd.hpp"
// auton
// 1 = top blue
// 2 = bottom blue
// 3 = top red
// 4 = top blue
//5 = skills


int index_ = 0;
int selector_var = 0;//[5] = {1, 2, 3, 4, 5};

 int get_selector_var(){
    return selector_var;
  }



  void log_to_lcd(int line, const char* string, int argument){

      pros::lcd::clear_line(line);
      pros::lcd::print(line, string, argument);
  }


  void auton_index_left(){

    if(index_ >= 5){
      index_ = 5;
    }
    else{
      ++index_;
    }
    log_to_lcd(3, "Auton is %d\n", index_);
  }


  void auton_index_right(){

    if(index_ <= 0){
      index_ = 0;
    }
    else{
      --index_;
    }
    log_to_lcd(3, "Auton is %d\n", index_);
  }


  void auton_locker(){
    static bool pressed = false;
    pressed = !pressed;

      if(pressed){
        selector_var = index_;
      }
      else{
        selector_var = 0;
      }
    log_to_lcd(2, "Auton is locked in %d\n", selector_var);

    //auton_selector(selector_var);

}
