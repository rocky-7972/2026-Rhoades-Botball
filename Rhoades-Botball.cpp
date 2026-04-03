#include <kipr/wombat.h>
#include <iostream>
#include <thread>
#include <cmath>
using namespace std::literals::chrono_literals;
const unsigned short ticks_per_degree=6900/90;
double bias;
int onval, offval;
enum dir {r = 0, l = 3};
constexpr unsigned short thresh[] = {500,500,1500};

void servo_move(double _final, double time, double port=0) {	
    ao();
    enable_servo(port);

    int start_pos = get_servo_position(port);
    int distance = _final - start_pos; 

    long start_time = systime();


    while((systime() - start_time) < time) {

        double progress = (double)(systime() - start_time) / time;


        int current_target = start_pos + (int)(distance * progress);

        set_servo_position(port, current_target);
        msleep(10); 
    }
    set_servo_position(port, _final);
    printf("I");
    disable_servo(port);
}
void integral(int speed = 100) {
    ao();

    // The threshold for the "virtual line" (the wall distance)
    int wall_threshold = 1150; 

    // Loop runs as long as BOTH digital sensors 0 and 1 are NOT pressed (0)
    while(digital(0) < 1 && digital(1) < 1) {

        if (analog(4) > wall_threshold) {
            // Too close to the wall (Value high) -> Veer AWAY (Left)
            // Slow down the Left motor, keep Right motor at speed
            motor(3, speed - 30);
            motor(0, speed);
        } 
        else {
            // Too far from the wall (Value low) -> Veer TOWARD (Right)
            // Keep Left motor at speed, slow down the Right motor
            motor(3, speed);
            motor(0, speed - 30);
        }

        msleep(5);
    }

    // Stop once a digital sensor is hit
    ao();
}
void antegral(int speed = 100, int dif=40) {
    ao();

    // The threshold for the "virtual line" (the wall distance)
    int wall_threshold = 1150; 

    // Loop runs as long as BOTH digital sensors 0 and 1 are NOT pressed (0)
    while(analog(0) < thresh[0] && analog(1) < thresh[1]) {

        if (analog(4) > wall_threshold) {
            // Too close to the wall (Value high) -> Veer AWAY (Left)
            // Slow down the Left motor, keep Right motor at speed
            motor(3, speed - dif);
            motor(0, speed);
        } 
        else {
            // Too far from the wall (Value low) -> Veer TOWARD (Right)
            // Keep Left motor at speed, slow down the Right motor
            motor(3, speed);
            motor(0, speed - dif);
        }

        msleep(5);
    }
    while(analog(0) > thresh[0] || analog(1) > thresh[1]) {
        if (analog(4) > wall_threshold) {
            // Too close to the wall (Value high) -> Veer AWAY (Left)
            // Slow down the Left motor, keep Right motor at speed
            motor(3, speed - dif);
            motor(0, speed);
        } 
        else {
            // Too far from the wall (Value low) -> Veer TOWARD (Right)
            // Keep Left motor at speed, slow down the Right motor
            motor(3, speed);
            motor(0, speed - dif);
        }

        msleep(5);  
    }

    // Stop once a digital sensor is hit
    ao();
}

void local(int speed = 50) {
    ao();


    while(analog(0) < thresh[0] || analog(1) < thresh[1]) {
        if(analog(0) >= thresh[0] && analog(1) < thresh[1]) {

            motor(0, 0);
            motor(3, speed-20); 
        }
        else if(analog(0) < thresh[0] && analog(1) >= thresh[1]) {

            motor(0, speed-20);
            motor(3, 0);
        } 
        else {
            motor(0, speed);
            motor(3, speed);
        }
        msleep(5);
    }
    while(analog(0) >= thresh[0] || analog(1) >= thresh[1]) {
        if(analog(0) < thresh[0] && analog(1) >= thresh[1]) {
            motor(0, 0);
            motor(3, speed-30);
        }
        else if(analog(0) >= thresh[0] && analog(1) < thresh[1]) {
            motor(0, speed-30);
            motor(3, 0);
        }
        else {
            motor(0, speed);
            motor(3, speed);
        }
        msleep(5);
    }


}
void d_local(int speed = 50) {
    ao(); 

    while(digital(0) == 0 || digital(1) == 0) {

        if(digital(0) == 1 && digital(1) == 0) {
            motor(0, 0);
            motor(3, speed);
        }
        else if(digital(0) == 0 && digital(1) == 1) {

            motor(0, speed);
            motor(3, 0);
        } 
        else {

            motor(0, speed);
            motor(3, speed);
        }


    }


}

int L_PORT = 0;
int R_PORT = 3;

enum class Color{
    BLUE,
    PINK
};
Color color;

double get_gyro() {
    return (double)gyro_y(); 
}
void h_local(int speed = 50) {
    ao();
    while(analog(0) < thresh[0] || analog(1) < thresh[1]) {
        if(analog(0) >= thresh[0] && analog(1) < thresh[1]) {

            motor(0, 0);
            motor(3, speed); 
        }
        else if(analog(0) < thresh[0] && analog(1) >= thresh[1]) {

            motor(0, speed);
            motor(3, 0);
        } 
        else {
            motor(0, speed);
            motor(3, speed);
        }
        msleep(5);
    }
}
double calibrate_gyro() {
    int i;
    double total = 0.0;
    printf("Calibrating... DO NOT MOVE\n");
    msleep(500); 
    for (i = 0; i < 1000; i++) {
        total += get_gyro(); 
        msleep(1);
    }
    double bias = total / 1000.0;
    printf("Bias: %f\n", bias);
    return bias;
}

void pivot(int speed, double degrees, int wheel_port,  double bias=bias) {
    double dev = 0;
    double target_deviation = degrees * (59000.0 / 90.0);
    int dir = (degrees > 0) ? 1 : -1;


    if (wheel_port == L_PORT) {
        mav(L_PORT, dir * speed);
    } else if (wheel_port == R_PORT) {
        mav(R_PORT, -1 * dir * speed);
    }

    while (fabs(dev) < fabs(target_deviation)) {
        dev += get_gyro() - bias; 
        msleep(10);
    }
    ao();
}


void spin(int speed, double degrees, double bias=bias) {
    double dev = 0;
    double target_deviation = degrees * (55000.0 / 90.0);
    int dir = (degrees > 0) ? 1 : -1;


    mav(R_PORT, 1 * dir * speed);
    mav(L_PORT, -1 * dir * speed);

    while (fabs(dev) < fabs(target_deviation)) {
        dev += get_gyro() - bias; 
        msleep(10);
    }
    ao();
}
void backward(int time, int speed=75){
    ao();
    cmpc(0);
    cmpc(3);
    while(((gmpc(0)+gmpc(3))/2)>-time){
        motor(0,-speed);
        motor(3,(-speed+2.1));
    }
    ao();
}
void lift(int speed=50){
    while(true){
        if(analog(3)>1600){
            motor(2, -speed);
        }
        else break;
        //msleep(10);
    }

    freeze(2);
}
void down(int speed=50){
    cmpc(2);
    while(gmpc(2)<4000){
        motor(2,speed);
    }
}
void forward(int time, int speed=75){
    cmpc(0);
    cmpc(3);
    while(((gmpc(0)+gmpc(3))/2)<time){
        motor(0,speed);
        motor(3,(speed-2.1));
    }
    ao();
}
void open(const dir side){
    cmpc(1);
    if(side==dir::r){
        while(gmpc(1)<479){
            motor(1,50);
        }
    }
    freeze(1);
    cmpc(1);
    if(side==dir::l){
        while(gmpc(1)>-290){
            motor(1,-50);
        }
    }
    freeze(1);
}
void sort(){
    for(int i=0; i<7; i++){
        servo_move(825, 500, 2);
        msleep(200);
        int x=analog(5);
        while(true){
            if(!(analog(5)<(x+700) && analog(5)>(x-700))){
                std::cout<<"sensed drum \n";
                servo_move(1380, 300, 2);
                msleep(400);
                if(analog(5)>700 && analog(5)<2200){
                    color=Color::BLUE;
                    std::cout<<"blue\n";
                }
                else if(analog(5)>2200 && analog(5)<3700){
                    color=Color::PINK; 
                    std::cout<<"pink\n";
                }

                //servo_move(1207, 500, 2);
                if(color==Color::BLUE){
                    std::thread t5(servo_move, 813, 1150, 0);
                    std::this_thread::sleep_for(500ms);
                    std::thread t6(servo_move, 1630, 1700, 1);
                    t5.join();
                    t6.join();
                    servo_move(825, 500, 2);
                    msleep(200);

                    std::thread t3(servo_move, 10, 1500, 1);
                    msleep(1000);
                    std::thread t4(servo_move, 1920, 1200, 0);
                    t3.join();
                    t4.join();

                }
                if(color==Color::PINK){
                    std::thread t7(servo_move, 750, 1140, 0);
                    std::this_thread::sleep_for(400ms);
                    std::thread t8(servo_move, 1860, 2050, 1);
                    t7.join();
                    t8.join();
                    servo_move(825, 500, 2);
                    msleep(200);

                    std::thread t9(servo_move, 10, 1640, 1);
                    msleep(1200);
                    std::thread t10(servo_move, 1920, 1200, 0);
                    t9.join();
                    t10.join();
                }
                break;
            }
        }
    }
    int y=analog(5);
    while(true){
        if(!(analog(5)<(y+700) && analog(5)>(y-700))){
            std::cout<<"sensed drum \n";
            servo_move(1380, 500, 2);
            msleep(500);
            if(analog(5)>700 && analog(5)<2200){
                color=Color::BLUE;
                std::cout<<"blue\n";
            }
            else if(analog(5)>2200 && analog(5)<3700){
                color=Color::PINK; 
                std::cout<<"pink\n";
            }

            break;
        }
    }
    if(color==Color::BLUE){
        std::thread t5(servo_move, 813, 1150, 0);
        std::this_thread::sleep_for(500ms);
        std::thread t6(servo_move, 1630, 1700, 1);
        t5.join();
        t6.join();
        servo_move(825, 500, 2);

    }
    if(color==Color::PINK){
        std::thread t7(servo_move, 740, 1140, 0);
        std::this_thread::sleep_for(500ms);
        std::thread t8(servo_move, 1860, 2050, 1);
        t7.join();
        t8.join();
        servo_move(825, 500, 2);
    }

}

void line(const dir side, int time, int port, int speed=100, int dif=13){
    cmpc(0);
    cmpc(3);
    if(side==dir::r){
        while(abs(((gmpc(0)+gmpc(3))/2))<abs(time)){
            if(analog(port)<thresh[port]){
                motor(0,-speed);
                motor(3,(-speed+dif));
            }
            if(analog(port)>thresh[port]){
                motor(3,-speed);
                motor(0,(-speed+dif));
            }
        }
    }
    if(side==dir::l){
        while(abs(((gmpc(0)+gmpc(3))/2))>abs(time)){
            if(analog(port)>thresh[port]){
                motor(0,-speed);
                motor(3,(-speed+dif));
            }
            if(analog(port)<thresh[port]){
                motor(3,-speed);
                motor(0,(-speed+dif));
            }
        }
    }
}
void wline(const dir side, int port, int speed=100, int dif=13){
    cmpc(0);
    cmpc(3);
    if(side==dir::r){
        while(digital(2)<1){
            if(analog(port)<thresh[port]){
                motor(0,-speed);
                motor(3,(-speed+dif));
            }
            if(analog(port)>thresh[port]){
                motor(3,-speed);
                motor(0,(-speed+dif));
            }

        }
    }
    if(side==dir::l){
        while(digital(2)<1){
            if(analog(port)>thresh[port]){
                motor(0,-speed);
                motor(3,(-speed+dif));
            }
            if(analog(port)<thresh[port]){
                motor(3,-speed);
                motor(0,(-speed+dif));
            }

        }
    }
}
int main()
{
    down();
    bias=calibrate_gyro();
    servo_move(1026, 1000, 1);
    servo_move(1846,1000,0);
    /*while(digital(9)<1){
        msleep(1);
    }*/
    
    std::thread t_local(local, 100);
    std::thread t_servo(servo_move, 1501, 500, 0);
    t_local.join();
    t_servo.join();
	spin(1000, 10);

    std::thread t2(servo_move, 1, 900, 1);
    std::thread t1(local, 100); //just goes forward
    t1.join();
    t2.join();
    ao();
    while(analog(0) < thresh[0] || analog(1) < thresh[1]) {
        if(analog(0) >= thresh[0] && analog(1) < thresh[1]) {

            motor(0, 0);
            motor(3, -50); 
        }
        else if(analog(0) < thresh[0] && analog(1) >= thresh[1]) {

            motor(0, -50);
            motor(3, 0);
        } 
        else {
            motor(0, -50);
            motor(3, -50);
        }
        msleep(5);
    }

    backward(300,50);
	ao();    
    pivot (2000, 95, r, bias);

    d_local(100);
    backward(380, 50);
	ao();
    //servo_move(1, 1400, 1);
    ao();
    //spin(500, 0.5, bias);
    servo_move(1920, 600, 0);

    sort();
    
    
    servo_move (200, 1200, 1);
    backward(900);
    pivot(3000, 90, r);
    lift(100);
    
    antegral(80,40);
    antegral(80,35);
	integral(90);
    
    d_local(100);
    ao();
    
    pivot(1000, -39, r);
    ao();
    pivot(1000, -36, l);
    open(l);
    msleep(200);
    
    pivot(4000, 30, l);
    while(analog(0)<thresh[0]){
        motor(0,-100);
        motor(3,-100);
    }
    backward(1500, 100);
    ao();
    local(-100);
    while(analog(0) < thresh[0] || analog(1) < thresh[1]) {
        if(analog(0) >= thresh[0] && analog(1) < thresh[1]) {

            motor(0, 0);
            motor(3, 80); 
        }
        else if(analog(0) < thresh[0] && analog(1) >= thresh[1]) {

            motor(0, 80);
            motor(3, 0);
        } 
        else {
            motor(0, 80);
            motor(3, 80);
        }
        msleep(5);
    }
    spin(3000, 90);

    wline(l, 0, 100, 53);
    ao();
    lift(75);

    ao();

    pivot(1000, 47, r);
    pivot(1000, 69,l);
    msleep(200);
    open(r);  //drops on first post
    msleep(500);

    pivot(3000, -45,l);
    spin(1000, 55);

    ao();
    servo_move(1325,1000,0);
    ao();

    while(digital(0)<1){
        if(analog(2)>thresh[2]){
            motor(0,100);
            motor(3,(100-23));
        }
        if(analog(2)<thresh[2]){
            motor(3,100);
            motor(0,(100-23));
        }
    }
    spin(1000, -108);
    integral();
    d_local(100);
    backward(150,50);
    spin(1000, 100);
    d_local(40);

    backward(1500, 50);
    pivot(500, 12, l);
    forward(55, 20);

    msleep(100);
    open(l); // drop on second post
    msleep(200);
    return 0;
}

