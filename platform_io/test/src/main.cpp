#include "mbed.h"

DigitalOut myled(LED1);

int main() {
std::chrono::milliseconds dura( 20 );
    while(1) {
        myled = 1;
        ThisThread::sleep_for(dura);
        myled = 0;
        ThisThread::sleep_for(dura);
    }
}