#include <iostream>
#include <math.h>
using namespace std;

void calc(float velocity, float omega)

{
    // locations of the wheels relative to the center
    float front_x = 0.27;   // distance forward to front wheels
    float rear_x = 0.26;    // distance back to rear wheels
    float center_y = 0.14;  // sideways distance to center wheels
    float front_y = 0.12;   // sideways distance to front wheels
    float rear_y = 0.12;    // sideways distance to rear wheels
    float wheel_radius = 0.065; // wheel radius
    


    // calculate turning radius for each wheel    
    float r0 = velocity / omega;    // turning radius to center of robot
    float r_lc = r0 - center_y;     // turning radius to left center wheel
    float r_rc = r0 + center_y;     // turning radius to right center wheel
    float r_lf = sqrt((r0-front_y)*(r0-front_y)+ front_x*front_x); // radius to left front 
    float r_lr = sqrt((r0-front_y)*(r0-front_y) + rear_x*rear_x); // radius to left rear
    float r_rf = sqrt((r0+front_y)*(r0+front_y) + front_x*front_x);
    float r_rr = sqrt ((r0+front_y)*(r0+front_y) + rear_x*rear_x);

    // calculate steering angle for each wheel
    float a_lf = atan2(front_x, r0-front_y);
    float a_rf = atan2(front_x, r0+front_y);
    float a_lr = atan2(-rear_x, r0-rear_y);
    float a_rr = atan2(-rear_x, r0+rear_y);
    
    // calculate angular velocity fr each wheel
    float omega_lf = velocity * r_lc/r0 / wheel_radius;
    float omega_lc = velocity * r_lc/r0/ wheel_radius;
    float omega_lr = velocity * r_lr/r0 / wheel_radius;
    float omega_rf = velocity * r_rf/r0 / wheel_radius;
    float omega_rc = velocity * r_rc/r0 / wheel_radius;
    float omega_rr = velocity * r_rr/r0 / wheel_radius;

    cout << "Vel="<<velocity<<" omega=" << omega <<endl;
    cout << "r0=" << r0 << " r_lf=" << r_lf << endl;
    cout << "velocities=" << endl;
    cout << "lf=" << omega_lf << " rf=" << omega_rf << endl;
    cout << "lc=" << omega_lc << " rc=" << omega_rc << endl;
    cout << "lr=" << omega_lr << " rr=" << omega_rr << endl;
    cout << "angles" << endl;
    cout << "lf=" << a_lf << " rf=" << a_rf << endl;
    cout << "lr=" << a_lr << " rr=" << a_rr << endl;
}

int main()
{
   //input command velocities
    calc (1.0, 0.5);
   return 0;
}


