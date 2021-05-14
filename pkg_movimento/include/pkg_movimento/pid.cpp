// PID regulator

#include<iostream>
#include<fstream>

using namespace std;

//PID class
class PID
{
private:

    double kp;
    double ki;
    double kd;

    double set_point = 0;
    bool antiwindup = false;
    double windupMax;

    double iTerm=0;

    double dt = 0.05;
    double now_time=0;
    double old_time=0;

    double outValue=0;
    double last_error = 0;
    double last_y = 0;

public:

    // Constructor
    PID(double kp_,double ki_,double kd_,double windupMax_=0)
    {
        kp = kp_;
        ki = ki_;
        kd = kd_;

        windupMax = windupMax_;
        if(windupMax_ != 0)
        {
            antiwindup = true;
        }else
        {
            antiwindup = false;
        }
    }

    // PID output according to y_measured
    double output(double y_measured)
    {
        double error = set_point - y_measured;

        //Get elapsed time

        iTerm = iTerm + ki * error * dt;
        iTerm = antiWindup(iTerm);
        std::cout << "P:" << kp * error << " I: " << iTerm << "\n";
        double out = kp * error + iTerm + kd * (last_y - y_measured)/dt;

        last_error = error;
        last_y = y_measured;
        if (out > 2)
        {
            out = 2.0 ;
        }
        if (out < -2)
        {
            out = -2.0 ;
        }
        return out;
    }

    // Set the setpoint
    void set_set_point(double set_point_)
    {
        set_point = set_point_;
    }

    // Get the setpoint (print it)
    void get_set_point()
    {
        cout << set_point << endl;
    }

    // Print the parameters
    void printParameters()
    {
        cout << "kp" << endl;
        cout << kp << endl;
        cout << "ki" << endl;
        cout << ki << endl;
        cout << "kd" << endl;
        cout << kd << endl;
        cout << "Windup limit" << endl;
        cout << windupMax << endl;
        cout << "Bool has windup" << endl;
        cout << antiwindup << endl;
        cout << "Set point" << endl;
        cout << set_point << endl;
    }

    void reset()
    {
        iTerm = 0.0;
    }

    // Anti windup system
    double antiWindup(double u)
    {
        if(!antiwindup)
        {
            return u;
        }else if (u >= windupMax)
        {
            return windupMax;
        }else if (u <= -windupMax)
        {
            return -windupMax;
        }else
        {
            return u;
        }

    }
};

// Save an array of data to a .csv file
void saveArray(double arrayToSave[])
{
    ofstream arrayData("C:\\users\\file1.csv");
    for(int k=0;k < 200; k++)
    {
        arrayData << arrayToSave[k] << "," << endl;
    }
    cout << "File saved" << endl;
}

