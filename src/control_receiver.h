//
// Created by alex on 4/24/19.
//

#ifndef SRC_CONTROL_RECEIVER_H
#define SRC_CONTROL_RECEIVER_H

class ControlReceiver
{
public:
    virtual ~ControlReceiver() = default;
    virtual void receiveControl(double rudder, double throttle) = 0;
    virtual void allDone() = 0;
};


#endif //SRC_CONTROL_RECEIVER_H
