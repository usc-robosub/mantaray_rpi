#pragma once

#include "include.h"

class FSM_Passive : public FSM_Base
{
    public:
    FSM_Passive();
    ~FSM_Passive();
    void enter();
    void exit();
    void run(int dt);
    std::string get_name();
    private:
    std::string name = "Passive";
    
};