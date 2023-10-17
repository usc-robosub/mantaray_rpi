#pragma once

/**
 * @brief A finite state machine class.
 * 
 */
/**
 * @file fsm.h
 * @brief Defines the FSM class for implementing a finite state machine.
 */

#include "fsm_base.h"
#include "fsm_pcontrol.h"
#include "fsm_passive.h"

/**
 * @brief The FSM class for implementing a finite state machine.
 */
class FSM {
    public:
    /**
     * @brief Construct a new FSM object.
     * 
     */
    FSM();

    /**
     * @brief Destroy the FSM object.
     * 
     */
    ~FSM();

    /**
     * @brief Run the FSM for a given time step.
     * 
     * @param dt The time step in milliseconds.
     */
    void run(int dt);

    /**
     * @brief Set the current state of the FSM.
     * 
     * @param stateIndex The index of the new state to set.
     */
    void setState(int stateIndex);

    FSM_Base* getState();

    /**
     * @brief Get the index of the current state of the FSM.
     * 
     * @return int The index of the current state.
     */
    int getStateIndex();

    /**
     * @brief Get the index of the next state of the FSM.
     * 
     * @return int The index of the next state.
     */
    int getNextState();
    

    private:
    int stateIndex; /**< The index of the current state. */
    std::vector<std::unique_ptr<FSM_Base>> state_list; /**< The list of states. */

};
