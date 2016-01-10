#ifndef STATEMACHINE_H
#define STATEMACHINE_H

//------------------------------------------------------------------------
//
//  Name:   StateMachine.h
//
//  Desc:   State machine class. Inherit from this class and create some 
//          states to give your agents FSM functionality
//
//  Author: Mat Buckland (fup@ai-junkie.com)
//
//------------------------------------------------------------------------

#include <cassert>
#include <string>
#include "State.h"
#include "Telegram.h"

template <class entity_type>
class StateMachine {
private:

	//a pointer to the agent that owns this instance
	entity_type*          _p_owner;

	State<entity_type>*   _p_current_state;

	//a record of the last state the agent was in
	State<entity_type>*   _p_previous_state;

	//this is called every time the FSM is updated
	State<entity_type>*   _p_global_state;


public:

	StateMachine(entity_type* owner) :_p_owner(owner),
		_p_current_state(NULL),
		_p_previous_state(NULL),
		_p_global_state(NULL)
	{}

	virtual ~StateMachine() {}

	//use these methods to initialize the FSM
	void SetCurrentState(State<entity_type>* s)		{ _p_current_state = s; }
	void SetGlobalState(State<entity_type>* s)		{ _p_global_state = s; }
	void SetPreviousState(State<entity_type>* s)	{ _p_previous_state = s; }

	//call this to update the FSM
	void  Update() const {
		//if a global state exists, call its execute method, else do nothing
		if (_p_global_state)   _p_global_state->Execute(_p_owner);

		//same for the current state
		if (_p_current_state) _p_current_state->Execute(_p_owner);
	}

	bool  HandleMessage(const Telegram& msg) const {
		//first see if the current state is valid and that it can handle
		//the message
		if (_p_current_state && _p_current_state->OnMessage(_p_owner, msg)) {
			return true;
		}

		//if not, and if a global state has been implemented, send 
		//the message to the global state
		if (_p_global_state && _p_global_state->OnMessage(_p_owner, msg)) {
			return true;
		}

		return false;
	}

	//change to a new state
	void  ChangeState(State<entity_type>* pNewState) {
		assert(pNewState && "<StateMachine::ChangeState>:trying to assign null state to current");

		//keep a record of the previous state
		_p_previous_state = _p_current_state;

		//call the exit method of the existing state
		_p_current_state->Exit(_p_owner);

		//change state to the new state
		_p_current_state = pNewState;

		//call the entry method of the new state
		_p_current_state->Enter(_p_owner);
	}

	//change state back to the previous state
	void  RevertToPreviousState() {
		ChangeState(_p_previous_state);
	}

	//returns true if the current state's type is equal to the type of the
	//class passed as a parameter. 
	bool  isInState(const State<entity_type>& st) const {
		if (typeid(*_p_current_state) == typeid(st)) return true;
		return false;
	}

	State<entity_type>*  CurrentState()  const { return _p_current_state; }
	State<entity_type>*  GlobalState()   const { return _p_global_state; }
	State<entity_type>*  PreviousState() const { return _p_previous_state; }

	//only ever used during debugging to grab the name of the current state
	std::string         GetNameOfCurrentState() const {
		std::string s(typeid(*_p_current_state).name());

		//remove the 'class ' part from the front of the string
		if (s.size() > 5) {
			s.erase(0, 6);
		}

		return s;
	}
};


#endif


