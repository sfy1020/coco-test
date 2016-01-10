#include "MessageDispatcher.h"
#include "BaseGameEntity.h"
#include "FrameCounter.h"
#include "EntityManager.h"

//uncomment below to send message info to the debug window
//#define SHOW_MESSAGING_INFO

//----------------------------- Dispatch ---------------------------------
//  
//  see description in header
//------------------------------------------------------------------------
void MessageDispatcher::Discharge(std::shared_ptr<BaseGameEntity> p_receiver, const Telegram& telegram)
{
	if (!p_receiver->handleMessage(telegram))
	{
		//telegram could not be handled
#ifdef SHOW_MESSAGING_INFO
		debug_con << "Message not handled" << "";
#endif
	}
}

//---------------------------- DispatchMsg ---------------------------
//
//  given a message, a receiver, a sender and any time delay, this function
//  routes the message to the correct agent (if no delay) or stores
//  in the message queue to be dispatched at the correct time
//------------------------------------------------------------------------
void MessageDispatcher::DispatchMsg(
	double			delay,
	unsigned int	sender,
	unsigned int	receiver,
	int				msg,
	void*			AdditionalInfo = NULL)
{

	//get a pointer to the receiver
	std::shared_ptr<BaseGameEntity> pReceiver = EntityMgr.getEntityFromID(receiver);

	//make sure the receiver is valid
	if (pReceiver == NULL)
	{
#ifdef SHOW_MESSAGING_INFO
		debug_con << "\nWarning! No Receiver with ID of " << receiver << " found" << "";
#endif

		return;
	}

	//create the telegram
	Telegram telegram(0, sender, receiver, msg, AdditionalInfo);

	//if there is no delay, route telegram immediately                       
	if (delay <= 0.0)
	{
#ifdef SHOW_MESSAGING_INFO
		debug_con << "\nTelegram dispatched at time: " << TickCounter->GetCurrentFrame()
			<< " by " << sender << " for " << receiver
			<< ". Msg is " << msg << "";
#endif

		//send the telegram to the recipient
		Discharge(pReceiver, telegram);
	}

	//else calculate the time when the telegram should be dispatched
	else
	{
		double CurrentTime = FrameCnt.getCurrentFrame();

		telegram.dispatch_time = CurrentTime + delay;

		//and put it in the queue
		_pque.insert(telegram);

#ifdef SHOW_MESSAGING_INFO
		debug_con << "\nDelayed telegram from " << sender << " recorded at time "
			<< TickCounter->GetCurrentFrame() << " for " << receiver
			<< ". Msg is " << msg << "";
#endif
	}
}

//---------------------- DispatchDelayedMessages -------------------------
//
//  This function dispatches any telegrams with a timestamp that has
//  expired. Any dispatched telegrams are removed from the queue
//------------------------------------------------------------------------
void MessageDispatcher::DispatchDelayedMessages()
{
	//first get current time
	double CurrentTime = FrameCnt.getCurrentFrame();

	//now peek at the queue to see if any telegrams need dispatching.
	//remove all telegrams from the front of the queue that have gone
	//past their sell by date
	while (!_pque.empty() &&
		(_pque.begin()->dispatch_time < CurrentTime) &&
		(_pque.begin()->dispatch_time > 0))
	{
		//read the telegram from the front of the queue
		const Telegram& telegram = *_pque.begin();

		//find the recipient
		std::shared_ptr<BaseGameEntity> pReceiver = EntityMgr.getEntityFromID(telegram.receiver);

#ifdef SHOW_MESSAGING_INFO
		debug_con << "\nQueued telegram ready for dispatch: Sent to "
			<< pReceiver->ID() << ". Msg is " << telegram.Msg << "";
#endif

		//send the telegram to the recipient
		Discharge(pReceiver, telegram);

		//remove it from the queue
		_pque.erase(_pque.begin());
	}
}



