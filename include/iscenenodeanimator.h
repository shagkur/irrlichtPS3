#ifndef __ISCENENODEANIMATOR_H__
#define __ISCENENODEANIMATOR_H__

#include "irefcounter.h"
#include "escenenodeanimatortypes.h"
#include "vector3d.h"
#include "ieventreceiver.h"

namespace irr
{
	namespace scene
	{
		class ISceneNode;
		class ISceneManager;

		class ISceneNodeAnimator : public virtual IRefCounter, public IEventReceiver
		{
		public:
			virtual void animateNode(ISceneNode *node,u32 timeMs) = 0;

			virtual bool isEventReceiverEnabled() const
			{
				return false;
			}

			virtual bool onEvent(const SEvent& event)
			{
				return false;
			}

			virtual bool hasFinished() const
			{
				return false;
			}

			virtual ESCENE_NODE_ANIMATOR_TYPE getType() const
			{
				return ESNAT_UNKNOWN;
			}
		};
	}
}

#endif
