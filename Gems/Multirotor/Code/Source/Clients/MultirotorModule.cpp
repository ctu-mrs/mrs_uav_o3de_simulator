
#include <Multirotor/MultirotorTypeIds.h>
#include <MultirotorModuleInterface.h>
#include "MultirotorSystemComponent.h"

namespace Multirotor
{
    class MultirotorModule
        : public MultirotorModuleInterface
    {
    public:
        AZ_RTTI(MultirotorModule, MultirotorModuleTypeId, MultirotorModuleInterface);
        AZ_CLASS_ALLOCATOR(MultirotorModule, AZ::SystemAllocator);
    };
}// namespace Multirotor

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), Multirotor::MultirotorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_Multirotor, Multirotor::MultirotorModule)
#endif
