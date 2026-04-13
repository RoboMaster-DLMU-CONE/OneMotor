#include <one/utils/CCM.h>
#include <one/utils/DtcmAllocator.hpp>

k_heap g_dtcm_heap;
static OM_CCM_ATTR uint8_t g_dtcm_buffer[CONFIG_OM_DTCM_HEAP_SIZZE];


namespace one::motor::detail
{
    void dtcm_heap_init()
    {
        static bool initialized = false;
        if (!initialized)
        {
            k_heap_init(&g_dtcm_heap, &g_dtcm_buffer, CONFIG_OM_DTCM_HEAP_SIZZE);
            initialized = true;
        }
    }
}
