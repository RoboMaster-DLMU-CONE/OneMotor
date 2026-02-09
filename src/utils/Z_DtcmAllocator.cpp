#include <one/utils/DtcmAllocator.hpp>

k_heap g_dtcm_heap;
#ifdef CONFIG_DT_HAS_ST_STM32_CCM_ENABLED
static __dtcm_bss_section uint8_t g_dtcm_buffer[CONFIG_OM_DTCM_HEAP_SIZZE];
#endif


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
