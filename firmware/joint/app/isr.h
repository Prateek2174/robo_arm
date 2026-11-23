#include <stdint.h>

typedef struct {
    uint32_t fast_cycles_last;
    uint32_t fast_cycles_max;
    uint32_t slow_cycles_last;
    uint32_t slow_cycles_max;
} isr_stats_t;

void isr_init(void); // DWT enable, NVIC priorities, debug pin -- enables nothing
void isr_start(void); // 
void isr_reset_stats(void);
void isr_get_stats(isr_stats_t *stats);