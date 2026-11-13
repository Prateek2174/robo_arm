// Parameter storage API.
// Module: #13   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef PARAMS_H
#define PARAMS_H

#include <stdbool.h>

typedef enum {
    PARAMS_OK,              // loaded newest valid copy
    PARAMS_OK_MIGRATED,     // loaded an older version and upgraded
    PARAMS_DEFAULT_EMPTY,   // both pages are erased (on first boot)
    PARAMS_DEFAULT_CORRUPT, // no copy passed magic/size/CRC
    PARAMS_DEFAULT_VERSION, // unknown version
    PARAMS_DEFAULT_RANGE,   // CRC fine but a value is out of range (NaN, pole_pairs 0, bad node ID)
} params_status_t

//bump up versoin whenever params_t changes so you can keep track of older versions
//an unknown version falls back to defaults
//check size to know if its a truncated record

//params_t (calibration, gains, limits, node_id) 

typedef struct {

} params_t;

params_status_t params_init();
const params_t *params_get(void); //a read only view for modules to see
bool params_set(const params_t *new_params); //range checks then updates RAM copy only
bool params_save(void); //writes to the oder page -> only in IDLE


#endif // PARAMS_H
