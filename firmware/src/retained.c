#include "fw/retained.h"

#include <stddef.h>
#include <string.h>

#include "fw/hal.h"
#include "fw/params.h"

fw_retained_t *fw_retained_attach(bool *valid)
{
    size_t size = 0;
    void *mem = hal_retained_ram(&size);
    if (!mem || size < sizeof(fw_retained_t)) {
        if (valid) {
            *valid = false;
        }
        return NULL;
    }
    fw_retained_t *r = (fw_retained_t *)mem;
    bool ok = r->magic == FW_RETAINED_MAGIC &&
              r->crc == fw_crc32(r, offsetof(fw_retained_t, crc));
    if (valid) {
        *valid = ok;
    }
    return r;
}

void fw_retained_format(fw_retained_t *r)
{
    memset(r, 0, sizeof(*r));
    r->magic = FW_RETAINED_MAGIC;
    fw_retained_commit(r);
}

void fw_retained_commit(fw_retained_t *r)
{
    r->crc = fw_crc32(r, offsetof(fw_retained_t, crc));
}
