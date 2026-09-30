#ifndef PIXY_H_
#define PIXY_H_

#include <stdint.h>
#include <stddef.h>
#include "fsl_common.h"
#include "fsl_lpi2c_edma.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    LPI2C_Type                   *instance;
    uint8_t                       address;
    lpi2c_master_edma_handle_t   *edmaHandle;
} pixy_t;


void pixy_init_bus(LPI2C_Type *inst,
                   lpi2c_master_edma_handle_t *sharedHandle,
                   edma_handle_t *rxHandle,
                   edma_handle_t *txHandle);

void pixy_init(pixy_t *cam,
               LPI2C_Type *inst,
               uint8_t addr,
               lpi2c_master_edma_handle_t *sharedHandle);


status_t pixy_set_led(pixy_t *cam, uint8_t r, uint8_t g, uint8_t b);


status_t pixy_get_vectors(pixy_t *cam,
                          uint16_t *vectors,
                          size_t max_vectors,
                          size_t *num_vectors);

#ifdef __cplusplus
}
#endif

#endif
