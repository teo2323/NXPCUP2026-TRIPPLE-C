#include "pixy.h"
#include "peripherals.h"
#include "fsl_lpi2c_edma.h"
#include "fsl_common.h"
#include <stdbool.h>

extern volatile uint32_t g_systick_ms;

#define PIXY_I2C_TIMEOUT_MS 50U

static volatile bool     transferDone;
static volatile status_t s_transferStatus;

static void pixy_edma_cb(LPI2C_Type *base,
                         lpi2c_master_edma_handle_t *handle,
                         status_t status,
                         void *userData)
{
    (void)base; (void)handle; (void)userData;
    s_transferStatus = status;
    transferDone     = true;
}

void pixy_init(pixy_t *cam,
               LPI2C_Type *inst,
               uint8_t addr,
               edma_handle_t *rxHandle,
               edma_handle_t *txHandle)
{
    cam->instance = inst;
    cam->address  = addr;
    LPI2C_MasterCreateEDMAHandle(
        cam->instance,
        &cam->edmaHandle,
        rxHandle,
        txHandle,
        pixy_edma_cb,
        NULL);
}

static status_t pixy_wait_transfer(pixy_t *cam)
{
    uint32_t start_ms = g_systick_ms;
    while (!transferDone)
    {
        if ((g_systick_ms - start_ms) >= PIXY_I2C_TIMEOUT_MS)
        {
            LPI2C_MasterTransferAbortEDMA(cam->instance, &cam->edmaHandle);
            transferDone = true;
            return kStatus_Timeout;
        }
    }
    return s_transferStatus;
}

static status_t pixy_send(pixy_t *cam, const uint8_t *cmd, size_t len)
{
    transferDone     = false;
    s_transferStatus = kStatus_Success;
    lpi2c_master_transfer_t xfer = {
        .slaveAddress   = cam->address,
        .direction      = kLPI2C_Write,
        .subaddressSize = 0,
        .data           = (uint8_t *)cmd,
        .dataSize       = len,
        .flags          = kLPI2C_TransferDefaultFlag,
    };
    status_t s = LPI2C_MasterTransferEDMA(cam->instance, &cam->edmaHandle, &xfer);
    if (s != kStatus_Success) return s;
    return pixy_wait_transfer(cam);
}

static status_t pixy_recv(pixy_t *cam, uint8_t *buf, size_t len)
{
    transferDone     = false;
    s_transferStatus = kStatus_Success;
    lpi2c_master_transfer_t xfer = {
        .slaveAddress   = cam->address,
        .direction      = kLPI2C_Read,
        .subaddressSize = 0,
        .data           = buf,
        .dataSize       = len,
        .flags          = kLPI2C_TransferDefaultFlag,
    };
    status_t s = LPI2C_MasterTransferEDMA(cam->instance, &cam->edmaHandle, &xfer);
    if (s != kStatus_Success) return s;
    return pixy_wait_transfer(cam);
}

status_t pixy_set_led(pixy_t *cam, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t cmd[7] = {0xAE, 0xC1, 20, 3, r, g, b};
    status_t s = pixy_send(cam, cmd, sizeof(cmd));
    if (s != kStatus_Success) return s;
    SDK_DelayAtLeastUs(250U, SystemCoreClock);
    uint8_t resp[10];
    return pixy_recv(cam, resp, sizeof(resp));
}

status_t pixy_get_vectors(pixy_t *cam,
                          uint16_t *vectors,
                          size_t max_vectors,
                          size_t *num_vectors)
{
    if (cam == NULL || vectors == NULL || num_vectors == NULL) {
        return kStatus_InvalidArgument;
    }
    *num_vectors = 0;

    uint8_t cmd[6] = {0xAE, 0xC1, 48, 2, 1, 1};
    status_t s = pixy_send(cam, cmd, sizeof(cmd));
    if (s != kStatus_Success) return s;

    /* Turnaround delay: give Pixy2 processor time to prepare the response */
    SDK_DelayAtLeastUs(250U, SystemCoreClock);

    uint8_t buf[100];
    s = pixy_recv(cam, buf, sizeof(buf));
    if (s != kStatus_Success) return s;

    /* Validate Pixy2 response packet header: sync word 0xc1af (0xaf, 0xc1) */
    if (buf[0] != 0xAF || buf[1] != 0xC1) {
        return kStatus_Fail;
    }

    /* Packet length includes 6 bytes of header (sync[2], type[1], len[1], checksum[2]) + payload */
    uint16_t packetLen = (uint16_t)buf[3] + 6U;
    if (packetLen > sizeof(buf)) {
        packetLen = sizeof(buf);
    }

    size_t   count = 0;
    uint16_t idx   = 6;

    while ((idx + 2U) <= packetLen && count < max_vectors) {
        uint8_t featType = buf[idx];
        uint8_t featLen  = buf[idx + 1];
        if (featLen == 0 || (idx + 2U + featLen) > packetLen) {
            break;
        }

        /* Feature type 1: Line tracking vectors */
        if (featType == 1U) {
            uint8_t *data = &buf[idx + 2];
            uint8_t  nvec = featLen / 6U;

            for (uint8_t i = 0; i < nvec && count < max_vectors; i++) {
                size_t off = i * 6U;
                vectors[4 * count + 0] = data[off + 0];
                vectors[4 * count + 1] = data[off + 1];
                vectors[4 * count + 2] = data[off + 2];
                vectors[4 * count + 3] = data[off + 3];
                count++;
            }
        }
        idx += 2U + featLen;
    }

    *num_vectors = count;
    return kStatus_Success;
}
