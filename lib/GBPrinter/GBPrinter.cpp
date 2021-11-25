/*
 * This file is part of the esp32-phomemo-gameboy-printer project.
 * Copyright (C) 2018  Tido Klaassen <tido_gbprinter@4gh.eu>
 * Copyright (C) 2021  Jack Gaino <github@jackgaino.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include "ETH.h"
#include "GBPrinter.h"
#include "Arduino.h"

#define GPIO_MISO       19
#define GPIO_MOSI       23
#define GPIO_SCLK       18

#define mb()            asm volatile ("" : : : "memory")
#define ARRAY_SIZE(x)   (sizeof(x) / sizeof(*(x)))

#define TAG "GB-Printer"

#define PKT_RING_SIZE   4

#define STATUS_CHKSUM   (1u << 0)
#define STATUS_BUSY     (1u << 1)
#define STATUS_FULL     (1u << 2)
#define STATUS_UNPROC   (1u << 3)
#define STATUS_ERR0     (1u << 4)
#define STATUS_ERR1     (1u << 5)
#define STATUS_ERR2     (1u << 6)
#define STATUS_BATLOW   (1u << 7)

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

enum pr_state {
    state_sync0 = 0,
    state_sync1,
    state_cmd,
    state_compr,
    state_len_low,
    state_len_high,
    state_data,
    state_chk_low,
    state_chk_high,
    state_ack,
    state_status,
};

enum pr_cmd {
    cmd_init = 0x01,
    cmd_print = 0x02,
    cmd_data = 0x04,
    cmd_break = 0x08,
    cmd_inquiry = 0x0f,
};

enum pr_owner {
    own_isr = 0,
    own_task,
};

struct pr_packet {
    volatile enum pr_owner owner;
    enum pr_state state;
    uint8_t cmd;
    uint8_t compr;
    uint8_t data[MAX_DATA_SIZE];
    size_t data_len;
    size_t cur_len;
    uint16_t cur_sum;
    uint16_t chk_sum;
};

static uint8_t render_buff[IMG_SIZE];
char logging_buffer[100];
static QueueHandle_t img_queue;

static struct spi_ctrl {
    struct pr_packet work_packet;
    struct pr_packet packets[PKT_RING_SIZE];
    size_t clk_cnt;
    uint64_t clk_last;
    uint8_t rcv_data;
    uint8_t snd_data;
    volatile size_t wr_idx;
    volatile size_t rd_idx;
    volatile uint8_t status_isr;
    volatile uint8_t status_task;
    QueueHandle_t packet_done;
} ctrl;

void initialize_printer(void *pvParameters) {
    img_queue = (QueueHandle_t) pvParameters;
    pinMode(GPIO_MISO, OUTPUT);
    pinMode(GPIO_MOSI, INPUT);
    pinMode(GPIO_SCLK, INPUT);

    memset(&ctrl, 0x0, sizeof(spi_ctrl));
    ctrl.packet_done = xSemaphoreCreateBinary();
    if (ctrl.packet_done == NULL) {
        Serial.println("xSemaphoreCreateBinary() failed");
    } else {
        Serial.println("xSemaphoreCreateBinary() succeeded");
    }

    Serial.println("Attaching interrupt");
    attachInterruptArg(GPIO_SCLK, sclk_isr_handler, &ctrl, CHANGE);

    Serial.println("Creating task");
    xTaskCreatePinnedToCore(
            &packet_proto_task,
            "packet_proto_task",
            3072,
            NULL,
            2,
            NULL, 1);

    vTaskDelete(NULL);
}

void IRAM_ATTR sclk_isr_handler(void *arg) {
    struct spi_ctrl *localctrl;
    struct pr_packet *packet;
    uint64_t curr_time;
    BaseType_t task_woken;

//    Serial.println("Received packet");

    localctrl = (struct spi_ctrl *) arg;
    packet = &(localctrl->work_packet);

    if (digitalRead(GPIO_SCLK) == 0) {
        /* Falling edge. Check timeouts and set up MISO. */

        curr_time = esp_timer_get_time();

        /* Check for sync loss during byte transfer. Clock should run  */
        /* at ~8kHz (125us), so anything above 200us is definitely out */
        /* of spec.                                                    */
        if (unlikely(localctrl->clk_cnt != 0 && (curr_time - localctrl->clk_last) > 200)) {
            localctrl->clk_cnt = 0;
            localctrl->rcv_data = 0;
            localctrl->snd_data = 0;
            localctrl->status_isr |= STATUS_ERR0;
            packet->state = state_sync0;
        }

        /* According to the Game Boy programming manual the maximum allowed */
        /* pause between sending bytes in a packet is 5ms. */
        if (unlikely(packet->state != state_sync0
                     && (curr_time - localctrl->clk_last) > 5100)) {
            localctrl->status_isr |= STATUS_ERR0;
            packet->state = state_sync0;
        }

        localctrl->clk_last = curr_time;
//        Serial.println(localctrl->snd_data);
        digitalWrite(GPIO_MISO, (localctrl->snd_data & 0x80u) ? 1 : 0);
//        digitalWrite(GPIO_MISO, 1);
        localctrl->snd_data <<= 1;
    } else {
        // Rising edge. Sample MOSI and increase clock count.
        if (unlikely(localctrl->clk_cnt == 0)) {
            localctrl->rcv_data = 0;
        }

        localctrl->rcv_data <<= 1;
        if (digitalRead(GPIO_MOSI)) {
            localctrl->rcv_data |= 1;
        }

        ++localctrl->clk_cnt;
    }

    if (likely(localctrl->clk_cnt <= 7)) {
        goto done;
    }

    localctrl->clk_cnt = 0;

    if (packet->state >= state_cmd && packet->state < state_chk_low) {
        packet->cur_sum += localctrl->rcv_data;
    }

    /* Process the transferred byte. The state indicates how the byte that */
    /* has just completed is interpreted. If data needs to be sent out in  */
    /* state x, it must be copied to localctrl->snd_data during transition to   */
    /* that state.                                                         */
    switch (packet->state) {
        case state_sync0:
            if (localctrl->rcv_data == 0x88) {
                packet->cur_sum = 0;
                packet->data_len = 0;
                localctrl->status_isr &= ~(STATUS_CHKSUM | STATUS_BUSY);

                packet->state = state_sync1;
            }
            break;
        case state_sync1:
            if (localctrl->rcv_data == 0x33) {
                packet->state = state_cmd;
            } else {
                packet->state = state_sync0;
                localctrl->status_isr |= STATUS_ERR0;
            }
            break;
        case state_cmd:
            packet->cmd = localctrl->rcv_data;
            packet->state = state_compr;
            break;
        case state_compr:
            packet->compr = localctrl->rcv_data;
            packet->state = state_len_low;
            break;
        case state_len_low:
            packet->cur_len = localctrl->rcv_data;
            packet->state = state_len_high;
            break;
        case state_len_high:
            packet->cur_len |= (localctrl->rcv_data << 8);

            if (likely(packet->cur_len <= sizeof(packet->data))) {
                if (packet->cur_len > 0) {
                    packet->state = state_data;
                } else {
                    packet->state = state_chk_low;
                }
            } else {
                /* Bogus data length. We could try inferring the real length
                 * from the command byte, but since we probably have lost
                 * synchronisation anyway, we just give up on this packet */
                packet->state = state_sync0;
                localctrl->status_isr |= STATUS_ERR0;
            }
            break;
        case state_data:
            packet->data[packet->data_len++] = localctrl->rcv_data;
            --packet->cur_len;

            if (unlikely(packet->cur_len == 0)) {
                packet->state = state_chk_low;
            }
            break;
        case state_chk_low:
            packet->chk_sum = localctrl->rcv_data;
            packet->state = state_chk_high;
            break;
        case state_chk_high:
            packet->chk_sum |= (localctrl->rcv_data << 8);

            /* We need to send the ACK byte next. */
            localctrl->snd_data = 0x81;

            packet->state = state_ack;
            break;
        case state_ack:
            /* Packet is almost complete, we need to send out the checksum */
            /* in the next byte. Check for errors and update ISR status    */
            /* accordingly. If everything checks out, try copying packet   */
            /* to transfer ring.                                           */

            if (unlikely(packet->chk_sum != packet->cur_sum)) {
                localctrl->status_isr |= (STATUS_CHKSUM | STATUS_ERR0);
            } else {
                switch (packet->cmd) {
                    case cmd_data:
                        /* We expect to always receive complete bands (20 tiles) */
                        /* of image data.                                        */
                        if (packet->data_len % 40 == 0) {
                            localctrl->status_isr |= STATUS_UNPROC;
                        } else {
                            localctrl->status_isr |= STATUS_ERR0;
                        }
                        break;
                    case cmd_print:
                        if (packet->data_len == PR_PARAM_SIZE) {
                            localctrl->status_isr &= ~STATUS_UNPROC;
                            localctrl->status_isr |= STATUS_FULL;
                        } else {
                            localctrl->status_isr |= STATUS_ERR0;
                        }
                        break;
                    case cmd_inquiry:
                        break;
                    case cmd_init:
                    case cmd_break:
                        localctrl->status_isr = 0;
                        break;
                }

                if (localctrl->packets[localctrl->wr_idx].owner == own_isr) {
                    memcpy(&(localctrl->packets[localctrl->wr_idx]), packet, sizeof(*packet));
                } else {
                    /* Protocol task is too slow, signal busy error. */
                    localctrl->status_isr |= (STATUS_ERR0 | STATUS_BUSY);
                }
            }

            /* Send combined error status of ISR and task */
            localctrl->snd_data = (localctrl->status_isr | localctrl->status_task);

            packet->state = state_status;
            break;
        case state_status:
            /* Status byte has been sent out, packet transfer is complete. */
            /* Hand packet over to protocol task, unless we had a checksum */
            /* error or packet ring was full.                              */
            if (likely((localctrl->status_isr & (STATUS_CHKSUM | STATUS_BUSY)) == 0)) {
                mb();
                localctrl->packets[localctrl->wr_idx].owner = own_task;
                mb();

                xSemaphoreGiveFromISR(localctrl->packet_done, &task_woken);
                if (task_woken) {
                    portYIELD_FROM_ISR();
                }

                ++localctrl->wr_idx;
                localctrl->wr_idx %= ARRAY_SIZE(localctrl->packets);
            }

            /* Get ready to start reception of next packet */
            packet->state = state_sync0;
            localctrl->snd_data = 0;
            break;
        default:
            /* Should never be reached. */
            packet->state = state_sync0;
            localctrl->status_isr |= STATUS_ERR0;
            break;
    }

    done:
    return;
}

[[noreturn]] void IRAM_ATTR packet_proto_task(void *pvParameters) {
    struct pr_packet *packet;
    struct pr_data *data = NULL;

    Serial.println("Packet proto task started.");

    while (true) {
        /* Make sure we have an image data buffer ready before we accept */
        /* any command packages from the ISR.                            */

        if (data == NULL) {
            data = static_cast<pr_data *>(calloc(1, sizeof(*data)));
            if (data == NULL) {
                Serial.println("Out of memory");
                ctrl.status_task = STATUS_BUSY;
                continue;
            }
        }

        /* If we have a completed image buffer, try handing it to the main */
        /* task for rendering and saving to flash.                         */
        if (data->printed != 0) {
            if (xQueueSend(img_queue, &data, 0) != pdTRUE) {
                ctrl.status_task |= STATUS_BUSY;
                ++data->busy_cnt;

                if (data->busy_cnt < 5) {
                    /* Give main task a chance to process the queue. */
                    vTaskDelay(portTICK_PERIOD_MS);
                } else {
                    /* Give up and signal an error after five tries. */
                    memset(data, 0x0, sizeof(*data));
                    ctrl.status_task |= STATUS_ERR0;
                }
            } else {
                /* Data buffer belongs to main task now. We will allocate */
                /* a new one on next loop iteration.                      */
                data = NULL;
                ctrl.status_task &= ~STATUS_BUSY;
            }

            /* Restart loop for either retrying this buffer or allocating */
            /* a new one.                                                 */
            continue;
        }

        /* We have a non-completed data buffer now. Remove busy signal and */
        /* wait for command packet from ISR.                               */
        ctrl.status_task &= ~STATUS_BUSY;

        packet = &(ctrl.packets[ctrl.rd_idx]);
        if (packet->owner == own_isr) {
            xSemaphoreTake(ctrl.packet_done, portMAX_DELAY);
        }

        /* No new packet availabe? Restart loop. */
        if (packet->owner == own_isr) {
            continue;
        }

        sprintf(logging_buffer, "Got packet: cmd: 0x%02x status_isr: 0x%02x status_task: 0x%02x",
                packet->cmd, ctrl.status_isr, ctrl.status_task);
        Serial.println(logging_buffer);

        switch (packet->cmd) {
            case cmd_data:
                /* We expect to always receive complete bands (20 tiles)  */
                /* of image data. Copy data payload into the image buffer */
                /* and update the current offset.                         */

                sprintf(logging_buffer, "cmd_data: len: 0x%x", packet->data_len);
                Serial.println(logging_buffer);
                if ((packet->data_len % 40 == 0)
                    && (data->data_len + packet->data_len <= sizeof(data->data))) {
                    memcpy(&(data->data[data->data_len]),
                           packet->data,
                           packet->data_len);

                    data->data_len += packet->data_len;
                    ctrl.status_task |= STATUS_UNPROC;
                } else {
                    sprintf(logging_buffer, "Invalid data buffer length: 0x%x",
                            packet->data_len);
                    Serial.println(logging_buffer);

                    ctrl.status_task |= STATUS_ERR0;
                }
                break;
            case cmd_print:
                /* Image data transfer is complete. Prepare to hand data buffer */
                /* over to main task.                                           */

                sprintf(logging_buffer, "cmd_print: len: 0x%x", packet->data_len);
                Serial.println(logging_buffer);
                if (packet->data_len == sizeof(data->params)) {
                    memcpy(data->params, packet->data, packet->data_len);
                    ctrl.status_task &= ~STATUS_UNPROC;
                    ctrl.status_task |= STATUS_FULL;
                    data->busy_cnt = 0;
                    data->printed = 1;
                } else {
                    Serial.println("Parameter buffer overrun");
                    ctrl.status_task |= STATUS_ERR0;
                }
                break;
            case cmd_inquiry:
                /* Status inquiry is handled completely in ISR. */

                sprintf(logging_buffer, "cmd_inquiry: len: 0x%x", packet->data_len);
                Serial.println(logging_buffer);
                break;
            case cmd_init:
            case cmd_break:
                /* Hard reset the whole transaction. This is the only way to     */
                /* clear possible error states from the task's status indicator. */

                Serial.println("cmd_init/break");
                memset(data, 0x0, sizeof(*data));
                ctrl.status_task = 0;
                break;
        }

        /* Hand ownership of command packet back to ISR, move read index */
        /* to next element in ring.                                      */
        mb();
        packet->owner = own_isr;
        mb();

        ++ctrl.rd_idx;
        ctrl.rd_idx %= ARRAY_SIZE(ctrl.packets);
    }
}

void draw_tile(uint8_t *data, uint8_t *image)
{
    int x, y;
    uint16_t *tmp;

    for(y = 0; y < 8; ++y){
        tmp = (uint16_t *) &image[40 * y];
        *tmp = 0;
        for(x = 7; x >= 0; --x){
            *tmp |= ((data[1] >> x) & 0x1) << (2 * x + 1);
            *tmp |= ((data[0] >> x) & 0x1) << (2 * x);
        }
        *tmp = ~htons(*tmp);

        data += 2;
    }
}

esp_err_t draw_bitmap(struct pr_data *data) {
    size_t w, h, x, y;
    uint8_t *tile_data, *tile_dest;
    unsigned int tile;
    esp_err_t result;

    result = ESP_OK;

    w = IMG_WIDTH;
    h = (data->data_len * 4) / IMG_WIDTH;

    if(h > IMG_HEIGHT){
        ESP_LOGE(TAG, "[%s] Invalid image hight: %d", __func__, h);
        result = ESP_ERR_INVALID_SIZE;
        goto err_out;
    }

    memset(render_buff, 0x0, sizeof(render_buff));

    for(tile = 0; tile < (w * h / 64); ++tile){
        x = tile % 20;
        y = 8 * (tile / 20);

        tile_data = &(data->data[tile * 16]);
        tile_dest = &(render_buff[(x * 2) + (y * 40)]);
        draw_tile(tile_data, tile_dest);
    }

    memmove(data->data, render_buff, data->data_len);
    data->width = w;
    data->height = h;

    err_out:
    return result;
}
