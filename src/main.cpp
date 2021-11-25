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

#include <Arduino.h>
#include "GBPrinter.h"
#include "BluetoothSerial.h"

static QueueHandle_t my_image_queue;

BluetoothSerial SerialBT;

const uint8_t printerHeader[] = {0x1b, 0x40, // ESC @: initialize printer
                                 0x1b, 0x61, // ESC a: select justification
                                 0x01,       //     1: centered
                                 0x1f, 0x11, 0x02, 0x04};
const uint8_t blockMarker[] = {0x1d, 0x76, 0x30, 0x00, 0x30, 0x00};
const uint8_t printerFooter[] = {0x1f, 0x11, 0x08,
                                 0x1f, 0x11, 0x0e,
                                 0x1f, 0x11, 0x07,
                                 0x1f, 0x11, 0x09};
const uint8_t imagePadding[] = {0, 0, 0, 0};

const uint8_t bayerMatrix[2][2] = {
        {1, 2},
        {3, 1}
};


void print_image(struct pr_data *data) {
    uint8_t line_repeat;
    unsigned int line;
    uint8_t line_byte;
    uint8_t pixels;

    SerialBT.write(printerHeader, sizeof(printerHeader));

    for (line = 0; line < data->height; line++) {
        SerialBT.write(blockMarker, sizeof(blockMarker));
        SerialBT.write(0x02);  // tell the printer we're sending two lines at a time
        SerialBT.write(0x00);
        for (line_repeat = 0; line_repeat < 2; line_repeat++ ) {
            SerialBT.write(imagePadding, sizeof(imagePadding));
            for (line_byte = 0; line_byte < (IMG_WIDTH / 4); line_byte++) {
                uint8_t originalPixels = ~(data->data[(line*(IMG_WIDTH/4)) + line_byte]);

                // this nonsense applies some basic bayer dithering since the printer is not greyscale
                pixels = 0;
                pixels |= ((((originalPixels & 0xC0) >> 6) < bayerMatrix[((line_byte * 4)    ) % 2][(line * 2 + line_repeat) % 2]) ? 0x00 : 0x80);
                pixels |= ((((originalPixels & 0xC0) >> 6) < bayerMatrix[((line_byte * 4) + 1) % 2][(line * 2 + line_repeat) % 2]) ? 0x00 : 0x40);
                pixels |= ((((originalPixels & 0x30) >> 4) < bayerMatrix[((line_byte * 4) + 2) % 2][(line * 2 + line_repeat) % 2]) ? 0x00 : 0x20);
                pixels |= ((((originalPixels & 0x30) >> 4) < bayerMatrix[((line_byte * 4) + 3) % 2][(line * 2 + line_repeat) % 2]) ? 0x00 : 0x10);
                pixels |= ((((originalPixels & 0x0C) >> 2) < bayerMatrix[((line_byte * 4) + 4) % 2][(line * 2 + line_repeat) % 2]) ? 0x00 : 0x08);
                pixels |= ((((originalPixels & 0x0C) >> 2) < bayerMatrix[((line_byte * 4) + 5) % 2][(line * 2 + line_repeat) % 2]) ? 0x00 : 0x04);
                pixels |= ((((originalPixels & 0x03)     ) < bayerMatrix[((line_byte * 4) + 6) % 2][(line * 2 + line_repeat) % 2]) ? 0x00 : 0x02);
                pixels |= ((((originalPixels & 0x03)     ) < bayerMatrix[((line_byte * 4) + 7) % 2][(line * 2 + line_repeat) % 2]) ? 0x00 : 0x01);

                SerialBT.write(pixels);
            }
            SerialBT.write(imagePadding, sizeof(imagePadding));
        }
    }

    SerialBT.write(0x1b);
    SerialBT.write(0x64);                        // this is the line feed command
    SerialBT.write((data->params[1] & 0x0f)*2);  // we send the printer the same value we get from the game boy
                                                    // this allows for printing things that take multiple pages (e.g. pokedex entries)
    SerialBT.write(printerFooter, sizeof(printerFooter));
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    SerialBT.begin("gbprinter", true);
    bool connected = SerialBT.connect("T02");

    if(connected) {
        Serial.println("Connected Succesfully!");
    } else {
        while(!SerialBT.connected(10000)) {
            Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app.");
        }
    }

    my_image_queue = xQueueCreate(2, sizeof(struct pr_data *));
    if(my_image_queue == nullptr){
        Serial.println("[%s] Creating packet queue failed.");
    } else {
        Serial.println("Creating packet queue succeeded");
    }

    xTaskCreatePinnedToCore(
            initialize_printer,
            "initialize_printer",
            3072,
            my_image_queue,
            1,
            nullptr,
            1);
}

void loop() {
    struct pr_data *outputdata = nullptr;

    if (xQueueReceive(my_image_queue, &outputdata, portMAX_DELAY) == pdTRUE) {
        Serial.println("time to print!");
        draw_bitmap(outputdata);
        print_image(outputdata);
        free(outputdata);
    };
}