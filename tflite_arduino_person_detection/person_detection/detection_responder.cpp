/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "main_functions.h"

#include "detection_responder.h"
#include "image_provider.h"
#include "model_settings.h"
#include "person_detect_model_data.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_log.h>
#include "esp_main.h"
#if DISPLAY_SUPPORT
#include "image_provider.h"
#include "esp_lcd.h"
static QueueHandle_t xQueueLCDFrame = NULL;
#endif

void RespondToDetection(tflite::ErrorReporter* error_reporter,
                        float person_score, float no_person_score) {
  int person_score_int = (person_score) * 100 + 0.5;
  int no_person_score_int = (no_person_score) * 100 + 0.5;

  // Print the scores to the console
  printf("Person score: %d%%, No person score: %d%%\n", person_score_int, no_person_score_int);

  // If you still want to use the error reporter for logging
  TF_LITE_REPORT_ERROR(error_reporter, "Person score: %d%%, No person score: %d%%",
                       person_score_int, no_person_score_int);
}

/*void RespondToDetection(tflite::ErrorReporter* error_reporter,
                        float person_score, float no_person_score) {
  int person_score_int = (person_score) * 100 + 0.5;
#if DISPLAY_SUPPORT
  if (xQueueLCDFrame == NULL) {
    xQueueLCDFrame = xQueueCreate(2, sizeof(struct lcd_frame));
    register_lcd(xQueueLCDFrame, NULL, false);
  }

  int color = 0x1f << 6; // red
  if (person_score_int < 60) { // treat score less than 60% as no person
    color = 0x3f; // green
  }
  app_lcd_color_for_detection(color);

  // display frame (freed by lcd task)
  lcd_frame_t *frame = (lcd_frame_t *) malloc(sizeof(lcd_frame_t));
  frame->width = 96 * 2;
  frame->height = 96 * 2;
  frame->buf = image_provider_get_display_buf();
  xQueueSend(xQueueLCDFrame, &frame, portMAX_DELAY);
  (void) no_person_score;
#else
  TF_LITE_REPORT_ERROR(error_reporter, "person score:%d%%, no person score %d%%",
                       person_score_int, 100 - person_score_int);
#endif
}*/
