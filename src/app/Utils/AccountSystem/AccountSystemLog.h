/*
 * MIT License
 * Copyright (c) 2021 _VIFEXTech
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef __ACCOUNT_SYSTEM_LOG_H
#define __ACCOUNT_SYSTEM_LOG_H

#include "lvgl.h"

#define DATA_CENTER_USE_LOG 1

#if !defined(ARDUINO) && DATA_CENTER_USE_LOG
#include <stdio.h>
#  define DC_LOG_INFO(format, ...)      LV_LOG_INFO(format, ##__VA_ARGS__)
#  define DC_LOG_WARN(format, ...)      LV_LOG_WARN(format, ##__VA_ARGS__)
#  define DC_LOG_ERROR(format, ...)     LV_LOG_ERROR(format, ##__VA_ARGS__)
#else
#include<Arduino.h>
#  define DC_LOG_INFO(format, ...)      LV_LOG_INFO(format, ##__VA_ARGS__)
#  define DC_LOG_WARN(format, ...)      LV_LOG_WARN(format, ##__VA_ARGS__)
#  define DC_LOG_ERROR(format, ...)     LV_LOG_ERROR(format, ##__VA_ARGS__)
#endif

#endif
