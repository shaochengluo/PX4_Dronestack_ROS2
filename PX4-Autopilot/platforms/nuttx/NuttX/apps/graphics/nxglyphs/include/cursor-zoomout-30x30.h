/***************************************************************************
 * apps/graphics/nxglyphs/include/cursor-zoomout-30x30.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/video/rgbcolors.h>
#include <nuttx/nx/nxcursor.h>

#if CONFIG_NXWIDGETS_BPP == 8
#  define FGCOLOR1             RGB8_WHITE
#  define FGCOLOR2             RGB8_BLACK
#  define FGCOLOR3             RGB8_GRAY
#elif CONFIG_NXWIDGETS_BPP == 16
#  define FGCOLOR1             RGB16_WHITE
#  define FGCOLOR2             RGB16_BLACK
#  define FGCOLOR3             RGB16_GRAY
#elif CONFIG_NXWIDGETS_BPP == 24 || CONFIG_NXWIDGETS_BPP == 32
#  define FGCOLOR1             RGB24_WHITE
#  define FGCOLOR2             RGB24_BLACK
#  define FGCOLOR3             RGB24_GRAY
#else
#  error "Pixel depth not supported (CONFIG_NXWIDGETS_BPP)"
#endif

static const uint8_t g_zoomOutImage[] =
{
  0x00, 0x00, 0xaa, 0xa8, 0x00, 0x00, 0x00, 0x00,    /* Row 0 */
  0x00, 0x0a, 0xaa, 0xaa, 0x80, 0x00, 0x00, 0x00,    /* Row 1 */
  0x00, 0xaa, 0xd5, 0x5e, 0xa0, 0x00, 0x00, 0x00,    /* Row 2 */
  0x02, 0xad, 0x55, 0x55, 0xea, 0x00, 0x00, 0x00,    /* Row 3 */
  0x0a, 0x95, 0x55, 0x55, 0x7a, 0x00, 0x00, 0x00,    /* Row 4 */
  0x0a, 0x55, 0x55, 0x55, 0x5e, 0x80, 0x00, 0x00,    /* Row 5 */
  0x2b, 0x55, 0x55, 0x55, 0x56, 0x80, 0x00, 0x00,    /* Row 6 */
  0x29, 0x55, 0x55, 0x55, 0x55, 0xa0, 0x00, 0x00,    /* Row 7 */
  0xa5, 0x55, 0x55, 0x55, 0x55, 0xa0, 0x00, 0x00,    /* Row 8 */
  0xa5, 0x55, 0x55, 0x55, 0x55, 0xa0, 0x00, 0x00,    /* Row 9 */
  0xa5, 0x5e, 0xaa, 0xaa, 0x55, 0xa0, 0x00, 0x00,    /* Row 10 */
  0xa5, 0x5a, 0xaa, 0xaa, 0xd5, 0xa0, 0x00, 0x00,    /* Row 11 */
  0xa5, 0x55, 0x55, 0x55, 0x55, 0xa0, 0x00, 0x00,    /* Row 12 */
  0xa5, 0x55, 0x55, 0x55, 0x55, 0xa0, 0x00, 0x00,    /* Row 13 */
  0xad, 0x55, 0x55, 0x55, 0x55, 0xa0, 0x00, 0x00,    /* Row 14 */
  0x29, 0x55, 0x55, 0x55, 0x57, 0x80, 0x00, 0x00,    /* Row 15 */
  0x0a, 0x55, 0x55, 0x55, 0x56, 0x80, 0x00, 0x00,    /* Row 16 */
  0x0a, 0x95, 0x55, 0x55, 0x5a, 0xa0, 0x00, 0x00,    /* Row 17 */
  0x02, 0xa5, 0x55, 0x55, 0x6a, 0xa8, 0x00, 0x00,    /* Row 18 */
  0x00, 0xab, 0x55, 0x56, 0xa9, 0x6a, 0x00, 0x00,    /* Row 19 */
  0x00, 0x2a, 0xaa, 0xaa, 0xab, 0x5a, 0x80, 0x00,    /* Row 20 */
  0x00, 0x02, 0xaa, 0xa8, 0x0a, 0xd6, 0xa0, 0x00,    /* Row 21 */
  0x00, 0x00, 0x00, 0x00, 0x02, 0xb5, 0xa8, 0x00,    /* Row 22 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0xad, 0x7a, 0x00,    /* Row 23 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x5e, 0x80,    /* Row 24 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0xd7, 0xa0,    /* Row 25 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xb5, 0xe0,    /* Row 26 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xad, 0xa0,    /* Row 27 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0xa0,    /* Row 28 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x80,    /* Row 29 */
};

const struct nx_cursorimage_s g_zoomOutCursor =
{
  .size =
  {
    .w = 30,
    .h = 30
  },
  .color1 =
  {
    FGCOLOR1
  },
  .color2 =
  {
    FGCOLOR1
  },
  .color3 =
  {
    FGCOLOR3
  },
  .image  = g_zoomOutImage
};