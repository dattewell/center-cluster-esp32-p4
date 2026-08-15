/**
 * @file lv_conf.h
 * Configuration file for v9.5.0
 *
 * ACTIVE as of 2026-08-15. Previously this file was the untouched v8.4.0
 * template and had NO effect on the build - CONFIG_LV_CONF_SKIP=y (LVGL's
 * Kconfig default) meant lv_conf_internal.h never included it at all, so
 * every setting below was silently ignored in favor of Kconfig/sdkconfig.
 * Confirmed via readelf on the linked ELF at the time (a hand-edited
 * LV_MEM_SIZE here had zero effect on the actual compiled pool size).
 *
 * This file is a full rewrite against LVGL 9.5's current
 * lv_conf_template.h, with every value set to match what was actually
 * active via Kconfig at migration time - so flipping CONFIG_LV_CONF_SKIP=n
 * (see sdkconfig.defaults) changes the *mechanism* controlling config, not
 * the *behavior* of the already-tested build. lv_conf_internal.h's
 * #ifndef guards mean anything defined here now wins outright over the
 * matching CONFIG_LV_* Kconfig symbol; Kconfig values for anything this
 * file also defines become vestigial (harmless to leave set, but inert).
 *
 * LV_BUILD_DEMOS/LV_USE_DEMO_* and unused LV_FONT_MONTSERRAT_* sizes
 * (pruned 2026-08-15, see git log): both were inherited enabled from the
 * previous Kconfig state and cost flash for code/data this app never
 * exercises. Fonts kept: 14 (LV_FONT_DEFAULT), 38/40/48 (the only sizes
 * referenced anywhere under main/ - grep for lv_font_montserrat_ before
 * adding a size back rather than assuming).
 */

/* clang-format off */
#if 1 /* Set this to "1" to enable content */

#ifndef LV_CONF_H
#define LV_CONF_H

/* If you need to include anything here, do it inside the `__ASSEMBLY__` guard */
#if  0 && defined(__ASSEMBLY__)
#include "my_include.h"
#endif

/*====================
   COLOR SETTINGS
 *====================*/

/** Color depth: 1 (I1), 8 (L8), 16 (RGB565), 24 (RGB888), 32 (XRGB8888) */
#define LV_COLOR_DEPTH 16

/*=========================
   STDLIB WRAPPER SETTINGS
 *=========================*/

/** Possible values
 * - LV_STDLIB_BUILTIN:     LVGL's built in implementation
 * - LV_STDLIB_CLIB:        Standard C functions, like malloc, strlen, etc
 * - LV_STDLIB_MICROPYTHON: MicroPython implementation
 * - LV_STDLIB_RTTHREAD:    RT-Thread implementation
 * - LV_STDLIB_CUSTOM:      Implement the functions externally
 */
#define LV_USE_STDLIB_MALLOC    LV_STDLIB_BUILTIN

/** Possible values
 * - LV_STDLIB_BUILTIN:     LVGL's built in implementation
 * - LV_STDLIB_CLIB:        Standard C functions, like malloc, strlen, etc
 * - LV_STDLIB_MICROPYTHON: MicroPython implementation
 * - LV_STDLIB_RTTHREAD:    RT-Thread implementation
 * - LV_STDLIB_CUSTOM:      Implement the functions externally
 */
#define LV_USE_STDLIB_STRING    LV_STDLIB_BUILTIN

/** Possible values
 * - LV_STDLIB_BUILTIN:     LVGL's built in implementation
 * - LV_STDLIB_CLIB:        Standard C functions, like malloc, strlen, etc
 * - LV_STDLIB_MICROPYTHON: MicroPython implementation
 * - LV_STDLIB_RTTHREAD:    RT-Thread implementation
 * - LV_STDLIB_CUSTOM:      Implement the functions externally
 */
#define LV_USE_STDLIB_SPRINTF   LV_STDLIB_BUILTIN

#define LV_STDINT_INCLUDE       <stdint.h>
#define LV_STDDEF_INCLUDE       <stddef.h>
#define LV_STDBOOL_INCLUDE      <stdbool.h>
#define LV_INTTYPES_INCLUDE     <inttypes.h>
#define LV_LIMITS_INCLUDE       <limits.h>
#define LV_STDARG_INCLUDE       <stdarg.h>

#if LV_USE_STDLIB_MALLOC == LV_STDLIB_BUILTIN
    /** Size of memory available for `lv_malloc()` in bytes (>= 2kB)
     *
     * History (see git log for full detail): raised 64K -> 128K -> 192K ->
     * 200K across several rounds this project hit LV_ASSERT_HANDLER's
     * while(1) halt in lv_draw_add_task (LV_ASSERT_MALLOC tripping on a
     * NULL lv_malloc_zeroed() - this pool exhausted, not a crash). 200K
     * verified via readelf + real hardware boot (DEMO_MODE=0,
     * SD_FEATURES_ENABLED=1) as of 2026-08-15. This is internal SRAM
     * (CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY is off) - `idf.py size`
     * showed DIRAM already ~73% used at link time with this pool in place,
     * so there isn't much more headroom to grow it further without moving
     * it to PSRAM instead. If it recurs, use LV_USE_MEM_MONITOR (below)
     * to profile actual peak usage instead of guessing at bigger numbers.
     */
    #define LV_MEM_SIZE (200U * 1024U)          /**< [bytes] */

    /** Size of the memory expand for `lv_malloc()` in bytes */
    #define LV_MEM_POOL_EXPAND_SIZE 0

    /** Set an address for the memory pool instead of allocating it as a normal array. Can be in external SRAM too. */
    #define LV_MEM_ADR 0     /**< 0: unused*/
    /* Instead of an address give a memory allocator that will be called to get a memory pool for LVGL. E.g. my_malloc */
    #if LV_MEM_ADR == 0
        #undef LV_MEM_POOL_INCLUDE
        #undef LV_MEM_POOL_ALLOC
    #endif
#endif  /*LV_USE_STDLIB_MALLOC == LV_STDLIB_BUILTIN*/

/*====================
   HAL SETTINGS
 *====================*/

/** Default display refresh, input device read and animation step period. */
#define LV_DEF_REFR_PERIOD  33      /**< [ms] */

/** Default Dots Per Inch. Used to initialize default sizes such as widgets sized, style paddings.
 * (Not so important, you can adjust it to modify default sizes and spaces.) */
#define LV_DPI_DEF 130              /**< [px/inch] */

/*=================
 * OPERATING SYSTEM
 *=================*/
/** Select operating system to use. Possible options:
 * - LV_OS_NONE
 * - LV_OS_PTHREAD
 * - LV_OS_FREERTOS
 * - LV_OS_CMSIS_RTOS2
 * - LV_OS_RTTHREAD
 * - LV_OS_WINDOWS
 * - LV_OS_MQX
 * - LV_OS_SDL2
 * - LV_OS_CUSTOM */
#define LV_USE_OS   LV_OS_NONE

#if LV_USE_OS == LV_OS_CUSTOM
    #define LV_OS_CUSTOM_INCLUDE <stdint.h>
#endif
#if LV_USE_OS == LV_OS_FREERTOS
    #define LV_USE_FREERTOS_TASK_NOTIFY 1
#endif

/*========================
 * RENDERING CONFIGURATION
 *========================*/

/** Align stride of all layers and images to this bytes */
#define LV_DRAW_BUF_STRIDE_ALIGN                1

/** Align start address of draw_buf addresses to this bytes*/
#define LV_DRAW_BUF_ALIGN                       4

/** Using matrix for transformations.
 * Requirements:
 * - `LV_USE_MATRIX = 1`.
 * - Rendering engine needs to support 3x3 matrix transformations. */
#define LV_DRAW_TRANSFORM_USE_MATRIX            0

/* If a widget has `style_opa < 255` (not `bg_opa`, `text_opa` etc) or not NORMAL blend mode
 * it is buffered into a "simple" layer before rendering. The widget can be buffered in smaller chunks.
 * "Transformed layers" (if `transform_angle/zoom` are set) use larger buffers
 * and can't be drawn in chunks. */

/** The target buffer size for simple layer chunks. */
#define LV_DRAW_LAYER_SIMPLE_BUF_SIZE    (24 * 1024)    /**< [bytes]*/

/* Limit the max allocated memory for simple and transformed layers.
 * It should be at least `LV_DRAW_LAYER_SIMPLE_BUF_SIZE` sized but if transformed layers are also used
 * it should be enough to store the largest widget too (width x height x 4 area).
 * Set it to 0 to have no limit. */
#define LV_DRAW_LAYER_MAX_MEMORY 0  /**< No limit by default [bytes]*/

/** Stack size of drawing thread.
 * NOTE: If FreeType or ThorVG is enabled, it is recommended to set it to 32KB or more.
 */
#define LV_DRAW_THREAD_STACK_SIZE    (8 * 1024)         /**< [bytes]*/

/** Thread priority of the drawing task. */
#define LV_DRAW_THREAD_PRIO LV_THREAD_PRIO_HIGH

#define LV_USE_DRAW_SW 1
#if LV_USE_DRAW_SW == 1
    #define LV_DRAW_SW_SUPPORT_RGB565       1
    #define LV_DRAW_SW_SUPPORT_RGB565_SWAPPED       1
    #define LV_DRAW_SW_SUPPORT_RGB565A8     1
    #define LV_DRAW_SW_SUPPORT_RGB888       1
    #define LV_DRAW_SW_SUPPORT_XRGB8888     1
    #define LV_DRAW_SW_SUPPORT_ARGB8888     1
    #define LV_DRAW_SW_SUPPORT_ARGB8888_PREMULTIPLIED 1
    #define LV_DRAW_SW_SUPPORT_L8           1
    #define LV_DRAW_SW_SUPPORT_AL88         1
    #define LV_DRAW_SW_SUPPORT_A8           1
    #define LV_DRAW_SW_SUPPORT_I1           1

    #define LV_DRAW_SW_I1_LUM_THRESHOLD 127

    /** Set number of draw units.
     *  - > 1 requires operating system to be enabled in `LV_USE_OS`.
     *  - > 1 means multiple threads will render the screen in parallel. */
    #define LV_DRAW_SW_DRAW_UNIT_CNT    1

    #define LV_USE_DRAW_ARM2D_SYNC      0
    #define LV_USE_NATIVE_HELIUM_ASM    0

    /**
     * - 0: Use a simple renderer capable of drawing only simple rectangles with gradient, images, text, and straight lines only.
     * - 1: Use a complex renderer capable of drawing rounded corners, shadow, skew lines, and arcs too. */
    #define LV_DRAW_SW_COMPLEX          1

    #if LV_DRAW_SW_COMPLEX == 1
        #define LV_DRAW_SW_SHADOW_CACHE_SIZE 0
        #define LV_DRAW_SW_CIRCLE_CACHE_SIZE 4
    #endif

    #define  LV_USE_DRAW_SW_ASM     LV_DRAW_SW_ASM_NONE

    #if LV_USE_DRAW_SW_ASM == LV_DRAW_SW_ASM_CUSTOM
        #define  LV_DRAW_SW_ASM_CUSTOM_INCLUDE ""
    #endif

    #define LV_USE_DRAW_SW_COMPLEX_GRADIENTS    0

#endif

/* No GPU on this hardware (ESP32-P4 software rendering via LV_USE_DRAW_SW
 * above; the Waveshare BSP's MIPI-DSI panel driver handles the display
 * interface separately, outside LVGL's own GPU backends below). */
#define LV_USE_NEMA_GFX 0
#define LV_USE_PXP 0
#define LV_USE_G2D 0
#define LV_USE_DRAW_DAVE2D 0
#define LV_USE_DRAW_SDL 0
#define LV_USE_DRAW_VG_LITE 0
#define LV_USE_DRAW_DMA2D 0
#define LV_USE_DRAW_OPENGLES 0
#define LV_USE_PPA  0
#define LV_USE_DRAW_EVE 0
#define LV_USE_DRAW_NANOVG 0

/*=======================
 * FEATURE CONFIGURATION
 *=======================*/

/*-------------
 * Logging
 *-----------*/

/** Enable log module.
 * This project's own ESP_LOGx calls (main.c) go through ESP-IDF's separate
 * esp_log system regardless of this setting - this only controls LVGL's
 * own internal diagnostic tracing (widget creation, timers, layout, etc),
 * which has been off (matching Kconfig's default) for the whole time this
 * build has actually been tested. */
#define LV_USE_LOG 0
#if LV_USE_LOG
    #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
    #define LV_LOG_PRINTF 0
    #define LV_LOG_USE_TIMESTAMP 1
    #define LV_LOG_USE_FILE_LINE 1
    #define LV_LOG_TRACE_MEM        1
    #define LV_LOG_TRACE_TIMER      1
    #define LV_LOG_TRACE_INDEV      1
    #define LV_LOG_TRACE_DISP_REFR  1
    #define LV_LOG_TRACE_EVENT      1
    #define LV_LOG_TRACE_OBJ_CREATE 1
    #define LV_LOG_TRACE_LAYOUT     1
    #define LV_LOG_TRACE_ANIM       1
    #define LV_LOG_TRACE_CACHE      1
#endif  /*LV_USE_LOG*/

/*-------------
 * Asserts
 *-----------*/

#define LV_USE_ASSERT_NULL          1   /**< Check if the parameter is NULL. (Very fast, recommended) */
#define LV_USE_ASSERT_MALLOC        1   /**< Checks is the memory is successfully allocated or no. (Very fast, recommended) */
#define LV_USE_ASSERT_STYLE         0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ           0

/** Add a custom handler when assert happens e.g. to restart MCU.
 * Deliberate halt-on-failure, not a bug: LV_USE_ASSERT_MALLOC above means a
 * pool-exhausted lv_malloc_zeroed() trips this instead of returning NULL
 * and corrupting state. See LV_MEM_SIZE comment for the incident history
 * this caught. */
#define LV_ASSERT_HANDLER_INCLUDE <stdint.h>
#define LV_ASSERT_HANDLER while(1);     /**< Halt by default */

/*-------------
 * Debug
 *-----------*/

#define LV_USE_REFR_DEBUG 0
#define LV_USE_LAYER_DEBUG 0
#define LV_USE_PARALLEL_DRAW_DEBUG 0

/*-------------
 * Others
 *-----------*/

#define LV_ENABLE_GLOBAL_CUSTOM 0
#if LV_ENABLE_GLOBAL_CUSTOM
    #define LV_GLOBAL_CUSTOM_INCLUDE <stdint.h>
#endif

#define LV_CACHE_DEF_SIZE       0
#define LV_IMAGE_HEADER_CACHE_DEF_CNT 0
#define LV_GRADIENT_MAX_STOPS   2

/** Adjust color mix functions rounding - 128 (round up from half) matches
 * what's been active via Kconfig (CONFIG_LV_COLOR_MIX_ROUND_OFS=128), not
 * the template's own default of 0. */
#define LV_COLOR_MIX_ROUND_OFS  128

#define LV_OBJ_STYLE_CACHE      0
#define LV_USE_OBJ_ID           0
#define LV_USE_OBJ_NAME         0
#define LV_OBJ_ID_AUTO_ASSIGN   LV_USE_OBJ_ID
#define LV_USE_OBJ_ID_BUILTIN   1
#define LV_USE_OBJ_PROPERTY 0
#define LV_USE_OBJ_PROPERTY_NAME 1
#define LV_USE_GESTURE_RECOGNITION 0

/*=====================
 *  COMPILER SETTINGS
 *====================*/

#define LV_BIG_ENDIAN_SYSTEM 0
#define LV_ATTRIBUTE_TICK_INC
#define LV_ATTRIBUTE_TIMER_HANDLER
#define LV_ATTRIBUTE_FLUSH_READY
#define LV_ATTRIBUTE_MEM_ALIGN_SIZE 1
#define LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_LARGE_CONST
#define LV_ATTRIBUTE_LARGE_RAM_ARRAY

/* Empty by default - CONFIG_LV_ATTRIBUTE_FAST_MEM_USE_IRAM is deliberately
 * OFF (see sdkconfig.defaults): enabling it injects -DLV_ATTRIBUTE_FAST_MEM=
 * IRAM_ATTR at the esp.cmake level, which was the trigger for the
 * --enable-non-contiguous-regions linker failure on this chip's rev<3.0
 * SRAM layout. Leave this empty; control IRAM placement via that Kconfig
 * symbol, not here. */
#define LV_ATTRIBUTE_FAST_MEM

#define LV_EXPORT_CONST_INT(int_value) struct _silence_gcc_warning
#define LV_ATTRIBUTE_EXTERN_DATA
#define LV_USE_FLOAT            0
#define LV_USE_MATRIX           0
#ifndef LV_USE_PRIVATE_API
    #define LV_USE_PRIVATE_API  0
#endif

/*==================
 *   FONT USAGE
 *===================*/

/* Only the sizes actually referenced anywhere under main/ are enabled -
 * confirmed via `grep -rhoE "lv_font_montserrat_[0-9]+" main/`, not assumed.
 * 14 is LV_FONT_DEFAULT (below) and ui_OtherData.c's ui_AFRStatusGood; 38 is
 * ui_RPMAFR.c's ui_AFRStatus; 40 is ui_OtherData.c/ui_RPMAFR.c's ui_TimeV/
 * ui_TimeV2; 48 is ui_OtherData.c's ui_AFRStatusBad. Re-run that grep before
 * adding a size back rather than guessing. */
#define LV_FONT_MONTSERRAT_8  0
#define LV_FONT_MONTSERRAT_10 0
#define LV_FONT_MONTSERRAT_12 0
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 0
#define LV_FONT_MONTSERRAT_18 0
#define LV_FONT_MONTSERRAT_20 0
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 0
#define LV_FONT_MONTSERRAT_26 0
#define LV_FONT_MONTSERRAT_28 0
#define LV_FONT_MONTSERRAT_30 0
#define LV_FONT_MONTSERRAT_32 0
#define LV_FONT_MONTSERRAT_34 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_38 1
#define LV_FONT_MONTSERRAT_40 1
#define LV_FONT_MONTSERRAT_42 0
#define LV_FONT_MONTSERRAT_44 0
#define LV_FONT_MONTSERRAT_46 0
#define LV_FONT_MONTSERRAT_48 1

#define LV_FONT_MONTSERRAT_28_COMPRESSED    0
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW    0
#define LV_FONT_SOURCE_HAN_SANS_SC_14_CJK   0
#define LV_FONT_SOURCE_HAN_SANS_SC_16_CJK   0

#define LV_FONT_UNSCII_8  0
#define LV_FONT_UNSCII_16 0

#define LV_FONT_CUSTOM_DECLARE

/** Always set a default font */
#define LV_FONT_DEFAULT &lv_font_montserrat_14

/** Kconfig had this on (CONFIG_LV_FONT_FMT_TXT_LARGE=y); template default is 0. */
#define LV_FONT_FMT_TXT_LARGE 1

/** Kconfig had this on (CONFIG_LV_USE_FONT_COMPRESSED=y); template default is 0. */
#define LV_USE_FONT_COMPRESSED 1

#define LV_USE_FONT_PLACEHOLDER 1

/*=================
 *  TEXT SETTINGS
 *=================*/

#define LV_TXT_ENC LV_TXT_ENC_UTF8

/** Matches Kconfig's active value (CONFIG_LV_TXT_BREAK_CHARS), which lacks
 * the closing `]` the template default includes - preserved as-is rather
 * than silently "corrected" back to the template. */
#define LV_TXT_BREAK_CHARS " ,.;:-_)}"

#define LV_TXT_LINE_BREAK_LONG_LEN 0
#define LV_TXT_LINE_BREAK_LONG_PRE_MIN_LEN 3
#define LV_TXT_LINE_BREAK_LONG_POST_MIN_LEN 3

#define LV_USE_BIDI 0
#if LV_USE_BIDI
    #define LV_BIDI_BASE_DIR_DEF LV_BASE_DIR_AUTO
#endif

#define LV_USE_ARABIC_PERSIAN_CHARS 0
#define LV_TXT_COLOR_CMD "#"

/*==================
 * WIDGETS
 *================*/

#define LV_WIDGETS_HAS_DEFAULT_VALUE  1

#define LV_USE_ANIMIMG    1
#define LV_USE_ARC        1
#define LV_USE_ARCLABEL   1
#define LV_USE_BAR        1
#define LV_USE_BUTTON     1
#define LV_USE_BUTTONMATRIX  1

#define LV_USE_CALENDAR   1
#if LV_USE_CALENDAR
    #define LV_CALENDAR_WEEK_STARTS_MONDAY 0
    #if LV_CALENDAR_WEEK_STARTS_MONDAY
        #define LV_CALENDAR_DEFAULT_DAY_NAMES {"Mo", "Tu", "We", "Th", "Fr", "Sa", "Su"}
    #else
        #define LV_CALENDAR_DEFAULT_DAY_NAMES {"Su", "Mo", "Tu", "We", "Th", "Fr", "Sa"}
    #endif
    #define LV_CALENDAR_DEFAULT_MONTH_NAMES {"January", "February", "March",  "April", "May",  "June", "July", "August", "September", "October", "November", "December"}
    #define LV_USE_CALENDAR_HEADER_ARROW 1
    #define LV_USE_CALENDAR_HEADER_DROPDOWN 1
    #define LV_USE_CALENDAR_CHINESE 0
#endif  /*LV_USE_CALENDAR*/

#define LV_USE_CANVAS     1
#define LV_USE_CHART      1
#define LV_USE_CHECKBOX   1
#define LV_USE_DROPDOWN   1
#define LV_USE_IMAGE      1
#define LV_USE_IMAGEBUTTON     1
#define LV_USE_KEYBOARD   1

#define LV_USE_LABEL      1
#if LV_USE_LABEL
    #define LV_LABEL_TEXT_SELECTION 1
    #define LV_LABEL_LONG_TXT_HINT 1
    #define LV_LABEL_WAIT_CHAR_COUNT 3
#endif

#define LV_USE_LED        1
#define LV_USE_LINE       1
#define LV_USE_LIST       1
#define LV_USE_LOTTIE     0
#define LV_USE_MENU       1
#define LV_USE_MSGBOX     1
#define LV_USE_ROLLER     1
#define LV_USE_SCALE      1
#define LV_USE_SLIDER     1

#define LV_USE_SPAN       1
#if LV_USE_SPAN
    #define LV_SPAN_SNIPPET_STACK_SIZE 64
#endif

#define LV_USE_SPINBOX    1
#define LV_USE_SPINNER    1
#define LV_USE_SWITCH     1
#define LV_USE_TABLE      1
#define LV_USE_TABVIEW    1

#define LV_USE_TEXTAREA   1
#if LV_USE_TEXTAREA != 0
    #define LV_TEXTAREA_DEF_PWD_SHOW_TIME 1500
#endif

#define LV_USE_TILEVIEW   1
#define LV_USE_WIN        1
#define LV_USE_3DTEXTURE  0

/*==================
 * THEMES
 *==================*/

#define LV_USE_THEME_DEFAULT 1
#if LV_USE_THEME_DEFAULT
    #define LV_THEME_DEFAULT_DARK 0
    #define LV_THEME_DEFAULT_GROW 1
    #define LV_THEME_DEFAULT_TRANSITION_TIME 80
#endif /*LV_USE_THEME_DEFAULT*/

#define LV_USE_THEME_SIMPLE 1
#define LV_USE_THEME_MONO 1

/*==================
 * LAYOUTS
 *==================*/

#define LV_USE_FLEX 1
#define LV_USE_GRID 1

/*====================
 * 3RD PARTS LIBRARIES
 *====================*/

/* Filesystem drivers / image decoders / vector graphics libs: none of these
 * are used - the SD card debug log and crash-dump relay (main.c) go
 * through plain POSIX fopen()/fwrite() via ESP-IDF's VFS directly, not
 * LVGL's own lv_fs abstraction, so LV_USE_FS_FATFS etc. staying off here
 * doesn't affect that feature. */
#define LV_FS_DEFAULT_DRIVER_LETTER '\0'
#define LV_USE_FS_STDIO 0
#define LV_USE_FS_POSIX 0
#define LV_USE_FS_WIN32 0
#define LV_USE_FS_FATFS 0
#define LV_USE_FS_MEMFS 0
#define LV_USE_FS_LITTLEFS 0
#define LV_USE_FS_ARDUINO_ESP_LITTLEFS 0
#define LV_USE_FS_ARDUINO_SD 0
#define LV_USE_FS_UEFI 0
#define LV_USE_FS_FROGFS 0

#define LV_USE_LODEPNG 0
#define LV_USE_LIBPNG 0
#define LV_USE_BMP 0
#define LV_USE_TJPGD 0
#define LV_USE_LIBJPEG_TURBO 0
#define LV_USE_LIBWEBP 0
#define LV_USE_GIF 0
#define LV_USE_GSTREAMER 0
#define LV_BIN_DECODER_RAM_LOAD 0
#define LV_USE_RLE 0
#define LV_USE_QRCODE 0
#define LV_USE_BARCODE 0
#define LV_USE_FREETYPE 0
#define LV_USE_TINY_TTF 0
#define LV_USE_RLOTTIE 0
#define LV_USE_GLTF  0
#define LV_USE_VECTOR_GRAPHIC  0
#define LV_USE_THORVG_INTERNAL 0
#define LV_USE_THORVG_EXTERNAL 0
#define LV_USE_NANOVG 0
#define LV_USE_LZ4_INTERNAL  0
#define LV_USE_LZ4_EXTERNAL  0
#define LV_USE_SVG 0
#define LV_USE_SVG_ANIMATION 0
#define LV_USE_SVG_DEBUG 0
#define LV_USE_FFMPEG 0

/*==================
 * OTHERS
 *==================*/

#define LV_USE_SNAPSHOT 0

/* Live FPS/CPU-idle + LVGL memory-pool overlay - both off, matching current
 * Kconfig state. Flip LV_USE_SYSMON + LV_USE_PERF_MONITOR/LV_USE_MEM_MONITOR
 * to 1 here for a real, on-screen measurement instead of guessing if
 * LV_MEM_SIZE needs adjusting again. */
#define LV_USE_SYSMON   0
#if LV_USE_SYSMON
    #define LV_SYSMON_GET_IDLE lv_os_get_idle_percent
    #define LV_SYSMON_PROC_IDLE_AVAILABLE 0
    #if LV_SYSMON_PROC_IDLE_AVAILABLE
        #define LV_SYSMON_GET_PROC_IDLE lv_os_get_proc_idle_percent
    #endif
    #define LV_USE_PERF_MONITOR 0
    #if LV_USE_PERF_MONITOR
        #define LV_USE_PERF_MONITOR_POS LV_ALIGN_BOTTOM_RIGHT
        #define LV_USE_PERF_MONITOR_LOG_MODE 0
    #endif
    #define LV_USE_MEM_MONITOR 0
    #if LV_USE_MEM_MONITOR
        #define LV_USE_MEM_MONITOR_POS LV_ALIGN_BOTTOM_LEFT
    #endif
#endif /*LV_USE_SYSMON*/

#define LV_USE_PROFILER 0

#define LV_USE_MONKEY 0
#define LV_USE_GRIDNAV 0
#define LV_USE_FRAGMENT 0
#define LV_USE_IMGFONT 0
#define LV_USE_OBSERVER 1
#define LV_USE_IME_PINYIN 0
#define LV_USE_FILE_EXPLORER 0
#define LV_USE_FONT_MANAGER 0
#define LV_USE_TEST 0
#define LV_USE_TRANSLATION 0
#define LV_USE_COLOR_FILTER 0

/*==================
 * DEVICES
 *==================*/

/* None of LVGL's built-in device backends apply - display is the Waveshare
 * BSP's own MIPI-DSI panel driver (outside LVGL), touch is the BSP's GT911
 * driver, both wired up via esp_lvgl_port, not through any of these. */
#define LV_USE_SDL              0
#define LV_USE_X11              0
#define LV_USE_WAYLAND          0
#define LV_USE_LINUX_FBDEV      0
#define LV_USE_NUTTX    0
#define LV_USE_LINUX_DRM        0
#define LV_USE_TFT_ESPI         0
#define LV_USE_LOVYAN_GFX         0
#define LV_USE_EVDEV    0
#define LV_USE_LIBINPUT    0
#define LV_USE_ST7735        0
#define LV_USE_ST7789        0
#define LV_USE_ST7796        0
#define LV_USE_ILI9341       0
#define LV_USE_FT81X         0
#define LV_USE_NV3007        0
#if (LV_USE_ST7735 | LV_USE_ST7789 | LV_USE_ST7796 | LV_USE_ILI9341 | LV_USE_NV3007)
    #define LV_USE_GENERIC_MIPI 1
#else
    #define LV_USE_GENERIC_MIPI 0
#endif
#define LV_USE_RENESAS_GLCDC    0
#define LV_USE_ST_LTDC    0
#define LV_USE_NXP_ELCDIF   0
#define LV_USE_WINDOWS    0
#define LV_USE_UEFI 0
#define LV_USE_OPENGLES   0
#define LV_USE_GLFW   0
#define LV_USE_QNX              0
#define LV_USE_EXT_DATA   0

/*=====================
* BUILD OPTIONS
*======================*/

#define LV_BUILD_EXAMPLES 1

/** LVGL's own bundled demo apps (widgets/benchmark/stress/music) - this app
 * has its own SquareLine-generated UI and never calls any lv_demo_*()
 * function, so none of this needs to be compiled in. Was inherited enabled
 * from the previous Kconfig state; pruned 2026-08-15 (see git log). */
#define LV_BUILD_DEMOS 0

/*===================
 * DEMO USAGE
 ====================*/

#if LV_BUILD_DEMOS
    #define LV_USE_DEMO_WIDGETS 0
    #define LV_USE_DEMO_KEYPAD_AND_ENCODER 0
    #define LV_USE_DEMO_BENCHMARK 0
    #if LV_USE_DEMO_BENCHMARK
        #define LV_DEMO_BENCHMARK_ALIGNED_FONTS 0
    #endif
    #define LV_USE_DEMO_RENDER 0
    #define LV_USE_DEMO_STRESS 0
    #define LV_USE_DEMO_MUSIC 0
    #if LV_USE_DEMO_MUSIC
        #define LV_DEMO_MUSIC_SQUARE    0
        #define LV_DEMO_MUSIC_LANDSCAPE 0
        #define LV_DEMO_MUSIC_ROUND     0
        #define LV_DEMO_MUSIC_LARGE     0
        #define LV_DEMO_MUSIC_AUTO_PLAY 0
    #endif
    #define LV_USE_DEMO_VECTOR_GRAPHIC  0
    #define LV_USE_DEMO_GLTF            0
    #define LV_USE_DEMO_FLEX_LAYOUT     0
    #define LV_USE_DEMO_MULTILANG       0
    #define LV_USE_DEMO_EBIKE           0
    #if LV_USE_DEMO_EBIKE
        #define LV_DEMO_EBIKE_PORTRAIT  0
    #endif
    #define LV_USE_DEMO_HIGH_RES        0
    #define LV_USE_DEMO_SMARTWATCH      0
#endif /* LV_BUILD_DEMOS */

/*--END OF LV_CONF_H--*/

#endif /*LV_CONF_H*/

#endif /*End of "Content enable"*/
