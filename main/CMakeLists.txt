# -------------------------------------------------------
# Collect protocol JSON files
# -------------------------------------------------------

file(GLOB CAN_PROTOCOL_JSON
    "canbus/protocols/*.json"
)

# -------------------------------------------------------
# Generate protocol list file
# -------------------------------------------------------

set(PROTOCOL_LIST_FILE ${CMAKE_CURRENT_BINARY_DIR}/protocol_list.c)

file(WRITE ${PROTOCOL_LIST_FILE}
"#include \"protocol_loader.h\"\n\n"
)

# Generate extern declarations
foreach(json_file ${CAN_PROTOCOL_JSON})

    get_filename_component(json_name ${json_file} NAME)

    string(REPLACE "." "_" json_symbol ${json_name})
    string(REPLACE "-" "_" json_symbol ${json_symbol})

    file(APPEND ${PROTOCOL_LIST_FILE}
        "extern const char _binary_${json_symbol}_start[];\n"
    )

endforeach()

file(APPEND ${PROTOCOL_LIST_FILE} "\nconst char* protocol_json_list[] = {\n")

# Generate protocol list
foreach(json_file ${CAN_PROTOCOL_JSON})

    get_filename_component(json_name ${json_file} NAME)

    string(REPLACE "." "_" json_symbol ${json_name})
    string(REPLACE "-" "_" json_symbol ${json_symbol})

    file(APPEND ${PROTOCOL_LIST_FILE}
        "    _binary_${json_symbol}_start,\n"
    )

endforeach()

file(APPEND ${PROTOCOL_LIST_FILE} "};\n")

file(APPEND ${PROTOCOL_LIST_FILE}
"const int protocol_json_count = sizeof(protocol_json_list)/sizeof(protocol_json_list[0]);\n"
)

# -------------------------------------------------------
# Component
# -------------------------------------------------------

idf_component_register(
    SRCS 
        "main.c"
        "ui/ui.c"
        "ui/ui_helpers.c"
        "ui/screens/ui_MainSpeedo.c"
        "ui/screens/ui_OtherData.c"
        "ui/screens/ui_BootScreen.c"        
        "ui/images/ui_img_bootscreen_png.c"
        "ui/images/ui_img_gauge_background_png.c"
        "ui/images/ui_img_gauge_needle_png.c"
        "ui/fonts/ui_font_DIN_1451__96.c"
        "ui/fonts/ui_font_DIN_1451_Mittelschrift_.c"
        "ui/boot/boot_screen.c"
        "odometer/odometer.c"
        "lap_timer/lap_timer.c"
        "gps_wrap/gps_wrapper.cpp"
        "canbus/canbus.c"
        "canbus/protocol_loader.c"
        ${PROTOCOL_LIST_FILE}

    INCLUDE_DIRS 
        "./ui"
        "./lap_timer"
        "./odometer"
        "./gps_wrap"
        "./canbus"
        "."

    EMBED_TXTFILES
        ${CAN_PROTOCOL_JSON}
)