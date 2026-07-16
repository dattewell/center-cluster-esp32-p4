
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
        "ui/images/ui_img_gauge_png.c"
        "ui/images/ui_img_gauge_needle_png.c"
        "ui/images/ui_img_needle_no_shadow65_png.c"
        "ui/fonts/ui_font_DIN_1451__96.c"
        "ui/fonts/ui_font_DIN_1451_Mittelschrift_.c"
        "ui/fonts/ui_font_DSGE7_Classic_80.c"
        "odometer/odometer.c"
        "gps_wrap/gps_wrapper.cpp"


    INCLUDE_DIRS 
        "./ui"
        "./odometer"
        "./gps_wrap"
        "."

)
